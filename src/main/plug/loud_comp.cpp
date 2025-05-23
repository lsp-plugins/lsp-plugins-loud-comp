/*
 * Copyright (C) 2025 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2025 Vladimir Sadovnikov <sadko4u@gmail.com>
 *
 * This file is part of lsp-plugins-loud-comp
 * Created on: 3 авг. 2021 г.
 *
 * lsp-plugins-loud-comp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * lsp-plugins-loud-comp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with lsp-plugins-loud-comp. If not, see <https://www.gnu.org/licenses/>.
 */

#include <private/plugins/loud_comp.h>
#include <lsp-plug.in/common/alloc.h>
#include <lsp-plug.in/common/debug.h>
#include <lsp-plug.in/dsp/dsp.h>
#include <lsp-plug.in/dsp-units/units.h>
#include <lsp-plug.in/shared/id_colors.h>
#include <lsp-plug.in/shared/debug.h>
#include <lsp-plug.in/stdlib/math.h>

#include <generated/iso226/fletcher_munson.h>
#include <generated/iso226/robinson_dadson.h>
#include <generated/iso226/iso226-2003.h>
#include <generated/iso226/iso226-2023.h>

namespace lsp
{
    namespace plugins
    {
        static const freq_curve_t *freq_curves[]=
        {
            &iso226_2003_curve,
            &fletcher_munson_curve,
            &robinson_dadson_curve,
            &iso226_2023_curve,
        };

        static constexpr size_t EQ_SMOOTH_STEP      = 32;
        static constexpr size_t BUF_SIZE            = 0x200;
        static constexpr size_t NUM_CURVES          = (sizeof(freq_curves)/sizeof(freq_curve_t *));
        static constexpr float CURVE_APPROX         = 0.0005f * M_LN10;

        typedef struct approx_preset_t
        {
            uint8_t     nFilters;
            uint8_t     nSlope;
        } approx_preset_t;

        static const approx_preset_t approx_presets[] =
        {
            { 16, 1, },
            { 24, 2, },
            { 32, 3, },
            { 48, 4, },
            { 64, 5, }
        };

        //-------------------------------------------------------------------------
        // Plugin factory
        static const meta::plugin_t *plugins[] =
        {
            &meta::loud_comp_mono,
            &meta::loud_comp_stereo
        };

        static plug::Module *plugin_factory(const meta::plugin_t *meta)
        {
            return new loud_comp(meta, (meta == &meta::loud_comp_stereo) ? 2 : 1);
        }

        static plug::Factory factory(plugin_factory, plugins, 2);

        //-------------------------------------------------------------------------
        // Loudness compensator
        loud_comp::loud_comp(const meta::plugin_t *metadata, size_t channels): plug::Module(metadata)
        {
            nChannels       = channels;
            nMode           = meta::loud_comp_metadata::MODE_FFT;
            nCurve          = 0;
            nRank           = meta::loud_comp_metadata::FFT_RANK_MIN;
            nFilters        = 24;
            fGain           = 0.0f;
            fVolume         = -1.0f;
            fInLufs         = GAIN_AMP_M_INF_DB;
            fOutLufs        = GAIN_AMP_M_INF_DB;
            enGenerator     = GEN_PINK_M20LUFS;
            bBypass         = false;
            bRelative       = false;
            bReference      = false;
            bHClipOn        = false;
            fHClipLvl       = 1.0f;
            vChannels[0]    = NULL;
            vChannels[1]    = NULL;
            vTmpBuf         = NULL;
            vFreqApply      = NULL;
            vFreqMesh       = NULL;
            vAmpMesh        = NULL;
            bSyncMesh       = false;
            bSmooth         = false;
            pData           = NULL;
            pIDisplay       = NULL;

            dsp::fill(vOldGains, GAIN_AMP_0_DB, meta::loud_comp_metadata::FILTER_BANDS);
            dsp::fill(vGains, GAIN_AMP_0_DB, meta::loud_comp_metadata::FILTER_BANDS);

            pBypass         = NULL;
            pGain           = NULL;
            pMode           = NULL;
            pCurve          = NULL;
            pRank           = NULL;
            pApproximation  = NULL;
            pVolume         = NULL;
            pMesh           = NULL;
            pRelative       = NULL;
            pReference      = NULL;
            pGenerator      = NULL;
            pLufsIn         = NULL;
            pLufsOut        = NULL;
            pHClipOn        = NULL;
            pHClipRange     = NULL;
            pHClipReset     = NULL;
        }

        loud_comp::~loud_comp()
        {
            do_destroy();
        }

        void loud_comp::init(plug::IWrapper *wrapper, plug::IPort **ports)
        {
            // Pass wrapper
            plug::Module::init(wrapper, ports);

            // Initialize oscillator
            if (!sOsc.init())
                return;
            sOsc.set_amplitude(1.0f);
            sOsc.set_dc_offset(0.0f);
            sOsc.set_dc_reference(dspu::DC_ZERO);
            sOsc.set_duty_ratio(0.5f);
            sOsc.set_frequency(1000.0f);
            sOsc.set_oversampler_mode(dspu::OM_NONE);
            sOsc.set_phase(0.0f);
            sOsc.set_function(dspu::FG_SINE);

            sNoise.init();
            sNoise.set_generator(dspu::NG_GEN_LCG);
            sNoise.set_lcg_distribution(dspu::LCG_UNIFORM);
            sNoise.set_noise_color(dspu::NG_COLOR_PINK);

            // Initialize loudness meters
            if (sInMeter.init(nChannels) != STATUS_OK)
                return;
            if (sOutMeter.init(nChannels) != STATUS_OK)
                return;

            // Estimate temporary buffer size
            size_t sz_tmpbuf    = 0;
            for (size_t i=0; i<NUM_CURVES; ++i)
            {
                const freq_curve_t *c = freq_curves[i];
                if (sz_tmpbuf < c->hdots)
                    sz_tmpbuf       = c->hdots;
            }
            sz_tmpbuf           = align_size(sz_tmpbuf*sizeof(float), 0x100);

            // Compute size of data to allocate
            size_t sz_channel   = align_size(sizeof(channel_t), DEFAULT_ALIGN);
            size_t sz_fft       = (2 << meta::loud_comp_metadata::FFT_RANK_MAX) * sizeof(float);
            size_t sz_sync      = align_size(sizeof(float) * meta::loud_comp_metadata::CURVE_MESH_SIZE, DEFAULT_ALIGN);
            size_t sz_buf       = BUF_SIZE * sizeof(float);

            sz_tmpbuf           = lsp_max(sz_tmpbuf, sz_buf);

            // Total amount of data to allocate
            size_t sz_alloc     = (sz_channel + sz_buf*2) * nChannels + sz_fft + sz_sync*2 + sz_tmpbuf;
            uint8_t *ptr        = alloc_aligned<uint8_t>(pData, sz_alloc);
            if (ptr == NULL)
                return;

            // Allocate channels
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = advance_ptr_bytes<channel_t>(ptr, sz_channel);

                c->sDelay.construct();
                c->sBypass.construct();
                c->sProc.construct();
                c->sEqualizer.construct();
                c->sClipInd.construct();

                c->sDelay.init(1 << (meta::loud_comp_metadata::FFT_RANK_MAX - 1));
                c->sProc.init(meta::loud_comp_metadata::FFT_RANK_MAX);
                c->sProc.bind(process_callback, this, c);
                c->sProc.set_phase(0.5f * i);
                c->sEqualizer.init(meta::loud_comp_metadata::FILTER_BANDS, 0);
                c->sEqualizer.set_mode(dspu::EQM_IIR);

                c->vIn              = NULL;
                c->vOut             = NULL;
                c->vDry             = NULL;
                c->vBuffer          = NULL;
                c->fInLevel         = 0.0f;
                c->fOutLevel        = 0.0f;
                c->bHClip           = false;

                c->pIn              = NULL;
                c->pOut             = NULL;
                c->pMeterIn         = NULL;
                c->pMeterOut        = NULL;
                c->pHClipInd        = NULL;

                vChannels[i]        = c;
            }

            // Allocate buffers
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = vChannels[i];
                c->vDry             = advance_ptr_bytes<float>(ptr, sz_buf);
                c->vBuffer          = advance_ptr_bytes<float>(ptr, sz_buf);

                dsp::fill_zero(c->vBuffer, BUF_SIZE);
            }

            // Initialize buffers
            vFreqApply          = advance_ptr_bytes<float>(ptr, sz_fft);
            vFreqMesh           = advance_ptr_bytes<float>(ptr, sz_sync);
            vAmpMesh            = advance_ptr_bytes<float>(ptr, sz_sync);
            vTmpBuf             = advance_ptr_bytes<float>(ptr, sz_tmpbuf);

            lsp_trace("Binding ports...");
            size_t port_id      = 0;

            // Bind input audio ports
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i]->pIn);

            // Bind output audio ports
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i]->pOut);

            // Bind common ports
            BIND_PORT(pBypass);
            BIND_PORT(pGain);
            BIND_PORT(pMode);
            BIND_PORT(pCurve);
            BIND_PORT(pRank);
            BIND_PORT(pApproximation);
            BIND_PORT(pVolume);
            BIND_PORT(pReference);
            BIND_PORT(pGenerator);
            BIND_PORT(pHClipOn);
            BIND_PORT(pHClipRange);
            BIND_PORT(pHClipReset);
            BIND_PORT(pMesh);
            BIND_PORT(pRelative);
            BIND_PORT(pLufsIn);
            BIND_PORT(pLufsOut);

            // Bind input level meters
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i]->pMeterIn);

            // Bind hard clip indicators
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i]->pHClipInd);

            // Bind output level meters
            for (size_t i=0; i<nChannels; ++i)
                BIND_PORT(vChannels[i]->pMeterOut);

        }

        void loud_comp::destroy()
        {
            plug::Module::destroy();
            do_destroy();
        }

        void loud_comp::do_destroy()
        {
            sOsc.destroy();
            sInMeter.destroy();
            sOutMeter.destroy();

            if (pIDisplay != NULL)
            {
                pIDisplay->destroy();
                pIDisplay   = NULL;
            }

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = vChannels[i];
                if (c == NULL)
                    continue;

                c->sDelay.destroy();
                c->sProc.destroy();
                c->sEqualizer.destroy();
                vChannels[i]    = NULL;
            }

            vTmpBuf         = NULL;
            vFreqApply      = NULL;
            vFreqMesh       = NULL;

            if (pData != NULL)
            {
                free_aligned(pData);
                pData   = NULL;
            }
        }

        void loud_comp::ui_activated()
        {
            bSyncMesh           = true;
        }

        void loud_comp::update_sample_rate(long sr)
        {
            sOsc.set_sample_rate(sr);
            sNoise.set_sample_rate(sr);
            sInMeter.set_sample_rate(sr);
            sOutMeter.set_sample_rate(sr);

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c        = vChannels[i];

                // Update processor settings
                c->sBypass.init(sr);
                c->sClipInd.init(sr, 0.2f);
                c->sEqualizer.set_sample_rate(sr);
            }
        }

        loud_comp::generator_t loud_comp::decode_generator(size_t generator)
        {
            switch (generator)
            {
                case 0: return GEN_SINE;
                case 1: return GEN_PINK_M23LUFS;
                case 2: return GEN_PINK_M20LUFS;
                case 3: return GEN_PINK_M18LUFS;
                case 4: return GEN_PINK_M16LUFS;
                case 5: return GEN_PINK_M14LUFS;
                case 6: return GEN_PINK_M12LUFS;
                default: break;
            }
            return GEN_SINE;
        }

        float loud_comp::get_generator_amplitude(generator_t gen, bool stereo)
        {
            const float base = (stereo) ? GAIN_AMP_0_DB : GAIN_AMP_P_3_DB;

            switch (gen)
            {
                case GEN_PINK_M23LUFS: return base * GAIN_AMP_0_DB;
                case GEN_PINK_M20LUFS: return base * GAIN_AMP_P_3_DB;
                case GEN_PINK_M18LUFS: return base * GAIN_AMP_P_5_DB;
                case GEN_PINK_M16LUFS: return base * GAIN_AMP_P_7_DB;
                case GEN_PINK_M14LUFS: return base * GAIN_AMP_P_9_DB;
                case GEN_PINK_M12LUFS: return base * GAIN_AMP_P_11_DB;
                default: break;
            }
            return GAIN_AMP_M_INF_DB;
        }

        void loud_comp::update_settings()
        {
            const bool rst_clip     = pHClipReset->value() >= 0.5f;
            const bool bypass       = pBypass->value() >= 0.5f;
            const uint32_t mode     = pMode->value();
            const uint32_t curve    = pCurve->value();
            uint32_t rank           = meta::loud_comp_metadata::FFT_RANK_MIN + ssize_t(pRank->value());
            rank                    = lsp_limit(rank, meta::loud_comp_metadata::FFT_RANK_MIN, meta::loud_comp_metadata::FFT_RANK_MAX);
            const float volume      = pVolume->value();
            const bool relative     = pRelative->value() >= 0.5f;
            const bool reference    = pReference->value() >= 0.5f;
            const uint32_t iapprox  = pApproximation->value();
            const uint32_t filters  = approx_presets[iapprox].nFilters;
            const uint32_t slope    = approx_presets[iapprox].nSlope;

            // Need to update curve?
            if (mode != nMode)
            {
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c = vChannels[i];
                    c->sDelay.clear();
                    c->sProc.reset();
                    c->sEqualizer.reset();
                }
            }

            if (mode == meta::loud_comp_metadata::MODE_FFT)
            {
                if ((mode != nMode) || (curve != nCurve) || (rank != nRank) || (volume != fVolume))
                {
                    nMode               = mode;
                    nCurve              = curve;
                    nRank               = rank;
                    fVolume             = volume;
                    bSyncMesh           = true;
                    bSmooth             = false;

                    update_fft_curve();
                }
            }
            else // mode == meta::loud_comp_metadata::MODE_IIR
            {
                if ((mode != nMode) || (curve != nCurve) || (filters != nFilters) || (volume != fVolume))
                {
                    bSmooth             = (nMode == mode) && (nCurve == curve) && (nFilters == filters);
                    nMode               = mode;
                    nCurve              = curve;
                    nFilters            = filters;
                    fVolume             = volume;
                    bSyncMesh           = true;

                    update_iir_curve(slope);
                }
            }

            if (reference != bReference)
                sOsc.reset_phase_accumulator();
            if (bRelative != relative)
                bSyncMesh           = true;
            if ((bypass != bBypass) || (bSyncMesh))
                pWrapper->query_display_draw();

            fGain               = pGain->value();
            bHClipOn            = pHClipOn->value() >= 0.5f;
            bBypass             = bypass;
            bRelative           = relative;
            bReference          = reference;
            enGenerator         = decode_generator(pGenerator->value());

            // Set level of the noise generator
            const float noise_gain = get_generator_amplitude(enGenerator, nChannels > 1);
            sNoise.set_amplitude(noise_gain);

            if (bHClipOn)
            {
                const float range   = dspu::db_to_gain(pHClipRange->value());
                float min, max;
                if (nMode == meta::loud_comp_metadata::MODE_FFT)
                {
                    dsp::abs_minmax(vFreqApply, 2 << nRank, &min, &max);
                    fHClipLvl           = range * sqrtf(min * max);
                }
                else
                    fHClipLvl           = range * dsp::max(vGains, nFilters);
            }
            else
                fHClipLvl           = 1.0f;

            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c            = vChannels[i];
                const size_t latency    = (nMode == meta::loud_comp_metadata::MODE_FFT) ? c->sProc.latency() : c->sEqualizer.get_latency();

                c->sBypass.set_bypass(bypass);
                c->sProc.set_rank(rank);

                c->sDelay.set_delay(latency);
                if (rst_clip)
                    c->bHClip       = false;
            }
        }

        void loud_comp::generate_frequencies()
        {
            // Initialize list of frequencies
            const float norm        = logf(meta::loud_comp_metadata::FREQ_MAX/meta::loud_comp_metadata::FREQ_MIN) /
                                      (meta::loud_comp_metadata::CURVE_MESH_SIZE - 1);
            for (size_t i=0; i<meta::loud_comp_metadata::CURVE_MESH_SIZE; ++i)
                vFreqMesh[i]    = i * norm;
            dsp::exp1(vFreqMesh, meta::loud_comp_metadata::CURVE_MESH_SIZE);
            dsp::mul_k2(vFreqMesh, meta::loud_comp_metadata::FREQ_MIN, meta::loud_comp_metadata::CURVE_MESH_SIZE);
        }

        void loud_comp::update_fft_curve()
        {
            const freq_curve_t *c   = ((nCurve > 0) && (nCurve <= NUM_CURVES)) ? freq_curves[nCurve-1] : NULL;
            size_t fft_size         = 1 << nRank;
            size_t fft_csize        = (fft_size >> 1) + 1;

            if (c != NULL)
            {
                // Get the volume
                const float vol     = lsp_limit(fVolume - meta::loud_comp_metadata::PHONS_MIN, c->amin, c->amax) - c->amin;

                // Compute interpolation coefficients
                float range         = c->amax - c->amin;
                const float step    = range / (c->curves - 1);
                ssize_t nc          = vol / step;
                if (nc >= ssize_t(c->curves - 1))
                    --nc;
                const float k2      = CURVE_APPROX * (vol/step - nc);
                const float k1      = CURVE_APPROX - k2;

                // Interpolate curves to the temporary buffer, translate decibels to gain
                const int16_t *a    = c->data[nc];
                const int16_t *b    = c->data[nc+1];
                for (size_t i=0; i<c->hdots; ++i)
                    vTmpBuf[i]      = a[i] * k1 + b[i] * k2;

                dsp::exp1(vTmpBuf, c->hdots);

                // Compute frequency response
                ssize_t idx;
                float *v            = vFreqApply;
                const float rfmin   = 1.0f / c->fmin;
                const float kf      = float(fSampleRate) / float(fft_size);
                const float kdf     = (c->hdots - 1) / logf(c->fmax * rfmin);
                for (size_t i=0; i < fft_csize; ++i, v += 2)
                {
                    float f     = kf * i; // Target frequency
                    if (f <= c->fmin)
                        idx         = 0;
                    else if (f >= c->fmax)
                        idx         = c->hdots - 1;
                    else
                        idx             = logf(f * rfmin) * kdf;

                    f           = vTmpBuf[idx];
                    v[0]        = f;
                    v[1]        = f;
                }

                // Create reverse copy to complete the FFT response
                dsp::reverse2(&vFreqApply[fft_size+2], &vFreqApply[2], fft_size-2);

                if (fft_size == 256)
                {
                    for (size_t i=0; i<fft_size; ++i)
                        lsp_trace("i=%d; vfa={%.2f, %.2f}", int(i), vFreqApply[i*2], vFreqApply[i*2+1]);
                }
            }
            else
            {
                const float vol     = dspu::db_to_gain(fVolume);
                dsp::fill(vFreqApply, vol, fft_size * 2);
            }

            // Initialize list of frequencies
            generate_frequencies();

            // Build amp mesh
            const float xf          = float(fft_size) / float(fSampleRate);
            for (size_t i=0; i<meta::loud_comp_metadata::CURVE_MESH_SIZE; ++i)
            {
                size_t ix       = xf * vFreqMesh[i];
                if (ix > fft_csize)
                    ix                  = fft_csize;
                vAmpMesh[i]     = vFreqApply[ix << 1];
            }
        }

        void loud_comp::update_iir_curve(uint32_t slope)
        {
            const freq_curve_t *c   = ((nCurve > 0) && (nCurve <= NUM_CURVES)) ? freq_curves[nCurve-1] : NULL;
            float *freqs            = vTmpBuf;

            if (c != NULL)
            {
                // Generate the list of frequencies
                const float rfmin   = 1.0f / c->fmin;
                const float df      = logf(c->fmax * rfmin);
                const float kf      = df / (nFilters - 1);
                for (size_t i=0; i < nFilters; ++i)
                    freqs[i]            = c->fmin * expf((i + 0.5f) * kf);

                // Get the volume
                const float vol     = lsp_limit(fVolume - meta::loud_comp_metadata::PHONS_MIN, c->amin, c->amax) - c->amin;

                // Compute interpolation coefficients
                float range         = c->amax - c->amin;
                const float step    = range / (c->curves - 1);
                ssize_t nc          = vol / step;
                if (nc >= ssize_t(c->curves - 1))
                    --nc;
                const float k2      = CURVE_APPROX * (vol/step - nc);
                const float k1      = CURVE_APPROX - k2;

                // Compute frequency response
                ssize_t idx;
                const int16_t *a    = c->data[nc];
                const int16_t *b    = c->data[nc+1];
                const float kdf     = (c->hdots - 1) / df;

                for (size_t i=0; i < nFilters; ++i)
                {
                    const float f       = c->fmin * expf(i * kf);
                    if (f <= c->fmin)
                        idx                 = 0;
                    else if (f >= c->fmax)
                        idx                 = c->hdots - 1;
                    else
                        idx                 = logf(f * rfmin) * kdf;

                    vGains[i]           = expf(a[idx] * k1 + b[idx] * k2);
                }
            }
            else
            {
                const float df      = logf(SPEC_FREQ_MAX / SPEC_FREQ_MIN);
                const float kf      = df / (nFilters - 1);
                for (size_t i=0; i < nFilters; ++i)
                    freqs[i]            = SPEC_FREQ_MIN * expf((i + 0.5f) * kf);

                const float vol     = dspu::db_to_gain(fVolume);
                dsp::fill(vGains, vol, nFilters);
            }

            if (!bSmooth)
                dsp::copy(vOldGains, vGains, nFilters);

            // Configure equalizer
            dspu::filter_params_t fp;
            fp.nType            = dspu::FLT_NONE;
            fp.nSlope           = slope;
            fp.fFreq            = 0.0f;
            fp.fFreq2           = 0.0f;
            fp.fGain            = GAIN_AMP_0_DB;
            fp.fQuality         = 0.0f;

            for (size_t i=0; i < meta::loud_comp_metadata::FILTER_BANDS; ++i)
            {
                if (i >= nFilters) // Disabled filter
                    fp.nType        = dspu::FLT_NONE;
                else if (i == 0) // Low-shelf
                {
                    fp.nType        = dspu::FLT_BT_LRX_LOSHELF;
                    fp.fFreq        = freqs[0];
                    fp.fFreq2       = fp.fFreq;
                }
                else if (i < nFilters - 1) // Ladder-pass
                {
                    fp.nType        = dspu::FLT_BT_LRX_LADDERPASS;
                    fp.fFreq        = freqs[i-1];
                    fp.fFreq2       = freqs[i];
                }
                else // High-shelf
                {
                    fp.nType        = dspu::FLT_BT_LRX_HISHELF;
                    fp.fFreq        = freqs[i-1];
                    fp.fFreq2       = fp.fFreq;
                }

                fp.fGain        = vGains[i];

                for (size_t j=0; j<nChannels; ++j)
                    vChannels[j]->sEqualizer.set_params(i, &fp);
            }

            // Initialize list of frequencies
            generate_frequencies();

            // Compute the frequency response
            vChannels[0]->sEqualizer.freq_chart(vFreqApply, vFreqMesh, meta::loud_comp_metadata::CURVE_MESH_SIZE);
            dsp::pcomplex_mod(vAmpMesh, vFreqApply, meta::loud_comp_metadata::CURVE_MESH_SIZE);
        }

        void loud_comp::process_callback(void *object, void *subject, float *buf, size_t rank)
        {
            loud_comp *_this   = static_cast<loud_comp *>(object);
            channel_t *c            = static_cast<channel_t *>(subject);
            _this->process_spectrum(c, buf);
        }

        void loud_comp::process_spectrum(channel_t *c, float *buf)
        {
            // Apply spectrum changes to the FFT image
            size_t count = 2 << nRank;
            dsp::mul2(buf, vFreqApply, count);
        }

        void loud_comp::generate_signal(size_t samples)
        {
            float lvl;

            if (enGenerator == GEN_SINE)
                sOsc.process_overwrite(vChannels[0]->vOut, samples);
            else
                sNoise.process_overwrite(vChannels[0]->vOut, samples);

            vChannels[0]->fInLevel  = dsp::abs_max(vChannels[0]->vIn, samples) * fGain;
            vChannels[0]->fOutLevel = dsp::abs_max(vChannels[0]->vOut, samples);

            for (size_t i=1; i<nChannels; ++i)
            {
                dsp::copy(vChannels[i]->vOut, vChannels[0]->vOut, samples);
                vChannels[i]->fInLevel  = dsp::abs_max(vChannels[i]->vIn, samples) * fGain;
                vChannels[i]->fOutLevel = vChannels[0]->fOutLevel;
            }

            // Measure input and output loudness
            for (size_t offset = 0; offset < samples; )
            {
                size_t to_process   = lsp_min(samples - offset, BUF_SIZE);

                for (size_t i=0; i<nChannels; ++i)
                {
                    sInMeter.bind(i, NULL, vChannels[i]->vIn, 0);
                    sOutMeter.bind(i, NULL, vChannels[i]->vOut, 0);
                }

                sInMeter.process(vTmpBuf, to_process);
                lvl             = dsp::max(vTmpBuf, to_process);
                fInLufs         = lsp_max(fInLufs, lvl * fGain);

                sOutMeter.process(vTmpBuf, to_process);
                lvl             = dsp::max(vTmpBuf, to_process);
                fOutLufs        = lsp_max(fOutLufs, lvl);

                // Update sample counter
                offset             += to_process;
            }

            //---------------------------------------------------------------------
            // Perform clipping detection
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = vChannels[i];
                c->sClipInd.process(samples);
                if (bHClipOn)
                    c->pHClipInd->set_value((c->bHClip) ? 1.0f : 0.0f);
                else
                    c->pHClipInd->set_value((c->sClipInd.value()) ? 1.0f : 0.0f);
            }
        }

        void loud_comp::process_iir_equalizer(channel_t *c, size_t samples)
        {
            // Process the signal by the equalizer
            if (bSmooth)
            {
                dspu::filter_params_t fp;
                const float den   = 1.0f / samples;

                // In smooth mode, we need to update filter parameters for each sample
                for (size_t offset=0; offset<samples; )
                {
                    const size_t count          = lsp_min(samples - offset, EQ_SMOOTH_STEP);
                    const float k               = float(offset) * den;

                    // Tune filters
                    for (size_t j=0; j < nFilters; ++j)
                    {
                        c->sEqualizer.get_params(j, &fp);
                        fp.fGain                    = vOldGains[j] * expf(logf(vGains[j] / vOldGains[j])*k);
                        c->sEqualizer.set_params(j, &fp);
                    }

                    // Apply processing
                    c->sEqualizer.process(&c->vBuffer[offset], &c->vBuffer[offset], count);
                    offset                     += count;
                }
            }
            else
                c->sEqualizer.process(c->vBuffer, c->vBuffer, samples);
        }

        void loud_comp::process_audio(size_t samples)
        {
            float lvl;

            for (size_t offset = 0; offset < samples; )
            {
                size_t to_process   = lsp_min(samples - offset, BUF_SIZE);

                // Pre-process input signal
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = vChannels[i];

                    // Process the signal
                    c->sDelay.process(c->vDry, c->vIn, to_process);

                    // Apply input gain
                    dsp::mul_k3(c->vBuffer, c->vIn, fGain, to_process);
                    lvl             = dsp::abs_max(c->vBuffer, samples);
                    c->fInLevel     = lsp_max(c->fInLevel, lvl);
                }

                // Measure input loudness
                for (size_t i=0; i<nChannels; ++i)
                    sInMeter.bind(i, NULL, vChannels[i]->vBuffer, 0);
                sInMeter.process(vTmpBuf, to_process);
                lvl             = dsp::max(vTmpBuf, to_process);
                fInLufs         = lsp_max(fInLufs, lvl);

                // Do the loudness compensation
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = vChannels[i];

                    // Apply volume attenuation
                    if (nMode == meta::loud_comp_metadata::MODE_FFT)
                        c->sProc.process(c->vBuffer, c->vBuffer, to_process);
                    else
                        process_iir_equalizer(c, to_process);

                    // Perform clipping
                    lvl             = dsp::abs_max(c->vBuffer, to_process);
                    c->sClipInd.process(to_process);
                    bool clip       = lvl > fHClipLvl;
                    if (bHClipOn)
                    {
                        // Update buffer if clipping occurred
                        if (clip)
                        {
                            lvl             = fHClipLvl;
                            c->bHClip       = true;

                            dsp::limit1(c->vBuffer, -fHClipLvl, +fHClipLvl, to_process);
                        }

                        c->pHClipInd->set_value((c->bHClip) ? 1.0f : 0.0f);
                    }
                    else
                    {
                        if (clip)
                            c->sClipInd.blink();
                        c->pHClipInd->set_value((c->sClipInd.value()) ? 1.0f : 0.0f);
                    }
                    c->fOutLevel    = (c->fOutLevel < lvl) ? lvl : c->fOutLevel;

                    // Apply bypass
                    c->sBypass.process(c->vOut, c->vDry, c->vBuffer, to_process);
                }

                // Reset smooth flag for equalizer
                if (bSmooth)
                {
                    dsp::copy(vOldGains, vGains, nFilters);
                    bSmooth             = false;
                }

                // Measure output loudness
                for (size_t i=0; i<nChannels; ++i)
                    sOutMeter.bind(i, NULL, vChannels[i]->vBuffer, 0);
                sOutMeter.process(vTmpBuf, to_process);
                lvl                 = dsp::max(vTmpBuf, to_process);
                fOutLufs            = lsp_max(fOutLufs, lvl);

                // Update sample counter and pointers
                for (size_t i=0; i<nChannels; ++i)
                {
                    channel_t *c    = vChannels[i];

                    // Update pointers
                    c->vIn         += to_process;
                    c->vOut        += to_process;
                }
                offset         += to_process;
            }
        }

        void loud_comp::process(size_t samples)
        {
            //---------------------------------------------------------------------
            // Bind ports
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = vChannels[i];
                c->vIn          = c->pIn->buffer<float>();
                c->vOut         = c->pOut->buffer<float>();
                c->fInLevel     = 0.0f;
                c->fOutLevel    = 0.0f;
            }
            fInLufs         = GAIN_AMP_M_INF_DB;
            fOutLufs        = GAIN_AMP_M_INF_DB;

            //---------------------------------------------------------------------
            // Perform main processing
            if (bReference) // Reference signal generation
                generate_signal(samples);
            else // Audio processing
                process_audio(samples);

            //---------------------------------------------------------------------
            // Update meters
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c    = vChannels[i];
                c->pMeterIn->set_value(c->fInLevel);
                c->pMeterOut->set_value(c->fOutLevel);
            }
            pLufsIn->set_value(dspu::gain_to_lufs(fInLufs));
            const float out_lufs = dspu::gain_to_lufs(fOutLufs);
            pLufsOut->set_value(out_lufs);

            // Report latency
            set_latency(vChannels[0]->sDelay.get_delay());

            // Sync output mesh
            plug::mesh_t *mesh = pMesh->buffer<plug::mesh_t>();
            if ((bSyncMesh) && (mesh != NULL) && (mesh->isEmpty()))
            {
                // Output mesh data
                dsp::copy(mesh->pvData[0], vFreqMesh, meta::loud_comp_metadata::CURVE_MESH_SIZE);

                if (bRelative)
                {
                    float kf        = expf(-0.05f * M_LN10 * fVolume);
                    dsp::mul_k3(mesh->pvData[1], vAmpMesh, kf, meta::loud_comp_metadata::CURVE_MESH_SIZE);
                }
                else
                    dsp::copy(mesh->pvData[1], vAmpMesh, meta::loud_comp_metadata::CURVE_MESH_SIZE);
                mesh->data(2, meta::loud_comp_metadata::CURVE_MESH_SIZE);
                bSyncMesh   = false;
            }
        }

        bool loud_comp::inline_display(plug::ICanvas *cv, size_t width, size_t height)
        {
            // Check proportions
            if (height > (M_RGOLD_RATIO * width))
                height  = M_RGOLD_RATIO * width;

            // Init canvas
            if (!cv->init(width, height))
                return false;
            width   = cv->width();
            height  = cv->height();

            // Clear background
            bool bypass     = bBypass;
            bool relative   = bRelative;
            float volume    = fVolume;
            cv->set_color_rgb((bypass) ? CV_DISABLED : CV_BACKGROUND);
            cv->paint();

            // Draw axis
            if (relative)
            {
                // Draw axis
                cv->set_line_width(1.0);

                float zx    = 1.0f/SPEC_FREQ_MIN;
                float zy    = 1.0f/GAIN_AMP_M_12_DB;
                float dx    = width/(logf(SPEC_FREQ_MAX/SPEC_FREQ_MIN));
                float dy    = height/(logf(GAIN_AMP_M_12_DB/GAIN_AMP_P_72_DB));

                // Draw vertical lines
                cv->set_color_rgb(CV_YELLOW, 0.5f);
                for (float i=100.0f; i<SPEC_FREQ_MAX; i *= 10.0f)
                {
                    float ax = dx*(logf(i*zx));
                    cv->line(ax, 0, ax, height);
                }

                // Draw horizontal lines
                for (float i=GAIN_AMP_M_12_DB; i<GAIN_AMP_P_72_DB; i *= GAIN_AMP_P_12_DB)
                {
                    float ay = height + dy*(logf(i*zy));
                    if ((i >= (GAIN_AMP_0_DB - 1e-4)) && ((i <= (GAIN_AMP_0_DB + 1e-4))))
                        cv->set_color_rgb(CV_WHITE, 0.5f);
                    else
                        cv->set_color_rgb(CV_YELLOW, 0.5f);
                    cv->line(0, ay, width, ay);
                }

                // Allocate buffer: f, a(f), x, y
                pIDisplay           = core::IDBuffer::reuse(pIDisplay, 4, width);
                core::IDBuffer *b   = pIDisplay;
                if (b == NULL)
                    return false;

                float ni        = float(meta::loud_comp_metadata::CURVE_MESH_SIZE) / width; // Normalizing index
                volume          = expf(-0.05f * M_LN10 * volume);

                for (size_t j=0; j<width; ++j)
                {
                    size_t k        = j*ni;
                    b->v[0][j]      = vFreqMesh[k];
                    b->v[1][j]      = vAmpMesh[k];
                }

                dsp::mul_k2(b->v[1], volume, width);
                dsp::fill(b->v[2], 0.0f, width);
                dsp::fill(b->v[3], height, width);
                dsp::axis_apply_log1(b->v[2], b->v[0], zx, dx, width);
                dsp::axis_apply_log1(b->v[3], b->v[1], zy, dy, width);

                // Draw the mesh
                cv->set_color_rgb((bypass) ? CV_SILVER : CV_MESH);
                cv->set_line_width(2);
                cv->draw_lines(b->v[2], b->v[3], width);
            }
            else
            {
                // Draw axis
                cv->set_line_width(1.0);

                float zx    = 1.0f/SPEC_FREQ_MIN;
                float zy    = 1.0f/GAIN_AMP_M_96_DB;
                float dx    = width/(logf(SPEC_FREQ_MAX/SPEC_FREQ_MIN));
                float dy    = height/(logf(GAIN_AMP_M_96_DB/GAIN_AMP_P_12_DB));

                // Draw vertical lines
                cv->set_color_rgb(CV_YELLOW, 0.5f);
                for (float i=100.0f; i<SPEC_FREQ_MAX; i *= 10.0f)
                {
                    float ax = dx*(logf(i*zx));
                    cv->line(ax, 0, ax, height);
                }

                // Draw horizontal lines
                for (float i=GAIN_AMP_M_96_DB; i<GAIN_AMP_P_12_DB; i *= GAIN_AMP_P_12_DB)
                {
                    float ay = height + dy*(logf(i*zy));
                    if ((i >= (GAIN_AMP_0_DB - 1e-4)) && ((i <= (GAIN_AMP_0_DB + 1e-4))))
                        cv->set_color_rgb(CV_WHITE, 0.5f);
                    else
                        cv->set_color_rgb(CV_YELLOW, 0.5f);
                    cv->line(0, ay, width, ay);
                }

                // Allocate buffer: f, a(f), x, y
                pIDisplay           = core::IDBuffer::reuse(pIDisplay, 4, width);
                core::IDBuffer *b   = pIDisplay;
                if (b == NULL)
                    return false;

                float ni        = float(meta::loud_comp_metadata::CURVE_MESH_SIZE) / width; // Normalizing index
                for (size_t j=0; j<width; ++j)
                {
                    size_t k        = j*ni;
                    b->v[0][j]      = vFreqMesh[k];
                    b->v[1][j]      = vAmpMesh[k];
                }

                dsp::fill(b->v[2], 0.0f, width);
                dsp::fill(b->v[3], height, width);
                dsp::axis_apply_log1(b->v[2], b->v[0], zx, dx, width);
                dsp::axis_apply_log1(b->v[3], b->v[1], zy, dy, width);

                // Draw the volume line
                volume      = expf(0.05f * M_LN10 * fVolume);
                float vy    = height + dy * logf(volume * zy);
                cv->set_color_rgb((bypass) ? CV_GRAY : CV_GREEN, 0.5f);
                cv->line(0, vy, width, vy);

                // Draw the mesh
                cv->set_color_rgb((bypass) ? CV_SILVER : CV_MESH);
                cv->set_line_width(2);
                cv->draw_lines(b->v[2], b->v[3], width);
            }

            return true;
        }

        void loud_comp::dump(dspu::IStateDumper *v) const
        {
            plug::Module::dump(v);

            v->write("nChannels", nChannels);
            v->write("nMode", nMode);
            v->write("nCurve", nCurve);
            v->write("nRank", nRank);
            v->write("nApproximation", nFilters);
            v->write("fGain", fGain);
            v->write("fInLufs", fInLufs);
            v->write("fOutLufs", fOutLufs);
            v->write("bBypass", bBypass);
            v->write("bRelative", bRelative);
            v->write("bReference", bReference);
            v->write("bHClipOn", bHClipOn);
            v->write("fHClipLvl", fHClipLvl);
            v->begin_array("vChannels", vChannels, nChannels);
            for (size_t i=0; i<nChannels; ++i)
            {
                channel_t *c = vChannels[i];
                v->begin_object(c, sizeof(channel_t));
                {
                    v->write("vIn", c->vIn);
                    v->write("vOut", c->vOut);
                    v->write("vDry", c->vDry);
                    v->write("vBuffer", c->vBuffer);
                    v->write("fInLevel", c->fInLevel);
                    v->write("fOutLevel", c->fOutLevel);
                    v->write("bHClip", c->bHClip);

                    v->write_object("sBypass", &c->sBypass);
                    v->write_object("sDelay", &c->sDelay);
                    v->write_object("sProc", &c->sProc);
                    v->write_object("sEqualizer", &c->sEqualizer);
                    v->write_object("sClipInd", &c->sClipInd);

                    v->write("pIn", c->pIn);
                    v->write("pOut", c->pOut);
                    v->write("pMeterIn", c->pMeterIn);
                    v->write("pMeterOut", c->pMeterOut);
                    v->write("pHClipInd", c->pHClipInd);
                }
                v->end_object();
            }
            v->end_array();
            v->write("vTmpBuf", vTmpBuf);
            v->write("vFreqApply", vFreqApply);
            v->write("vFreqMesh", vFreqMesh);
            v->write("vAmpMesh", vAmpMesh);
            v->write("bSyncMesh", bSyncMesh);
            v->write("bSmooth", bSmooth);
            v->write("pIDisplay", pIDisplay);

            v->writev("vOldGains", vOldGains, meta::loud_comp_metadata::FILTER_BANDS);
            v->writev("vGains", vGains, meta::loud_comp_metadata::FILTER_BANDS);

            v->write_object("sOsc", &sOsc);
            v->write_object("sInMeter", &sInMeter);
            v->write_object("sOutMeter", &sOutMeter);

            v->write("pData", pData);

            v->write("pBypass", pBypass);
            v->write("pGain", pGain);
            v->write("pCurve", pCurve);
            v->write("pRank", pRank);
            v->write("pApproximation", pApproximation);
            v->write("pVolume", pVolume);
            v->write("pMesh", pMesh);
            v->write("pRelative", pRelative);
            v->write("pReference", pReference);
            v->write("pLufsIn", pLufsIn);
            v->write("pLufsOut", pLufsOut);
            v->write("pHClipOn", pHClipOn);
            v->write("pHClipRange", pHClipRange);
            v->write("pHClipReset", pHClipReset);
        }
    } /* namespace plugins */
} /* namespace lsp */


