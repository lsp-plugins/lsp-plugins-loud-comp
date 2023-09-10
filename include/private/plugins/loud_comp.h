/*
 * Copyright (C) 2023 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2023 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#ifndef PRIVATE_PLUGINS_LOUD_COMP_H_
#define PRIVATE_PLUGINS_LOUD_COMP_H_

#include <lsp-plug.in/plug-fw/plug.h>
#include <lsp-plug.in/plug-fw/core/IDBuffer.h>
#include <lsp-plug.in/dsp-units/ctl/Blink.h>
#include <lsp-plug.in/dsp-units/ctl/Bypass.h>
#include <lsp-plug.in/dsp-units/util/Delay.h>
#include <lsp-plug.in/dsp-units/util/Oscillator.h>
#include <lsp-plug.in/dsp-units/util/SpectralProcessor.h>

#include <private/meta/loud_comp.h>

namespace lsp
{
    namespace plugins
    {
        /**
         * Loudness Compensator plugin series
         */
        class loud_comp: public plug::Module
        {
            protected:
                typedef struct channel_t
                {
                    float                  *vIn;        // Input buffer
                    float                  *vOut;       // Output buffer
                    float                  *vDry;       // Dry signal
                    float                  *vBuffer;    // Temporary buffer
                    float                   fInLevel;   // Input level
                    float                   fOutLevel;  // Output level
                    bool                    bHClip;     // Hard-clip

                    dspu::Bypass            sBypass;    // Bypass
                    dspu::Delay             sDelay;     // Delay (for bypass)
                    dspu::SpectralProcessor sProc;      // Spectral processor
                    dspu::Blink             sClipInd;   // Clip blink

                    plug::IPort            *pIn;        // Input port
                    plug::IPort            *pOut;       // Output port
                    plug::IPort            *pMeterIn;   // Input meter
                    plug::IPort            *pMeterOut;  // Output meter
                    plug::IPort            *pHClipInd;  // Hard clipping indicator
                } channel_t;

            protected:
                size_t                  nChannels;      // Number of channels
                size_t                  nMode;          // Current curve mode
                size_t                  nRank;          // Current FFT rank
                float                   fGain;          // Input gain
                float                   fVolume;        // Volume
                bool                    bBypass;        // Bypass
                bool                    bRelative;      // Display relative curve instead of absolute
                bool                    bReference;     // Reference generator
                bool                    bHClipOn;       // Enable hard-clipping
                float                   fHClipLvl;      // Hard-clip threshold
                channel_t              *vChannels[2];   // Audio channels
                float                  *vTmpBuf;        // Temporary buffer for interpolating curve characteristics
                float                  *vFreqApply;     // Frequency response applied to the output signal
                float                  *vFreqMesh;      // List of frequencies for the mesh
                float                  *vAmpMesh;       // List of amplitudes for the mesh
                bool                    bSyncMesh;      // Synchronize mesh response with UI
                core::IDBuffer         *pIDisplay;      // Inline display buffer

                dspu::Oscillator        sOsc;           // Oscillator for reference sound

                uint8_t                *pData;          // Allocation data

                plug::IPort            *pBypass;        // Bypass
                plug::IPort            *pGain;          // Input gain
                plug::IPort            *pMode;          // Curve mode selector
                plug::IPort            *pRank;          // FFT rank selector
                plug::IPort            *pVolume;        // Output volume
                plug::IPort            *pMesh;          // Output mesh response
                plug::IPort            *pRelative;      // Relative mesh display
                plug::IPort            *pReference;     // Enable reference sine generator
                plug::IPort            *pHClipOn;       // Enable Hard clip
                plug::IPort            *pHClipRange;    // Hard clipping range
                plug::IPort            *pHClipReset;    // Hard clipping reset

            protected:
                void                update_response_curve();
                void                process_spectrum(channel_t *c, float *buf);
                void                do_destroy();

            protected:
                static void         process_callback(void *object, void *subject, float *buf, size_t rank);

            public:
                explicit loud_comp(const meta::plugin_t *metadata, size_t channels);
                virtual ~loud_comp() override;

                virtual void        init(plug::IWrapper *wrapper, plug::IPort **ports) override;
                virtual void        destroy() override;

            public:
                virtual void        ui_activated() override;
                virtual void        update_sample_rate(long sr) override;
                virtual void        update_settings() override;
                virtual void        process(size_t samples) override;
                virtual bool        inline_display(plug::ICanvas *cv, size_t width, size_t height) override;
                virtual void        dump(dspu::IStateDumper *v) const override;
        };

    } /* namespace plugins */
} /* namespace lsp */

#endif /* PRIVATE_PLUGINS_LOUD_COMP_H_ */
