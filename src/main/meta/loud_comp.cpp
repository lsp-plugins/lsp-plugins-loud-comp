/*
 * Copyright (C) 2021 Linux Studio Plugins Project <https://lsp-plug.in/>
 *           (C) 2021 Vladimir Sadovnikov <sadko4u@gmail.com>
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

#include <lsp-plug.in/plug-fw/meta/ports.h>
#include <lsp-plug.in/shared/meta/developers.h>
#include <private/meta/loud_comp.h>

#define LSP_PLUGINS_LOUD_COMP_VERSION_MAJOR       1
#define LSP_PLUGINS_LOUD_COMP_VERSION_MINOR       0
#define LSP_PLUGINS_LOUD_COMP_VERSION_MICRO       10

#define LSP_PLUGINS_LOUD_COMP_VERSION  \
    LSP_MODULE_VERSION( \
        LSP_PLUGINS_LOUD_COMP_VERSION_MAJOR, \
        LSP_PLUGINS_LOUD_COMP_VERSION_MINOR, \
        LSP_PLUGINS_LOUD_COMP_VERSION_MICRO  \
    )

namespace lsp
{
    namespace meta
    {
        static const int plugin_classes[]           = { C_AMPLIFIER, -1 };
        static const int clap_features_mono[]       = { CF_AUDIO_EFFECT, CF_UTILITY, CF_MONO, -1 };
        static const int clap_features_stereo[]     = { CF_AUDIO_EFFECT, CF_UTILITY, CF_STEREO, -1 };

        static const port_item_t loud_comp_fft_rank[] =
        {
            { "256",    NULL },
            { "512",    NULL },
            { "1024",   NULL },
            { "2048",   NULL },
            { "4096",   NULL },
            { "8192",   NULL },
            { "16384",  NULL },
            { NULL, NULL }
        };

        static const port_item_t loud_comp_std[] =
        {
            { "Flat",               "lcomp.curve.flat" },
            { "ISO226-2003",        "lcomp.curve.iso226" },
            { "Fletcher-Munson",    "lcomp.curve.fm" },
            { "Robinson-Dadson",    "lcomp.curve.rd" },
            { NULL, NULL }
        };

        #define LOUD_COMP_COMMON \
            AMP_GAIN("input", "Input gain", GAIN_AMP_0_DB, GAIN_AMP_P_72_DB), \
            COMBO("std", "Loudness contour standard", loud_comp_metadata::STD_DFL, loud_comp_std), \
            COMBO("fft", "FFT size", loud_comp_metadata::FFT_RANK_IDX_DFL, loud_comp_fft_rank), \
            CONTROL("volume", "Output volume", U_DB, loud_comp_metadata::PHONS), \
            SWITCH("refer", "Enable reference sine generator", 0.0f), \
            SWITCH("hclip", "Hard-clipping enable", 0.0f), \
            CONTROL("hcrange", "Hard-clipping range", U_DB, loud_comp_metadata::HCRANGE), \
            TRIGGER("hcclean", "Clean hard-clipping indicators"), \
            MESH("spec", "Level compensation frequency graph", 2, loud_comp_metadata::CURVE_MESH_SIZE), \
            SWITCH("relspec", "Equalization curve is shown relative to the volume", 0.0f)

        static const port_t loud_comp_mono_ports[] =
        {
            PORTS_MONO_PLUGIN,
            BYPASS,
            LOUD_COMP_COMMON,
            METER_GAIN("ilm", "Input level meter", GAIN_AMP_P_24_DB),
            BLINK("olc", "Output level clip"),
            METER_GAIN("olm", "Output level meter", GAIN_AMP_P_24_DB),
            PORTS_END
        };

        static const port_t loud_comp_stereo_ports[] =
        {
            PORTS_STEREO_PLUGIN,
            BYPASS,
            LOUD_COMP_COMMON,
            METER_GAIN("ilm_l", "Input level meter Left", GAIN_AMP_P_24_DB),
            METER_GAIN("ilm_r", "Input level meter Right", GAIN_AMP_P_24_DB),
            BLINK("olc_l", "Output level clip Left"),
            BLINK("olc_r", "Output level clip Right"),
            METER_GAIN("olm_l", "Output level meter Left", GAIN_AMP_P_24_DB),
            METER_GAIN("olm_r", "Output level meter Right", GAIN_AMP_P_24_DB),
            PORTS_END
        };

        const meta::bundle_t loud_comp_bundle =
        {
            "loud_comp",
            "Loudness Compensator",
            B_UTILITIES,
            "CuySiF1VSj8",
            "This plugin applies equal loudness contour corrections defined by ISO 226:2003\nstandard to the input signal depending on the output volume settings.\nAdditionally it can provide ear protection by applying hard-clipping to the\noutput signal if it exceeds the allowed configurable level."
        };

        // Loudness Compensator
        const meta::plugin_t  loud_comp_mono =
        {
            "Lautstärke Kompensator Mono",
            "Loudness Compensator Mono",
            "LK1M",
            &developers::v_sadovnikov,
            "loud_comp_mono",
            LSP_LV2_URI("loud_comp_mono"),
            LSP_LV2UI_URI("loud_comp_mono"),
            "eno9",
            LSP_LADSPA_LOUD_COMP_BASE + 0,
            LSP_LADSPA_URI("loud_comp_mono"),
            LSP_CLAP_URI("loud_comp_mono"),
            LSP_PLUGINS_LOUD_COMP_VERSION,
            plugin_classes,
            clap_features_mono,
            E_INLINE_DISPLAY | E_DUMP_STATE,
            loud_comp_mono_ports,
            "util/loud_comp.xml",
            NULL,
            mono_plugin_port_groups,
            &loud_comp_bundle
        };

        const meta::plugin_t  loud_comp_stereo =
        {
            "Lautstärke Kompensator Stereo",
            "Loudness Compensator Stereo",
            "LK1S",
            &developers::v_sadovnikov,
            "loud_comp_stereo",
            LSP_LV2_URI("loud_comp_stereo"),
            LSP_LV2UI_URI("loud_comp_stereo"),
            "wva0",
            LSP_LADSPA_LOUD_COMP_BASE + 1,
            LSP_LADSPA_URI("loud_comp_stereo"),
            LSP_CLAP_URI("loud_comp_stereo"),
            LSP_PLUGINS_LOUD_COMP_VERSION,
            plugin_classes,
            clap_features_stereo,
            E_INLINE_DISPLAY | E_DUMP_STATE,
            loud_comp_stereo_ports,
            "util/loud_comp.xml",
            NULL,
            stereo_plugin_port_groups,
            &loud_comp_bundle
        };
    } // namespace meta
} // namespace lsp
