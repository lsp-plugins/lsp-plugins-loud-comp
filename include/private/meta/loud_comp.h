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

#ifndef PRIVATE_META_LOUD_COMP_H_
#define PRIVATE_META_LOUD_COMP_H_

#include <lsp-plug.in/plug-fw/meta/types.h>
#include <lsp-plug.in/plug-fw/const.h>

namespace lsp
{
    namespace meta
    {
        struct loud_comp_metadata
        {
            static constexpr float PHONS_MIN            = -83;
            static constexpr float PHONS_MAX            = 7;
            static constexpr float PHONS_DFL            = 0;
            static constexpr float PHONS_STEP           = 0.1f;

            static constexpr float HCRANGE_MIN          = 0;
            static constexpr float HCRANGE_MAX          = 24;
            static constexpr float HCRANGE_DFL          = 6;
            static constexpr float HCRANGE_STEP         = 0.05f;

            static constexpr float FREQ_MIN             = SPEC_FREQ_MIN;
            static constexpr float FREQ_MAX             = SPEC_FREQ_MAX;

            static constexpr size_t FFT_RANK_MIN        = 8;
            static constexpr size_t FFT_RANK_MAX        = 14;
            static constexpr size_t FFT_RANK_IDX_DFL    = 4;

            static constexpr size_t STD_DFL             = 4;

            static constexpr size_t CURVE_MESH_SIZE     = 512;
        };

        extern const meta::plugin_t loud_comp_mono;
        extern const meta::plugin_t loud_comp_stereo;
    } // namespace meta
} // namespace lsp


#endif /* PRIVATE_META_LOUD_COMP_H_ */
