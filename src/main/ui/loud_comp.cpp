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

#include <private/plugins/loud_comp.h>
#include <lsp-plug.in/plug-fw/ui.h>

namespace lsp
{
    namespace plugui
    {
        //---------------------------------------------------------------------
        // Plugin UI factory
        static const meta::plugin_t *plugin_uis[] =
        {
            &meta::loud_comp_mono,
            &meta::loud_comp_stereo
        };

        static ui::Factory factory(plugin_uis, 2);

    } // namespace plugui
} // namespace lsp


