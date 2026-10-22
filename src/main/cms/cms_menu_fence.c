/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#ifdef USE_CMS
#ifdef USE_FENCE

#include "cms/cms.h"
#include "cms/cms_types.h"

#include "config/config.h"

#include "flight/fence.h"

#include "cms/cms_menu_fence.h"

static uint8_t fenceAltitudeMode;
static uint16_t fenceMaxAltitudeM;
static uint8_t fenceAltitudeWarningM;
static uint8_t fenceDistanceMode;
static uint16_t fenceMaxDistanceM;
static uint8_t fenceDistanceWarningM;
static uint8_t fenceBrakeAngleDeg;

static const void *cmsx_Fence_onEnter(displayPort_t *pDisp)
{
    UNUSED(pDisp);

    fenceAltitudeMode = fenceConfig()->altitudeMode;
    fenceMaxAltitudeM = fenceConfig()->maxAltitudeM;
    fenceAltitudeWarningM = fenceConfig()->altitudeWarningM;
    fenceDistanceMode = fenceConfig()->distanceMode;
    fenceMaxDistanceM = fenceConfig()->maxDistanceM;
    fenceDistanceWarningM = fenceConfig()->distanceWarningM;
    fenceBrakeAngleDeg = fenceConfig()->brakeAngleDeg;

    return NULL;
}

static const void *cmsx_Fence_onExit(displayPort_t *pDisp, const OSD_Entry *self)
{
    UNUSED(pDisp);
    UNUSED(self);

    fenceConfigMutable()->altitudeMode = fenceAltitudeMode;
    fenceConfigMutable()->maxAltitudeM = fenceMaxAltitudeM;
    fenceConfigMutable()->altitudeWarningM = fenceAltitudeWarningM;
    fenceConfigMutable()->distanceMode = fenceDistanceMode;
    fenceConfigMutable()->maxDistanceM = fenceMaxDistanceM;
    fenceConfigMutable()->distanceWarningM = fenceDistanceWarningM;
    fenceConfigMutable()->brakeAngleDeg = fenceBrakeAngleDeg;

    return NULL;
}

static const OSD_Entry cmsx_menuFenceEntries[] =
{
    { "-- FENCE --", OME_Label, NULL, NULL },

    { "ALT MODE",    OME_TAB,    NULL, &(OSD_TAB_t)    { &fenceAltitudeMode, FENCE_MODE_COUNT - 1, fenceModeNames } },
    { "ALT MAX M",   OME_UINT16, NULL, &(OSD_UINT16_t) { &fenceMaxAltitudeM, FENCE_ALTITUDE_MIN_M, FENCE_ALTITUDE_MAX_M, 5 } },
    { "ALT WARN M",  OME_UINT8,  NULL, &(OSD_UINT8_t)  { &fenceAltitudeWarningM, 0, FENCE_WARNING_MARGIN_MAX_M, 1 } },
    { "DIST MODE",   OME_TAB,    NULL, &(OSD_TAB_t)    { &fenceDistanceMode, FENCE_MODE_COUNT - 1, fenceModeNames } },
    { "DIST MAX M",  OME_UINT16, NULL, &(OSD_UINT16_t) { &fenceMaxDistanceM, FENCE_DISTANCE_MIN_M, FENCE_DISTANCE_MAX_M, 10 } },
    { "DIST WARN M", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &fenceDistanceWarningM, 0, FENCE_WARNING_MARGIN_MAX_M, 1 } },
    { "BRAKE ANGLE", OME_UINT8,  NULL, &(OSD_UINT8_t)  { &fenceBrakeAngleDeg, FENCE_BRAKE_ANGLE_MIN_DEG, FENCE_BRAKE_ANGLE_MAX_DEG, 5 } },

    { "BACK", OME_Back, NULL, NULL },
    { NULL, OME_END, NULL, NULL}
};

CMS_Menu cmsx_menuFence = {
#ifdef CMS_MENU_DEBUG
    .GUARD_text = "MENUFENCE",
    .GUARD_type = OME_MENU,
#endif
    .onEnter = cmsx_Fence_onEnter,
    .onExit = cmsx_Fence_onExit,
    .onDisplayUpdate = NULL,
    .entries = cmsx_menuFenceEntries
};

#endif // USE_FENCE
#endif // USE_CMS
