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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

extern "C" {
    #include "platform.h"
    #include "build/debug.h"
    #include "pg/pg_ids.h"

    #include "common/axis.h"
    #include "common/maths.h"
    #include "common/time.h"
    #include "common/vector.h"

    #include "config/feature.h"

    #include "fc/runtime_config.h"

    #include "flight/autopilot.h"
    #include "flight/fence.h"
    #include "flight/position_estimator.h"
    #include "flight/position_nav.h"

    #include "io/gps.h"

    #include "pg/autopilot.h"
    #include "pg/gps_rescue.h"
    #include "pg/pos_hold.h"

    #include "rx/rx.h"

    PG_REGISTER(autopilotConfig_t, autopilotConfig, PG_AUTOPILOT, 0);
    PG_REGISTER(posHoldConfig_t, posHoldConfig, PG_POSHOLD_CONFIG, 0);
    PG_REGISTER(gpsRescueConfig_t, gpsRescueConfig, PG_GPS_RESCUE, 0);
    PG_REGISTER(rxConfig_t, rxConfig, PG_RX_CONFIG, 0);

    float fenceAllowedClimbRateCmS(float altitudeCm, float ceilingCm);
    float fenceStopDistanceCm(float distanceCm, float radialSpeedCmS, float decelCmSS);

    uint8_t armingFlags = 0;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;
    uint32_t GPS_distanceToHomeCm;

    // altitude
    static bool mockAltitudeValid;
    static float mockAltitudeCm;
    static float mockClimbRateCmS;
    static float mockCosTilt;
    static bool mock3d;
    bool isAltitudeAvailable(void) { return mockAltitudeValid; }
    float getAltitudeCmControl(void) { return mockAltitudeCm; }
    float getAltitudeDerivativeControl(void) { return mockClimbRateCmS; }
    float getCosTiltAngle(void) { return mockCosTilt; }
    bool featureIsEnabled(const uint32_t mask) { return mask == FEATURE_3D && mock3d; }
    uint16_t autopilotGetEffectiveHoverThrottlePwm(void) { return 1300; }
    float pidGetDT(void) { return 1.0f / 8000.0f; }     // 8 kHz loop

    // position: home sits mockHomeEfCm from the estimator origin
    static positionEstimate3d_t mockEstimate;
    static bool mockOriginValid;
    static vector2_t mockHomeEfCm;
    const positionEstimate3d_t *positionEstimatorGetEstimate(void) { return &mockEstimate; }
    bool positionEstimatorGetGpsOrigin(gpsLocation_t *out) { memset(out, 0, sizeof(*out)); return mockOriginValid; }
    void GPS_distance2d(const gpsLocation_t *from, const gpsLocation_t *to, vector2_t *distance)
    {
        UNUSED(from);
        UNUSED(to);
        *distance = mockHomeEfCm;
    }

    // navigator
    static positionNavCommand_t mockNavCommand;
    static int mockNavTargetsSet;
    void positionNavSetTargetEf(const vector3_t *targetPosEfM, float cruiseSpeedMps, float acceptanceRadiusM,
                                float completionSpeedMps, bool includeAltitude, positionNavReachedCallbackFn callback, void *userData)
    {
        UNUSED(callback);
        UNUSED(userData);
        mockNavCommand.active = true;
        mockNavCommand.completed = false;
        mockNavCommand.sequence++;
        mockNavCommand.targetPosEfM = *targetPosEfM;
        mockNavCommand.cruiseSpeedMps = cruiseSpeedMps;
        mockNavCommand.acceptanceRadiusM = acceptanceRadiusM;
        mockNavCommand.completionSpeedMps = completionSpeedMps;
        mockNavCommand.includeAltitude = includeAltitude;
        mockNavTargetsSet++;
    }
    void positionNavStartAfresh(void) { }
    void positionNavSetSettleTimeout(float timeoutS) { UNUSED(timeoutS); }
    void positionNavClearTarget(void) { mockNavCommand.active = false; mockNavCommand.completed = false; }
    bool positionNavHasActiveTarget(void) { return mockNavCommand.active; }
    bool positionNavTargetReached(void) { return mockNavCommand.completed; }
    const positionNavCommand_t *positionNavGetActiveCommand(void) { return &mockNavCommand; }
    static float mockYawRateLimitDps;
    void autopilotSetYawRateLimit(float rateLimitDps) { mockYawRateLimitDps = rateLimitDps; }
    static bool mockHeadingOverrideValid;
    static float mockHeadingOverrideDeg;
    void autopilotSetNavHeadingOverride(bool valid, float headingDeg)
    {
        mockHeadingOverrideValid = valid;
        mockHeadingOverrideDeg = headingDeg;
    }

    // position hold, sticks, the rest of the aircraft
    static bool mockPosHoldReady;
    static bool mockAutopilotInControl;
    static bool mockFailsafe;
    static float mockStick[3];
    bool posHoldReady(void) { return mockPosHoldReady; }
    bool isAutopilotInControl(void) { return mockAutopilotInControl; }
    bool failsafeIsActive(void) { return mockFailsafe; }
    float getRcDeflectionAbs(int axis) { return fabsf(mockStick[axis]); }
    static bool mockThrottleNearHover;
    static bool mockRxValid;
    bool altHoldThrottleNearHover(void) { return mockThrottleNearHover; }
    bool rxAreFlightChannelsValid(void) { return mockRxValid; }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static timeUs_t now;

#define LOOPS_PER_SECOND 8000

// the throttle the mixer flies once the cap's filter has settled
static float settledCap(float throttle)
{
    float out = throttle;
    for (int i = 0; i < LOOPS_PER_SECOND / 2; i++) {
        out = fenceLimitThrottle(throttle);
    }
    return out;
}

// the handover run for seconds of mixer loops at a steady input
static float handover(float throttle, float seconds)
{
    float out = throttle;
    for (int i = 0; i < lrintf(seconds * LOOPS_PER_SECOND); i++) {
        out = fenceHandoverThrottle(throttle);
    }
    return out;
}

class FenceTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        fenceInit();
        armingFlags = 0;
        flightModeFlags = 0;
        stateFlags = 0;
        memset(&gpsSol, 0, sizeof(gpsSol));

        mockAltitudeValid = true;
        mockAltitudeCm = 0.0f;
        mockClimbRateCmS = 0.0f;
        mockCosTilt = 1.0f;
        mock3d = false;

        memset(&mockEstimate, 0, sizeof(mockEstimate));
        mockEstimate.isValidXY = true;
        mockOriginValid = true;
        mockHomeEfCm = {{ 0.0f, 0.0f }};

        memset(&mockNavCommand, 0, sizeof(mockNavCommand));
        mockNavTargetsSet = 0;
        mockHeadingOverrideValid = false;
        mockYawRateLimitDps = 0.0f;
        mockPosHoldReady = true;
        mockAutopilotInControl = true;
        mockFailsafe = false;
        mockStick[FD_ROLL] = mockStick[FD_PITCH] = mockStick[FD_YAW] = 0.0f;
        mockThrottleNearHover = true;
        mockRxValid = true;
        GPS_distanceToHomeCm = 0;

        fenceConfigMutable()->altitudeMode = FENCE_MODE_OFF;
        fenceConfigMutable()->maxAltitudeM = 120;
        fenceConfigMutable()->altitudeWarningM = 20;
        fenceConfigMutable()->distanceMode = FENCE_MODE_OFF;
        fenceConfigMutable()->maxDistanceM = 100;
        fenceConfigMutable()->distanceWarningM = 20;
        fenceConfigMutable()->brakeAngleDeg = 30;   // braking counted on for 0.7 g tan 30 = 3.96 m/s^2

        autopilotConfigMutable()->maxVelocity = 500;
        autopilotConfigMutable()->maxAngle = 50;
        posHoldConfigMutable()->deadband = 5;
        posHoldConfigMutable()->positionSource = POSHOLD_SOURCE_AUTO;
        gpsRescueConfigMutable()->minSats = 8;
        rxConfigMutable()->mincheck = 1050;

        now = 1000000;
    }

    void arm(void)
    {
        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_HOME);
        gpsSol.numSat = 12;
    }

    // east of the estimator origin, flying east (away from a home at the origin)
    void placeEast(float distanceM, float speedCmS)
    {
        mockEstimate.position.v[ENU_E] = distanceM * 100.0f;
        mockEstimate.position.v[ENU_N] = 0.0f;
        mockEstimate.velocity.v[ENU_E] = speedCmS;
        mockEstimate.velocity.v[ENU_N] = 0.0f;
    }

    void step(timeUs_t dtUs = 10000)
    {
        now += dtUs;
        fenceUpdate(now);
    }
};

// Ceiling envelope

TEST_F(FenceTest, AllowedClimbRateStopsAtTheCeiling)
{
    EXPECT_FLOAT_EQ(0.0f, fenceAllowedClimbRateCmS(12000.0f, 12000.0f));
    // close to it, the position gain: 1 m below allows 1 m/s
    EXPECT_NEAR(100.0f, fenceAllowedClimbRateCmS(11900.0f, 12000.0f), 0.01f);
    // far from it, braking at 5 m/s^2: 100 m below allows sqrt(2 * 5 * 100) = 31.6 m/s
    EXPECT_NEAR(3162.3f, fenceAllowedClimbRateCmS(2000.0f, 12000.0f), 1.0f);
}

TEST_F(FenceTest, AboveTheCeilingIsPushedBackDown)
{
    EXPECT_NEAR(-50.0f, fenceAllowedClimbRateCmS(12050.0f, 12000.0f), 0.01f);
    // ...no faster than the recovery rate
    EXPECT_NEAR(-200.0f, fenceAllowedClimbRateCmS(15000.0f, 12000.0f), 0.01f);
}

TEST_F(FenceTest, NoThrottleCapUnlessArmedAndLimiting)
{
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 12000.0f;
    mockClimbRateCmS = 1000.0f;
    step();
    EXPECT_FLOAT_EQ(0.9f, fenceLimitThrottle(0.9f));   // disarmed

    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_WARN;
    step();
    EXPECT_FLOAT_EQ(0.9f, fenceLimitThrottle(0.9f));   // warnings only

    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeValid = false;
    step();
    EXPECT_FLOAT_EQ(0.9f, fenceLimitThrottle(0.9f));   // nothing to measure it with

    mockAltitudeValid = true;
    mock3d = true;
    step();
    EXPECT_FLOAT_EQ(0.9f, fenceLimitThrottle(0.9f));   // reversible motors
}

TEST_F(FenceTest, FarBelowTheCeilingTheThrottleIsLeftAlone)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 3000.0f;
    mockClimbRateCmS = 500.0f;
    step();
    EXPECT_FLOAT_EQ(1.0f, settledCap(1.0f));
}

TEST_F(FenceTest, ClimbingIntoTheCeilingCapsTheThrottle)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 11500.0f;      // 5 m below, allowed 5 m/s
    mockClimbRateCmS = 1500.0f;     // climbing at 15 m/s
    step();
    // cut to half of hover, and no further
    EXPECT_NEAR(0.5f * (1300.0f - 1050.0f) / 950.0f, settledCap(1.0f), 0.001f);
    // the pilot can always throttle lower
    EXPECT_FLOAT_EQ(0.0f, fenceLimitThrottle(0.0f));
}

TEST_F(FenceTest, HoveringAtTheCeilingCapsNearHover)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 12000.0f;
    mockClimbRateCmS = 0.0f;
    step();
    // hover 1300 us over a 1050..2000 range
    EXPECT_NEAR((1300.0f - 1050.0f) / 950.0f, settledCap(1.0f), 0.001f);
}

TEST_F(FenceTest, TiltRaisesTheCapAndInvertedLiftsIt)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 12000.0f;
    step();
    const float levelCap = settledCap(1.0f);

    mockCosTilt = 0.5f;
    step();
    EXPECT_NEAR(2.0f * levelCap, settledCap(1.0f), 0.01f);

    // let go of at once upside down
    mockCosTilt = -0.5f;
    step();
    EXPECT_FLOAT_EQ(1.0f, fenceLimitThrottle(1.0f));
}

TEST_F(FenceTest, AltitudeTargetsStayUnderTheCeiling)
{
    float targetCm = 15000.0f;
    EXPECT_FALSE(fenceConstrainAltitudeTarget(&targetCm));
    EXPECT_FLOAT_EQ(15000.0f, targetCm);

    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    EXPECT_TRUE(fenceConstrainAltitudeTarget(&targetCm));
    EXPECT_FLOAT_EQ(11800.0f, targetCm);

    targetCm = 5000.0f;
    EXPECT_FALSE(fenceConstrainAltitudeTarget(&targetCm));
    EXPECT_FLOAT_EQ(5000.0f, targetCm);
}

// Arming

TEST_F(FenceTest, ArmingNeedsWhatAnEnforcedLimitMeasuresWith)
{
    mockAltitudeValid = false;
    EXPECT_TRUE(fenceIsReadyToArm());

    fenceConfigMutable()->altitudeMode = FENCE_MODE_WARN;
    EXPECT_TRUE(fenceIsReadyToArm());
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    EXPECT_FALSE(fenceIsReadyToArm());
    mockAltitudeValid = true;
    EXPECT_TRUE(fenceIsReadyToArm());

    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    EXPECT_FALSE(fenceIsReadyToArm());
    ENABLE_STATE(GPS_FIX);
    gpsSol.numSat = 7;
    EXPECT_FALSE(fenceIsReadyToArm());
    gpsSol.numSat = 8;
    EXPECT_TRUE(fenceIsReadyToArm());
}

// Radius

TEST_F(FenceTest, StopDistanceCountsOnlyOutwardSpeed)
{
    EXPECT_FLOAT_EQ(5000.0f, fenceStopDistanceCm(5000.0f, -1000.0f, 400.0f));
    // 10 m/s: 7.5 m flown before the brake bites, then 12.5 m braking at 4 m/s^2
    EXPECT_FLOAT_EQ(7000.0f, fenceStopDistanceCm(5000.0f, 1000.0f, 400.0f));
}

TEST_F(FenceTest, WarnModeNeverTakesTheAircraft)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_WARN;
    placeEast(105.0f, 1000.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_DISTANCE_MAX, fenceGetWarning());
    EXPECT_EQ(0, fenceGetDistanceLimitM());
}

TEST_F(FenceTest, BreachIsCaughtBeforeTheRadius)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    EXPECT_EQ(100, fenceGetDistanceLimitM());

    placeEast(85.0f, 0.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_DISTANCE_NEAR, fenceGetWarning());
    EXPECT_EQ(15, fenceGetDistanceMarginM());

    // 7 m/s out would carry it to 96.4 m: still fine
    placeEast(85.0f, 700.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    // 10 m/s out would carry it to 105.1 m
    placeEast(85.0f, 1000.0f);
    step();
    EXPECT_TRUE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
}

TEST_F(FenceTest, ReturnBrakesFliesBackHoldsAndHandsBack)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
    EXPECT_EQ(0, mockNavTargetsSet);         // braking first

    // stopped beyond the radius: flown back to the return point 20 m inside it, on the way home
    placeEast(104.0f, 0.0f);
    step();
    ASSERT_EQ(1, mockNavTargetsSet);
    EXPECT_NEAR(80.0f, mockNavCommand.targetPosEfM.v[ENU_E], 0.01f);
    EXPECT_NEAR(0.0f, mockNavCommand.targetPosEfM.v[ENU_N], 0.01f);
    EXPECT_FALSE(mockNavCommand.includeAltitude);   // altitude stays with the pilot through alt hold
    // nose towards home, west, for the whole leg
    EXPECT_TRUE(mockHeadingOverrideValid);
    EXPECT_NEAR(-90.0f, mockHeadingOverrideDeg, 0.5f);
    EXPECT_FLOAT_EQ(90.0f, mockYawRateLimitDps);    // turned onto unhurriedly
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());

    // the pilot's sticks do nothing until it is back
    mockStick[FD_PITCH] = 0.5f;
    placeEast(90.0f, -500.0f);
    step();
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());

    // arrived: position hold holds it there, the pilot's sticks still ignored
    mockNavCommand.completed = true;
    placeEast(80.0f, 0.0f);
    step();
    EXPECT_TRUE(fenceOwnsFlight());
    EXPECT_FALSE(mockNavCommand.active);    // the fence cleared its target...
    EXPECT_FALSE(mockHeadingOverrideValid); // ...and its heading
    EXPECT_FLOAT_EQ(0.0f, mockYawRateLimitDps);
    EXPECT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());

    // sticks deflected: held
    for (int i = 0; i < 100; i++) {
        step();
    }
    EXPECT_TRUE(fenceOwnsFlight());

    // centred for half a second: handed back
    mockStick[FD_PITCH] = 0.0f;
    for (int i = 0; i < 49; i++) {
        step();
    }
    EXPECT_TRUE(fenceOwnsFlight());
    step();
    step();
    EXPECT_FALSE(fenceOwnsFlight());
}

TEST_F(FenceTest, BrakingShortOfTheReturnPointHoldsWhereItStopped)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(70.0f, 2000.0f);              // 20 m/s out would carry it to 135 m
    step();
    ASSERT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());

    placeEast(75.0f, 0.0f);                 // braked harder than counted on
    step();
    EXPECT_EQ(0, mockNavTargetsSet);
    EXPECT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());
}

TEST_F(FenceTest, DriftingOutWhileHeldIsCaught)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    placeEast(80.0f, 0.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());

    // blown out by the wind
    placeEast(97.0f, 500.0f);
    step();
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
}

TEST_F(FenceTest, HomeAwayFromTheEstimatorOrigin)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockHomeEfCm = {{ 0.0f, 5000.0f }};     // home 50 m north of where estimation started
    mockEstimate.position.v[ENU_E] = 0.0f;
    mockEstimate.position.v[ENU_N] = 15500.0f;  // 105 m north of home
    step();
    ASSERT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
    step();
    ASSERT_EQ(1, mockNavTargetsSet);
    EXPECT_NEAR(0.0f, mockNavCommand.targetPosEfM.v[ENU_E], 0.01f);
    EXPECT_NEAR(130.0f, mockNavCommand.targetPosEfM.v[ENU_N], 0.01f);
}

TEST_F(FenceTest, FailsafeAndRescueTakePrecedence)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    flightModeFlags |= GPS_RESCUE_MODE;
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    flightModeFlags &= ~GPS_RESCUE_MODE;
    step();
    ASSERT_TRUE(fenceOwnsFlight());

    mockFailsafe = true;
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    // once the failsafe is over the fence picks it up again
    mockFailsafe = false;
    step();
    EXPECT_TRUE(fenceOwnsFlight());
}

TEST_F(FenceTest, AMissionFlightPlanIsNotTakenOver)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    flightModeFlags |= AUTOPILOT_MODE;
    step();
    EXPECT_FALSE(fenceOwnsFlight());
}

TEST_F(FenceTest, LosingPositionHoldHandsBackAndDoesNotSnatchAgain)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    mockAutopilotInControl = false;          // position hold has not taken it yet
    step();
    ASSERT_TRUE(fenceOwnsFlight());
    step();
    EXPECT_TRUE(fenceOwnsFlight());          // inside the grace period

    for (int i = 0; i < 60; i++) {
        step();
    }
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_DISTANCE_MAX, fenceGetWarning());

    // still out there: left alone
    mockAutopilotInControl = true;
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    // back inside, and out again: enforced again
    placeEast(70.0f, 0.0f);
    step();
    placeEast(105.0f, 0.0f);
    step();
    EXPECT_TRUE(fenceOwnsFlight());
}

TEST_F(FenceTest, NoGpsMeansNoRadiusAndAWarning)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    DISABLE_STATE(GPS_FIX);
    placeEast(105.0f, 0.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_UNAVAILABLE, fenceGetWarning());
}

// Without a compass, before the GPS course has taught the IMU a heading

TEST_F(FenceTest, WithoutAHeadingThePilotIsToldToFlyStraight)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockPosHoldReady = false;
    placeEast(50.0f, 0.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_NO_HEADING, fenceGetWarning());

    mockPosHoldReady = true;                // a few seconds flying straight ahead
    step();
    EXPECT_EQ(FENCE_WARNING_NONE, fenceGetWarning());
}

TEST_F(FenceTest, WithoutAHeadingTheFenceTakesItEarlier)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    // 5 m/s out from 85 m: braked, it stops at 91.9 m
    placeEast(85.0f, 500.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    // only levelled, drag takes 20 m more and it would stop at 108.8 m
    mockPosHoldReady = false;
    step();
    EXPECT_TRUE(fenceOwnsFlight());
}

TEST_F(FenceTest, WithoutAHeadingTheFenceLevelsItAndHandsItBackStopped)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockPosHoldReady = false;
    placeEast(90.0f, 400.0f);
    step();
    ASSERT_TRUE(fenceOwnsFlight());
    EXPECT_TRUE(fenceHoldsLevel());
    EXPECT_EQ(FENCE_WARNING_STOPPING, fenceGetWarning());
    EXPECT_EQ(0, mockNavTargetsSet);        // nothing to fly back with

    // still coasting: sticks centred do not hand it back yet
    placeEast(97.0f, 200.0f);
    for (int i = 0; i < 100; i++) {
        step();
    }
    EXPECT_TRUE(fenceHoldsLevel());

    // stopped: the pilot can have it back, to fly in themselves
    placeEast(98.0f, 50.0f);
    step();
    EXPECT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());
    for (int i = 0; i < 52; i++) {
        step();
    }
    EXPECT_FALSE(fenceOwnsFlight());

    // still drifting out a little: not levelled again, the pilot is flying it back
    for (int i = 0; i < 100; i++) {
        step();
    }
    EXPECT_FALSE(fenceOwnsFlight());

    // flown in nose first: the heading is learnt, and the full fence is back
    mockPosHoldReady = true;
    placeEast(99.0f, 300.0f);
    step();
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
}

TEST_F(FenceTest, LevelledAgainOnlyOnceBackInside)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockPosHoldReady = false;
    placeEast(98.0f, 400.0f);
    step();
    placeEast(99.0f, 0.0f);
    for (int i = 0; i < 53; i++) {
        step();
    }
    ASSERT_FALSE(fenceOwnsFlight());

    placeEast(99.0f, 300.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    placeEast(60.0f, 0.0f);                 // flown back in
    step();
    placeEast(90.0f, 400.0f);               // and out again
    step();
    EXPECT_TRUE(fenceHoldsLevel());
}

TEST_F(FenceTest, WithoutAHeadingADriftingAircraftIsHandedBackInTheEnd)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockPosHoldReady = false;
    placeEast(98.0f, 300.0f);
    step();
    ASSERT_TRUE(fenceHoldsLevel());

    placeEast(101.0f, 300.0f);              // carried on the wind, never stopping
    step(9900000);
    EXPECT_EQ(FENCE_WARNING_STOPPING, fenceGetWarning());
    step(200000);
    EXPECT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());
}

TEST_F(FenceTest, AHeadingLearntWhileLevelledBrakesAndFliesBack)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockPosHoldReady = false;
    placeEast(98.0f, 400.0f);
    step();
    ASSERT_TRUE(fenceHoldsLevel());

    mockPosHoldReady = true;
    step();
    EXPECT_FALSE(fenceHoldsLevel());
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
}

TEST_F(FenceTest, ReturnThatNeverArrivesHandsBack)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    step();
    step();
    ASSERT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
    step(59000000);
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
    step(2000000);
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_FALSE(mockNavCommand.active);
}

TEST_F(FenceTest, DisarmingEndsAReturn)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    step();
    step();
    ASSERT_TRUE(mockNavCommand.active);
    DISABLE_ARMING_FLAG(ARMED);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_FALSE(mockNavCommand.active);
    EXPECT_EQ(FENCE_WARNING_NONE, fenceGetWarning());
}

TEST_F(FenceTest, AltitudeWarnings)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_WARN;
    mockAltitudeCm = 9000.0f;
    step();
    EXPECT_EQ(FENCE_WARNING_NONE, fenceGetWarning());

    mockAltitudeCm = 10500.0f;
    step();
    EXPECT_EQ(FENCE_WARNING_ALTITUDE_NEAR, fenceGetWarning());
    EXPECT_EQ(15, fenceGetAltitudeMarginM());

    mockAltitudeCm = 12100.0f;
    step();
    EXPECT_EQ(FENCE_WARNING_ALTITUDE_MAX, fenceGetWarning());

    mockAltitudeValid = false;
    step();
    EXPECT_EQ(FENCE_WARNING_UNAVAILABLE, fenceGetWarning());
}

// Smoothing

TEST_F(FenceTest, TheCapEasesIn)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 12000.0f;
    step();
    const float hover = (1300.0f - 1050.0f) / 950.0f;

    // one loop in, most of the way still to go
    const float first = fenceLimitThrottle(1.0f);
    EXPECT_GT(first, 0.95f);
    // 16 ms (one time constant at 10 Hz) in, about two thirds of the way down
    float out = first;
    for (int i = 1; i < 127; i++) {
        out = fenceLimitThrottle(1.0f);
    }
    EXPECT_NEAR(1.0f - (1.0f - hover) * 0.63f, out, 0.02f);
    EXPECT_NEAR(hover, settledCap(1.0f), 0.001f);
}

TEST_F(FenceTest, BrakingLeansNoFurtherThanTheBrakeAngle)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    EXPECT_FLOAT_EQ(0.0f, fenceGetMaxAngleDeg());   // the pilot flies

    placeEast(105.0f, 0.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
    EXPECT_FLOAT_EQ(30.0f, fenceGetMaxAngleDeg());

    // never past what position hold itself may lean
    autopilotConfigMutable()->maxAngle = 20;
    EXPECT_FLOAT_EQ(20.0f, fenceGetMaxAngleDeg());
}

TEST_F(FenceTest, AGentlerBrakeAngleTakesTheAircraftEarlier)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    fenceConfigMutable()->brakeAngleDeg = 50;   // 0.7 g tan 50 = 8.18 m/s^2

    // 10 m/s out from 85 m stops at 98.6 m braking hard
    placeEast(85.0f, 1000.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    // but at 105.1 m braking at 30 degrees
    fenceConfigMutable()->brakeAngleDeg = 30;
    step();
    EXPECT_TRUE(fenceOwnsFlight());
}

TEST_F(FenceTest, ThrottleEasesOverToAltHoldWhenTheFenceTakesTheAircraft)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(50.0f, 0.0f);
    step();
    EXPECT_FLOAT_EQ(0.6f, handover(0.6f, 0.1f));    // the pilot's throttle, untouched

    placeEast(105.0f, 0.0f);
    step();
    ASSERT_TRUE(fenceOwnsFlight());
    // alt hold has not computed its throttle yet: hardly felt
    EXPECT_GT(handover(0.0f, 0.01f), 0.55f);
    // half way through, half way over
    EXPECT_NEAR(0.45f, handover(0.3f, 0.14f), 0.01f);
    EXPECT_FLOAT_EQ(0.3f, handover(0.3f, 0.2f));
}

TEST_F(FenceTest, ThrottleEasesBackToThePilotWhenHandedBack)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    placeEast(80.0f, 0.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());
    EXPECT_FLOAT_EQ(0.3f, handover(0.3f, 0.5f));    // alt hold's throttle

    for (int i = 0; i < 52; i++) {
        step();     // sticks centred half a second: handed back
    }
    ASSERT_FALSE(fenceOwnsFlight());
    EXPECT_NEAR(0.55f, handover(0.8f, 0.5f), 0.01f);
    EXPECT_FLOAT_EQ(0.8f, handover(0.8f, 0.6f));
}

TEST_F(FenceTest, NoHandoverWhileDisarmed)
{
    EXPECT_FLOAT_EQ(0.2f, fenceHandoverThrottle(0.2f));
    arm();
    EXPECT_FLOAT_EQ(0.7f, fenceHandoverThrottle(0.7f));
}

// Throttle stick, handing back, arming

TEST_F(FenceTest, ThrottleStickIgnoredUntilBroughtBackToHover)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockThrottleNearHover = false;          // punching out as the fence takes over
    placeEast(105.0f, 0.0f);
    step();
    ASSERT_TRUE(fenceOwnsFlight());
    EXPECT_TRUE(fenceHoldsThrottleStick());
    step();
    EXPECT_TRUE(fenceHoldsThrottleStick());

    mockThrottleNearHover = true;
    step();
    EXPECT_FALSE(fenceHoldsThrottleStick());

    // from then on alt hold reads it as a climb rate
    mockThrottleNearHover = false;
    step();
    EXPECT_FALSE(fenceHoldsThrottleStick());
}

TEST_F(FenceTest, HandedBackOnlyWithTheThrottleAtHover)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    placeEast(80.0f, 0.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());

    mockThrottleNearHover = false;          // throttle left down
    for (int i = 0; i < 100; i++) {
        step();
    }
    EXPECT_TRUE(fenceOwnsFlight());

    mockThrottleNearHover = true;
    for (int i = 0; i < 52; i++) {
        step();
    }
    EXPECT_FALSE(fenceOwnsFlight());
}

TEST_F(FenceTest, NotHandedBackOnReceiverFailsafeValues)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    placeEast(80.0f, 0.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());

    mockRxValid = false;                    // link lost, sticks at their centred rxfail values
    for (int i = 0; i < 100; i++) {
        step();
    }
    EXPECT_TRUE(fenceOwnsFlight());
}

TEST_F(FenceTest, ArmingRefusedOutsideTheRadiusOfAKeptHome)
{
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    ENABLE_STATE(GPS_FIX);
    gpsSol.numSat = 12;
    EXPECT_TRUE(fenceIsReadyToArm());

    // gps_set_home_point_once keeps an earlier home while disarmed
    ENABLE_STATE(GPS_FIX_HOME);
    GPS_distanceToHomeCm = 7000;
    EXPECT_TRUE(fenceIsReadyToArm());
    GPS_distanceToHomeCm = 9000;            // past the 80 m return point
    EXPECT_FALSE(fenceIsReadyToArm());
    EXPECT_EQ(FENCE_WARNING_UNAVAILABLE, fenceGetWarning());
}

TEST_F(FenceTest, NothingEnforcedOnReversibleMotors)
{
    mock3d = true;
    EXPECT_TRUE(fenceIsReadyToArm());
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    EXPECT_FALSE(fenceIsReadyToArm());
    fenceConfigMutable()->altitudeMode = FENCE_MODE_WARN;
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    ENABLE_STATE(GPS_FIX);
    gpsSol.numSat = 12;
    EXPECT_FALSE(fenceIsReadyToArm());

    // armed regardless (the setting changed in flight): not taken, and said so
    arm();
    placeEast(105.0f, 0.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
}

TEST_F(FenceTest, OpticalFlowAloneMeasuresNoDistance)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    posHoldConfigMutable()->positionSource = POSHOLD_SOURCE_OPTICALFLOW_ONLY;
    placeEast(105.0f, 0.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_UNAVAILABLE, fenceGetWarning());
}
