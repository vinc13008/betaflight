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
    float fenceStopDistanceCm(float distanceCm, float radialSpeedCmS);

    uint8_t armingFlags = 0;
    uint16_t flightModeFlags = 0;
    uint8_t stateFlags = 0;
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;

    gpsSolutionData_t gpsSol;
    gpsLocation_t GPS_home_llh;

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
    void autopilotSetNavHeadingOverride(bool valid, float headingDeg) { UNUSED(valid); UNUSED(headingDeg); }

    // position hold, sticks, the rest of the aircraft
    static bool mockPosHoldReady;
    static bool mockAutopilotInControl;
    static bool mockFailsafe;
    static float mockStick[3];
    bool posHoldReady(void) { return mockPosHoldReady; }
    bool isAutopilotInControl(void) { return mockAutopilotInControl; }
    bool failsafeIsActive(void) { return mockFailsafe; }
    float getRcDeflectionAbs(int axis) { return fabsf(mockStick[axis]); }
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

static timeUs_t now;

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
        mockPosHoldReady = true;
        mockAutopilotInControl = true;
        mockFailsafe = false;
        mockStick[FD_ROLL] = mockStick[FD_PITCH] = mockStick[FD_YAW] = 0.0f;

        fenceConfigMutable()->altitudeMode = FENCE_MODE_OFF;
        fenceConfigMutable()->maxAltitudeM = 120;
        fenceConfigMutable()->altitudeWarningM = 20;
        fenceConfigMutable()->distanceMode = FENCE_MODE_OFF;
        fenceConfigMutable()->maxDistanceM = 100;
        fenceConfigMutable()->distanceWarningM = 20;

        autopilotConfigMutable()->maxVelocity = 500;
        posHoldConfigMutable()->deadband = 5;
        gpsRescueConfigMutable()->minSats = 8;
        rxConfigMutable()->mincheck = 1050;

        now = 1000000;
    }

    void arm(void)
    {
        ENABLE_ARMING_FLAG(ARMED);
        ENABLE_STATE(GPS_FIX);
        ENABLE_STATE(GPS_FIX_HOME);
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
    EXPECT_FLOAT_EQ(1.0f, fenceLimitThrottle(1.0f));
}

TEST_F(FenceTest, ClimbingIntoTheCeilingCapsTheThrottle)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 11500.0f;      // 5 m below, allowed 5 m/s
    mockClimbRateCmS = 1500.0f;     // climbing at 15 m/s
    step();
    const float capped = fenceLimitThrottle(1.0f);
    EXPECT_LT(capped, 0.1f);
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
    EXPECT_NEAR((1300.0f - 1050.0f) / 950.0f, fenceLimitThrottle(1.0f), 0.001f);
}

TEST_F(FenceTest, TiltRaisesTheCapAndInvertedLiftsIt)
{
    arm();
    fenceConfigMutable()->altitudeMode = FENCE_MODE_LIMIT;
    mockAltitudeCm = 12000.0f;
    step();
    const float levelCap = fenceLimitThrottle(1.0f);

    mockCosTilt = 0.5f;
    step();
    EXPECT_NEAR(2.0f * levelCap, fenceLimitThrottle(1.0f), 0.01f);

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
    EXPECT_FLOAT_EQ(5000.0f, fenceStopDistanceCm(5000.0f, -1000.0f));
    // 10 m/s braking at 4 m/s^2 takes 12.5 m
    EXPECT_FLOAT_EQ(6250.0f, fenceStopDistanceCm(5000.0f, 1000.0f));
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

    // 10 m/s out would carry it to 97.5 m: still fine
    placeEast(85.0f, 1000.0f);
    step();
    EXPECT_FALSE(fenceOwnsFlight());

    // 15 m/s out would carry it to 113 m
    placeEast(85.0f, 1500.0f);
    step();
    EXPECT_TRUE(fenceOwnsFlight());
    EXPECT_TRUE(fenceOwnsPosition());
    EXPECT_EQ(FENCE_WARNING_RETURNING, fenceGetWarning());
}

TEST_F(FenceTest, ReturnBrakesFliesBackHoldsAndHandsBack)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    ASSERT_TRUE(fenceOwnsPosition());
    EXPECT_EQ(0, mockNavTargetsSet);         // braking first

    // stopped beyond the radius: flown back to the return point 20 m inside it, on the way home
    placeEast(104.0f, 0.0f);
    step();
    ASSERT_EQ(1, mockNavTargetsSet);
    EXPECT_NEAR(80.0f, mockNavCommand.targetPosEfM.v[ENU_E], 0.01f);
    EXPECT_NEAR(0.0f, mockNavCommand.targetPosEfM.v[ENU_N], 0.01f);
    EXPECT_FALSE(mockNavCommand.includeAltitude);   // altitude stays with the pilot through alt hold
    EXPECT_TRUE(fenceOwnsPosition());

    // the pilot's sticks do nothing until it is back
    mockStick[FD_PITCH] = 0.5f;
    placeEast(90.0f, -500.0f);
    step();
    EXPECT_TRUE(fenceOwnsPosition());

    // arrived: position hold holds it, the pilot may fly it there
    mockNavCommand.completed = true;
    placeEast(80.0f, 0.0f);
    step();
    EXPECT_TRUE(fenceOwnsFlight());
    EXPECT_FALSE(fenceOwnsPosition());
    EXPECT_FALSE(mockNavCommand.active);    // the fence cleared its target
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
    placeEast(70.0f, 2000.0f);              // 20 m/s out would carry it to 120 m
    step();
    ASSERT_TRUE(fenceOwnsPosition());

    placeEast(75.0f, 0.0f);                 // braked harder than counted on
    step();
    EXPECT_EQ(0, mockNavTargetsSet);
    EXPECT_FALSE(fenceOwnsPosition());
    EXPECT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());
}

TEST_F(FenceTest, HoldingAndFlyingOutAgainIsCaught)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(98.0f, 800.0f);
    step();
    placeEast(80.0f, 0.0f);
    step();
    ASSERT_EQ(FENCE_WARNING_HOLDING, fenceGetWarning());

    mockStick[FD_PITCH] = 1.0f;
    placeEast(97.0f, 500.0f);
    step();
    EXPECT_TRUE(fenceOwnsPosition());
}

TEST_F(FenceTest, HomeAwayFromTheEstimatorOrigin)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    mockHomeEfCm = {{ 0.0f, 5000.0f }};     // home 50 m north of where estimation started
    mockEstimate.position.v[ENU_E] = 0.0f;
    mockEstimate.position.v[ENU_N] = 15500.0f;  // 105 m north of home
    step();
    ASSERT_TRUE(fenceOwnsPosition());
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

TEST_F(FenceTest, NoPositionMeansNoReturnAndAWarning)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    mockPosHoldReady = false;               // e.g. no heading yet
    step();
    EXPECT_FALSE(fenceOwnsFlight());
    EXPECT_EQ(FENCE_WARNING_DISTANCE_MAX, fenceGetWarning());

    placeEast(50.0f, 0.0f);
    step();
    EXPECT_EQ(FENCE_WARNING_UNAVAILABLE, fenceGetWarning());

    DISABLE_STATE(GPS_FIX);
    mockPosHoldReady = true;
    step();
    EXPECT_EQ(FENCE_WARNING_UNAVAILABLE, fenceGetWarning());
}

TEST_F(FenceTest, ReturnThatNeverArrivesHandsBack)
{
    arm();
    fenceConfigMutable()->distanceMode = FENCE_MODE_LIMIT;
    placeEast(105.0f, 0.0f);
    step();
    step();
    ASSERT_TRUE(fenceOwnsPosition());
    step(59000000);
    EXPECT_TRUE(fenceOwnsPosition());
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
