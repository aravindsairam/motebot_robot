#include <gtest/gtest.h>

#include <algorithm>
#include <limits>
#include <string>

#include "vacuum_safety/gate_logic.hpp"

using vacuum_safety::Config;
using vacuum_safety::GateLogic;
using vacuum_safety::Inputs;
using vacuum_safety::Output;
using vacuum_safety::Source;
using vacuum_safety::Velocity;

namespace
{
const double kFloor = 0.015;  // normal cliff sensor reading over floor
const double kDrop = 0.165;   // reading over the stairwell
const double kInf = std::numeric_limits<double>::infinity();

// Everything healthy at time `now`: fresh mode, floor under all four sensors, bumper released.
Inputs healthy(double now, const std::string & mode = "manual")
{
  Inputs in;
  in.now = now;
  in.mode = mode;
  in.mode_time = now;
  in.cliff_range = {kFloor, kFloor, kFloor, kFloor};
  in.cliff_time = {now, now, now, now};
  in.bumper = 0;
  in.bumper_time = now;
  return in;
}

Inputs driving(double now, double v, double w = 0.0, const std::string & mode = "manual")
{
  Inputs in = healthy(now, mode);
  if (mode == "navigation") {
    in.nav = {v, w};
    in.nav_time = now;
  } else {
    in.manual = {v, w};
    in.manual_time = now;
  }
  return in;
}

bool isStale(const Output & out, const std::string & what)
{
  return std::find(out.stale.begin(), out.stale.end(), what) != out.stale.end();
}
}  // namespace

// ---------------------------------------------------------------- who may drive

TEST(WhoMayDrive, ManualCommandPassesInManualAndMapping)
{
  for (const char * mode : {"manual", "mapping"}) {
    GateLogic gate;
    Output out = gate.update(driving(1.0, 0.2, 0.5, mode));
    EXPECT_EQ(out.source, Source::Manual) << mode;
    EXPECT_DOUBLE_EQ(out.command.linear, 0.2) << mode;
    EXPECT_DOUBLE_EQ(out.command.angular, 0.5) << mode;
    EXPECT_TRUE(out.blocked_reason.empty()) << mode;
  }
}

TEST(WhoMayDrive, Nav2CommandPassesOnlyInNavigation)
{
  GateLogic gate;
  Output out = gate.update(driving(1.0, 0.25, -0.3, "navigation"));
  EXPECT_EQ(out.source, Source::Nav);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.25);
  EXPECT_DOUBLE_EQ(out.command.angular, -0.3);
}

TEST(WhoMayDrive, DockingIsDrivenOnNav2sInput)
{
  GateLogic gate;
  Inputs in = healthy(1.0, "docking");
  in.nav = {0.1, 0.2};
  in.nav_time = 1.0;
  in.manual = {0.3, 0.0};  // the manual input is ignored while docking
  in.manual_time = 1.0;
  Output out = gate.update(in);
  EXPECT_EQ(out.source, Source::Nav);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.1);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.2);
}

TEST(WhoMayDrive, ExploringIsDrivenOnNav2sInputOnly)
{
  GateLogic gate;
  Inputs in = healthy(1.0, "exploring");
  in.nav = {0.2, -0.1};
  in.nav_time = 1.0;
  in.manual = {0.3, 0.0};  // the joystick cannot drive while the robot explores by itself
  in.manual_time = 1.0;
  Output out = gate.update(in);
  EXPECT_EQ(out.source, Source::Nav);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.2);
  EXPECT_DOUBLE_EQ(out.command.angular, -0.1);
  EXPECT_TRUE(out.blocked_reason.empty());
}

TEST(WhoMayDrive, CleaningIsDrivenOnNav2sInputOnly)
{
  GateLogic gate;
  Inputs in = healthy(1.0, "cleaning");
  in.nav = {0.2, -0.1};
  in.nav_time = 1.0;
  in.manual = {0.3, 0.0};  // the keyboard or the app's joystick cannot drive during a clean
  in.manual_time = 1.0;
  Output out = gate.update(in);
  EXPECT_EQ(out.source, Source::Nav);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.2);
  EXPECT_DOUBLE_EQ(out.command.angular, -0.1);
  EXPECT_TRUE(out.blocked_reason.empty());
}

TEST(WhoMayDrive, ManualCommandIsIgnoredInNavigation)
{
  GateLogic gate;
  Inputs in = healthy(1.0, "navigation");
  in.manual = {0.3, 0.0};
  in.manual_time = 1.0;
  Output out = gate.update(in);
  EXPECT_EQ(out.source, Source::Nav);
  EXPECT_TRUE(out.command.isZero());
}

TEST(WhoMayDrive, Nav2CommandIsIgnoredInManual)
{
  GateLogic gate;
  Inputs in = healthy(1.0, "manual");
  in.nav = {0.3, 0.0};
  in.nav_time = 1.0;
  Output out = gate.update(in);
  EXPECT_EQ(out.source, Source::Manual);
  EXPECT_TRUE(out.command.isZero());
}

TEST(WhoMayDrive, NobodyInIdleDockedOrFailsafe)
{
  for (const char * mode : {"idle", "docked", "failsafe"}) {
    GateLogic gate;
    Inputs in = healthy(1.0, mode);
    in.manual = {0.3, 0.2};
    in.manual_time = 1.0;
    in.nav = {0.3, 0.2};
    in.nav_time = 1.0;
    Output out = gate.update(in);
    EXPECT_EQ(out.source, Source::None) << mode;
    EXPECT_TRUE(out.command.isZero()) << mode;
    EXPECT_EQ(out.blocked_reason, mode);
  }
}

TEST(WhoMayDrive, NobodyDuringAModeChange)
{
  GateLogic gate;
  Inputs in = driving(1.0, 0.3);
  in.transitioning = true;
  Output out = gate.update(in);
  EXPECT_TRUE(out.command.isZero());
  EXPECT_EQ(out.blocked_reason, "mode change in progress");
}

TEST(WhoMayDrive, NobodyWhenTheModeIsUnknown)
{
  GateLogic gate;
  // A mode this gate has not been taught: a newer mode manager must not make the robot drive.
  Output out = gate.update(driving(1.0, 0.3, 0.0, "charging"));
  EXPECT_TRUE(out.command.isZero());
  EXPECT_EQ(out.blocked_reason, "unknown mode 'charging'");
}

TEST(WhoMayDrive, NobodyBeforeTheFirstModeArrives)
{
  GateLogic gate;
  Inputs in = driving(1.0, 0.3);
  in.mode_time = vacuum_safety::kNever;
  Output out = gate.update(in);
  EXPECT_TRUE(out.command.isZero());
  EXPECT_EQ(out.blocked_reason, "no mode from the mode manager");
  EXPECT_TRUE(isStale(out, "mode"));
}

TEST(WhoMayDrive, NobodyWhenTheModeManagerGoesQuiet)
{
  GateLogic gate;
  Inputs in = driving(10.0, 0.3);
  in.mode_time = 10.0 - 1.4;
  EXPECT_DOUBLE_EQ(gate.update(in).command.linear, 0.3);
  in.mode_time = 10.0 - 1.6;
  Output out = gate.update(in);
  EXPECT_TRUE(out.command.isZero());
  EXPECT_TRUE(isStale(out, "mode"));
}

// ---------------------------------------------------------------- command timeout

TEST(CommandTimeout, StopsWhenTheSourceGoesQuiet)
{
  GateLogic gate;
  Inputs in = healthy(10.0);
  in.manual = {0.3, 0.0};
  in.manual_time = 10.0 - 0.4;
  EXPECT_DOUBLE_EQ(gate.update(in).command.linear, 0.3);
  in.manual_time = 10.0 - 0.6;
  Output out = gate.update(in);
  EXPECT_TRUE(out.command.isZero());
  EXPECT_EQ(out.blocked_reason, "command timeout");
}

TEST(CommandTimeout, NoComplaintWhenNothingWasEverCommanded)
{
  GateLogic gate;
  Output out = gate.update(healthy(5.0));
  EXPECT_TRUE(out.command.isZero());
  EXPECT_TRUE(out.blocked_reason.empty());
}

TEST(CommandTimeout, NoComplaintAfterAnExplicitStop)
{
  GateLogic gate;
  Inputs in = healthy(10.0);
  in.manual = {0.0, 0.0};
  in.manual_time = 10.0 - 5.0;
  EXPECT_TRUE(gate.update(in).blocked_reason.empty());
}

// ---------------------------------------------------------------- cliff reflex

TEST(CliffReflex, BacksAwayThenPausesThenAllowsForwardAgain)
{
  GateLogic gate;
  EXPECT_DOUBLE_EQ(gate.update(driving(1.00, 0.3)).command.linear, 0.3);

  // a front cliff sensor trips: the reflex starts at once
  Inputs in = driving(1.02, 0.3);
  in.cliff_range[vacuum_safety::FRONT_LEFT] = kDrop;
  Output out = gate.update(in);
  EXPECT_TRUE(out.reflex_active);
  EXPECT_TRUE(out.cliff);
  EXPECT_DOUBLE_EQ(out.command.linear, -0.10);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
  EXPECT_EQ(out.blocked_reason, "cliff ahead: backing off");

  // still over the edge: keep backing, whatever is asked
  in = driving(1.30, 0.3, 1.0);
  in.cliff_range[vacuum_safety::FRONT_LEFT] = kDrop;
  out = gate.update(in);
  EXPECT_DOUBLE_EQ(out.command.linear, -0.10);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);

  // floor again at t=1.5: keeps backing for another 0.4 s
  EXPECT_DOUBLE_EQ(gate.update(driving(1.50, 0.3)).command.linear, -0.10);
  EXPECT_DOUBLE_EQ(gate.update(driving(1.85, 0.3)).command.linear, -0.10);

  // then the reflex ends and forward is refused for a second
  out = gate.update(driving(1.95, 0.3));
  EXPECT_FALSE(out.reflex_active);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_EQ(out.blocked_reason, "pausing after a reflex");
  EXPECT_DOUBLE_EQ(gate.update(driving(2.80, 0.3)).command.linear, 0.0);

  // turning and reversing are allowed during the pause
  out = gate.update(driving(2.85, -0.1, 0.8));
  EXPECT_DOUBLE_EQ(out.command.linear, -0.1);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.8);

  // after the pause forward works again
  out = gate.update(driving(3.00, 0.3));
  EXPECT_DOUBLE_EQ(out.command.linear, 0.3);
  EXPECT_TRUE(out.blocked_reason.empty());
}

TEST(CliffReflex, GivesUpBackingAfterTheTimeLimitButStillRefusesForward)
{
  GateLogic gate;
  gate.update(driving(1.0, 0.3));
  for (double t = 1.02; t < 3.0; t += 0.02) {
    Inputs in = driving(t, 0.3);
    in.cliff_range[vacuum_safety::FRONT_RIGHT] = kInf;
    EXPECT_TRUE(gate.update(in).reflex_active) << t;
  }
  Inputs in = driving(3.05, 0.3);
  in.cliff_range[vacuum_safety::FRONT_RIGHT] = kInf;
  Output out = gate.update(in);
  EXPECT_FALSE(out.reflex_active);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_EQ(out.blocked_reason, "cliff ahead");

  // the driver can still reverse away
  in = driving(3.10, -0.2);
  in.cliff_range[vacuum_safety::FRONT_RIGHT] = kInf;
  EXPECT_DOUBLE_EQ(gate.update(in).command.linear, -0.2);
}

TEST(CliffReflex, NoReadingCountsAsACliff)
{
  for (double reading : {kInf, std::numeric_limits<double>::quiet_NaN()}) {
    GateLogic gate;
    gate.update(driving(1.0, 0.3));
    Inputs in = driving(1.02, 0.3);
    in.cliff_range[vacuum_safety::FRONT_LEFT] = reading;
    Output out = gate.update(in);
    EXPECT_TRUE(out.cliff);
    EXPECT_TRUE(out.reflex_active);
  }
}

TEST(CliffReflex, OverridesNav2Too)
{
  GateLogic gate;
  gate.update(driving(1.0, 0.25, 0.0, "navigation"));
  Inputs in = driving(1.02, 0.25, 0.4, "navigation");
  in.cliff_range[vacuum_safety::FRONT_LEFT] = kDrop;
  Output out = gate.update(in);
  EXPECT_EQ(out.source, Source::Nav);
  EXPECT_DOUBLE_EQ(out.command.linear, -0.10);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
}

TEST(CliffReflex, NoReflexWhileNobodyIsDriving)
{
  GateLogic gate;
  gate.update(healthy(1.0, "idle"));
  Inputs in = healthy(1.02, "idle");
  in.cliff_range[vacuum_safety::FRONT_LEFT] = kDrop;
  Output out = gate.update(in);
  EXPECT_FALSE(out.reflex_active);
  EXPECT_TRUE(out.command.isZero());
  EXPECT_TRUE(out.cliff);
}

TEST(BumperReflex, ARobotStandingStillDoesNotJumpWhenTouched)
{
  for (const char * mode : {"manual", "navigation"}) {
    GateLogic gate;
    gate.update(healthy(1.0, mode));  // allowed to drive, but nothing is commanded
    Inputs in = healthy(1.02, mode);
    in.bumper = 4;
    Output out = gate.update(in);
    EXPECT_FALSE(out.reflex_active) << mode;
    EXPECT_TRUE(out.command.isZero()) << mode;
    EXPECT_TRUE(out.bumper) << mode;

    // and driving into whatever is touching it is refused
    in = driving(1.04, 0.2, 0.0, mode);
    in.bumper = 4;
    out = gate.update(in);
    EXPECT_FALSE(out.reflex_active) << mode;
    EXPECT_DOUBLE_EQ(out.command.linear, 0.0) << mode;
    EXPECT_EQ(out.blocked_reason, "bumper pressed") << mode;
  }
}

TEST(BumperReflex, StillFiresJustAfterTheCommandStopped)
{
  GateLogic gate;
  gate.update(driving(1.0, 0.3));
  gate.update(healthy(1.1));  // command went to zero; the robot is still rolling to a stop
  Inputs in = healthy(1.2);
  in.bumper = 4;
  EXPECT_TRUE(gate.update(in).reflex_active);
}

TEST(CliffReflex, AHazardAlreadyThereWhenDrivingStartsBlocksButDoesNotLurch)
{
  GateLogic gate;
  Inputs in = healthy(1.0, "idle");
  in.cliff_range[vacuum_safety::FRONT_LEFT] = kDrop;
  gate.update(in);
  // the mode becomes manual with the robot already at the edge
  in = driving(1.02, 0.3);
  in.cliff_range[vacuum_safety::FRONT_LEFT] = kDrop;
  Output out = gate.update(in);
  EXPECT_FALSE(out.reflex_active);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_EQ(out.blocked_reason, "cliff ahead");
}

// ---------------------------------------------------------------- side cliffs

TEST(SideCliff, RefusesForwardAndTurningTowardsIt)
{
  GateLogic gate;
  gate.update(driving(1.0, 0.0));
  Inputs in = driving(1.02, 0.2, 0.5);  // forward and turning left
  in.cliff_range[vacuum_safety::SIDE_LEFT] = kDrop;
  Output out = gate.update(in);
  EXPECT_FALSE(out.reflex_active);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);

  in = driving(1.04, 0.0, -0.5);  // turning right, away from it
  in.cliff_range[vacuum_safety::SIDE_LEFT] = kDrop;
  EXPECT_DOUBLE_EQ(gate.update(in).command.angular, -0.5);

  in = driving(1.06, 0.0, -0.5);  // a cliff on the right refuses a right turn
  in.cliff_range[vacuum_safety::SIDE_RIGHT] = kDrop;
  out = gate.update(in);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.0);
  EXPECT_EQ(out.blocked_reason, "cliff at the right");
}

// ---------------------------------------------------------------- bumper reflex

TEST(BumperReflex, BacksOffAndReleases)
{
  GateLogic gate;
  gate.update(driving(1.0, 0.3));
  Inputs in = driving(1.02, 0.3);
  in.bumper = 4;  // front centre
  Output out = gate.update(in);
  EXPECT_TRUE(out.bumper);
  EXPECT_TRUE(out.reflex_active);
  EXPECT_DOUBLE_EQ(out.command.linear, -0.10);
  EXPECT_EQ(out.blocked_reason, "bumper pressed: backing off");

  // released at 1.1; reflex continues until 1.5, then ends
  EXPECT_TRUE(gate.update(driving(1.10, 0.3)).reflex_active);
  EXPECT_TRUE(gate.update(driving(1.45, 0.3)).reflex_active);
  EXPECT_FALSE(gate.update(driving(1.55, 0.3)).reflex_active);
}

TEST(BumperReflex, WhileStillPressedForwardIsRefusedButReverseAndTurnAreNot)
{
  Config config;
  config.backoff_max = 0.5;
  GateLogic gate(config);
  gate.update(driving(1.0, 0.3));
  Inputs in = driving(1.02, 0.3);
  in.bumper = 2;
  gate.update(in);
  in = driving(1.60, 0.3, 0.7);  // reflex has timed out, bumper still pressed
  in.bumper = 2;
  Output out = gate.update(in);
  EXPECT_FALSE(out.reflex_active);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.7);
  EXPECT_EQ(out.blocked_reason, "bumper pressed");
  in = driving(1.62, -0.15);
  in.bumper = 2;
  EXPECT_DOUBLE_EQ(gate.update(in).command.linear, -0.15);
}

// ---------------------------------------------------------------- off the floor

TEST(OffFloor, NothingMovesWhenEveryCliffSensorSeesNoFloor)
{
  GateLogic gate;
  gate.update(driving(1.0, 0.3));
  Inputs in = driving(1.02, -0.2, 1.0);  // even reversing and turning
  in.cliff_range = {kInf, kInf, kDrop, kDrop};
  Output out = gate.update(in);
  EXPECT_TRUE(out.off_floor);
  EXPECT_FALSE(out.reflex_active);
  EXPECT_TRUE(out.command.isZero());
  EXPECT_EQ(out.blocked_reason, "robot is off the floor");
}

// ---------------------------------------------------------------- missing sensor data

TEST(MissingSensors, StaleCliffDataRefusesForwardOnly)
{
  GateLogic gate;
  Inputs in = driving(10.0, 0.3, 0.4);
  in.cliff_time[vacuum_safety::SIDE_RIGHT] = 10.0 - 0.6;
  Output out = gate.update(in);
  EXPECT_TRUE(isStale(out, "cliff"));
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_DOUBLE_EQ(out.command.angular, 0.4);
  EXPECT_EQ(out.blocked_reason, "cliff sensor data stopped");
  EXPECT_FALSE(out.cliff);  // unknown is not reported as a cliff

  in = driving(10.02, -0.2);
  in.cliff_time[vacuum_safety::SIDE_RIGHT] = 10.0 - 0.6;
  EXPECT_DOUBLE_EQ(gate.update(in).command.linear, -0.2);
}

TEST(MissingSensors, CliffDataNeverReceivedRefusesForward)
{
  GateLogic gate;
  Inputs in = driving(1.0, 0.3);
  in.cliff_time = {vacuum_safety::kNever, vacuum_safety::kNever, vacuum_safety::kNever, vacuum_safety::kNever};
  Output out = gate.update(in);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_FALSE(out.off_floor);
}

TEST(MissingSensors, StaleBumperDataRefusesForwardOnly)
{
  GateLogic gate;
  Inputs in = driving(10.0, 0.3);
  in.bumper_time = 10.0 - 0.6;
  Output out = gate.update(in);
  EXPECT_TRUE(isStale(out, "bumper"));
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
  EXPECT_EQ(out.blocked_reason, "bumper data stopped");
}

// ---------------------------------------------------------------- speed limits

TEST(SpeedLimits, ClampsToTheRobotsLimits)
{
  GateLogic gate;
  Output out = gate.update(driving(1.0, 1.0, 5.0));
  EXPECT_DOUBLE_EQ(out.command.linear, 0.306);
  EXPECT_DOUBLE_EQ(out.command.angular, 1.9);
  out = gate.update(driving(1.02, -1.0, -5.0));
  EXPECT_DOUBLE_EQ(out.command.linear, -0.306);
  EXPECT_DOUBLE_EQ(out.command.angular, -1.9);
}

TEST(SpeedLimits, ACommandThatIsNotANumberBecomesAStop)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (const Velocity & bad : {Velocity{nan, 0.0}, Velocity{0.2, nan}, Velocity{kInf, 0.0}, Velocity{0.0, -kInf}}) {
    GateLogic gate;
    Inputs in = healthy(1.0);
    in.manual = bad;
    in.manual_time = 1.0;
    Output out = gate.update(in);
    EXPECT_TRUE(out.command.isZero());
    EXPECT_EQ(out.blocked_reason, "invalid command");
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
