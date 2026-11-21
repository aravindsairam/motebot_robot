#include "vacuum_safety/gate_logic.hpp"

#include <algorithm>
#include <cmath>

namespace vacuum_safety
{

const char * toString(Source source)
{
  switch (source) {
    case Source::Manual: return "manual";
    case Source::Nav: return "nav2";
    default: return "none";
  }
}

GateLogic::GateLogic(const Config & config)
: config_(config) {}

Output GateLogic::update(const Inputs & in)
{
  Output out;
  const auto fresh = [&in](double stamp, double timeout) {
      return stamp >= 0.0 && in.now - stamp <= timeout;
    };

  // --- which inputs are missing --------------------------------------------------------------
  const bool mode_ok = fresh(in.mode_time, config_.mode_timeout);
  bool cliff_ok = true;
  for (double stamp : in.cliff_time) {
    cliff_ok = cliff_ok && fresh(stamp, config_.sensor_timeout);
  }
  const bool bumper_ok = fresh(in.bumper_time, config_.sensor_timeout);
  if (!mode_ok) {out.stale.emplace_back("mode");}
  if (!cliff_ok) {out.stale.emplace_back("cliff");}
  if (!bumper_ok) {out.stale.emplace_back("bumper");}

  // --- hazards -------------------------------------------------------------------------------
  // No reading (inf or NaN) is a cliff too: an absent floor returns nothing.
  const auto is_cliff = [&](std::size_t i) {
      const double r = in.cliff_range[i];
      return !std::isfinite(r) || r > config_.cliff_threshold;
    };
  const bool front_cliff = cliff_ok && (is_cliff(FRONT_LEFT) || is_cliff(FRONT_RIGHT));
  const bool left_cliff = cliff_ok && is_cliff(SIDE_LEFT);
  const bool right_cliff = cliff_ok && is_cliff(SIDE_RIGHT);
  out.off_floor = cliff_ok && is_cliff(FRONT_LEFT) && is_cliff(FRONT_RIGHT) &&
    is_cliff(SIDE_LEFT) && is_cliff(SIDE_RIGHT);
  out.cliff = front_cliff || left_cliff || right_cliff;
  out.bumper = bumper_ok && in.bumper != 0;
  const bool front_hazard = front_cliff || out.bumper;

  // --- 1. who may drive ----------------------------------------------------------------------
  std::string reason;
  if (!mode_ok) {
    reason = "no mode from the mode manager";
  } else if (in.transitioning) {
    reason = "mode change in progress";
  } else if (in.mode == "mapping" || in.mode == "manual") {
    out.source = Source::Manual;
  } else if (
    in.mode == "navigation" || in.mode == "docking" || in.mode == "cleaning" ||
    in.mode == "exploring")
  {
    // Docking is driven by Nav2 and then the docking server, on the same input; cleaning by the
    // cleaning node, which sends its rows and trips through Nav2; exploring by the explorer,
    // which sends its goals to Nav2.
    out.source = Source::Nav;
  } else if (in.mode == "idle" || in.mode == "failsafe" || in.mode == "docked") {
    reason = in.mode;
  } else {
    reason = "unknown mode '" + in.mode + "'";
  }

  // --- 2. the allowed source's command, unless it has gone quiet -------------------------------
  Velocity cmd;
  if (out.source != Source::None) {
    const Velocity & asked = out.source == Source::Manual ? in.manual : in.nav;
    const double stamp = out.source == Source::Manual ? in.manual_time : in.nav_time;
    if (!std::isfinite(asked.linear) || !std::isfinite(asked.angular)) {
      reason = "invalid command";  // NaN or infinity must never reach the wheels
    } else if (fresh(stamp, config_.command_timeout)) {
      cmd = asked;
    } else if (stamp >= 0.0 && !asked.isZero()) {
      reason = "command timeout";
    }
  }

  // Track the front hazard on every cycle, also while nobody is driving, so that a hazard
  // already present when driving becomes possible is not mistaken for a new one.
  const bool hazard_appeared = front_hazard && !front_hazard_before_;
  front_hazard_before_ = front_hazard;
  if (front_hazard) {
    hazard_clear_since_ = kNever;
  } else if (hazard_clear_since_ < 0.0) {
    hazard_clear_since_ = in.now;
  }

  // --- 3. off the floor: nothing moves ---------------------------------------------------------
  if (out.off_floor) {
    reflex_active_ = false;
    out.blocked_reason = "robot is off the floor";
    return out;  // command stays zero
  }

  // --- 4. reflex -------------------------------------------------------------------------------
  // Only a robot that is being driven backs away. One standing still that is touched, or in a
  // mode where nobody may drive, stays where it is.
  const bool moving = last_motion_ >= 0.0 && in.now - last_motion_ <= config_.motion_window;
  if (out.source == Source::None) {
    reflex_active_ = false;
  } else if (hazard_appeared && moving && !reflex_active_) {
    reflex_active_ = true;
    reflex_start_ = in.now;
  }
  if (reflex_active_) {
    const bool clear_long_enough =
      !front_hazard && in.now - hazard_clear_since_ >= config_.backoff_after_clear;
    const bool too_long = in.now - reflex_start_ >= config_.backoff_max;
    if (clear_long_enough || too_long) {
      reflex_active_ = false;
      forward_pause_until_ = in.now + config_.forward_pause;
    }
  }

  if (reflex_active_) {
    cmd.linear = -config_.backoff_speed;
    cmd.angular = 0.0;
    reason = front_cliff ? "cliff ahead: backing off" : "bumper pressed: backing off";
  } else if (out.source != Source::None) {
    // --- 5. refuse motion towards a hazard, or without the sensors that would see one ----------
    if (cmd.linear > 0.0) {
      const char * why = nullptr;
      if (front_cliff) {
        why = "cliff ahead";
      } else if (left_cliff || right_cliff) {
        why = "cliff at the side";
      } else if (out.bumper) {
        why = "bumper pressed";
      } else if (!cliff_ok) {
        why = "cliff sensor data stopped";
      } else if (!bumper_ok) {
        why = "bumper data stopped";
      } else if (in.now < forward_pause_until_) {
        why = "pausing after a reflex";
      }
      if (why != nullptr) {
        cmd.linear = 0.0;
        reason = why;
      }
    }
    if (left_cliff && cmd.angular > 0.0) {
      cmd.angular = 0.0;
      reason = "cliff at the left";
    }
    if (right_cliff && cmd.angular < 0.0) {
      cmd.angular = 0.0;
      reason = "cliff at the right";
    }
  }

  // --- 6. speed limits -------------------------------------------------------------------------
  cmd.linear = std::clamp(cmd.linear, -config_.max_linear, config_.max_linear);
  cmd.angular = std::clamp(cmd.angular, -config_.max_angular, config_.max_angular);

  if (!cmd.isZero()) {
    last_motion_ = in.now;
  }
  out.command = cmd;
  out.blocked_reason = reason;
  out.reflex_active = reflex_active_;
  return out;
}

}  // namespace vacuum_safety
