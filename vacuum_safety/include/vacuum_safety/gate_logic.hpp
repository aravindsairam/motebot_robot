// The safety gate's decisions, with no ROS in them.
//
// Every cycle the node hands in everything it knows (the robot's mode, the latest command from
// each source, the cliff and bumper readings, and when each of those arrived) and gets back the
// one velocity command that may go to the wheels, plus the reason if it differs from what was
// asked for. Time is passed in, so tests can replay any sequence exactly.
//
// Rules, in order:
//   1. The mode decides who may drive: the manual source in mapping and manual; Nav2 (and the
//      docking server, which shares its input) in navigation, docking and cleaning; nobody in idle,
//      docked, failsafe, during a mode change, or when the mode is unknown.
//   2. A command older than the command timeout counts as "stop". So does one that is not a number.
//   3. If every cliff sensor sees no floor the robot is off the floor: nothing moves.
//   4. A cliff ahead or a bumper press while the robot is being driven starts a reflex: back away
//      until the hazard has been clear for a moment (or a time limit), then hold off forward motion
//      briefly. A robot that is standing still does not react by moving.
//   5. While a hazard is present, or cliff or bumper data is missing, forward motion is refused.
//      A cliff at one side also refuses turning towards it. Reversing is always allowed.
//   6. The result is clamped to the robot's speed limits.

#ifndef VACUUM_SAFETY__GATE_LOGIC_HPP_
#define VACUUM_SAFETY__GATE_LOGIC_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace vacuum_safety
{

struct Velocity
{
  double linear{0.0};   // m/s, forward positive
  double angular{0.0};  // rad/s, counter-clockwise (left) positive

  bool isZero() const {return linear == 0.0 && angular == 0.0;}
};

enum class Source {None, Manual, Nav};

// Order of the cliff sensors in Inputs.
enum CliffSensor : std::size_t {FRONT_LEFT = 0, FRONT_RIGHT = 1, SIDE_LEFT = 2, SIDE_RIGHT = 3};
constexpr std::size_t kCliffSensors = 4;

// A time that means "never received".
constexpr double kNever = -1.0;

struct Config
{
  double command_timeout{0.5};      // s without a command before the source counts as silent
  double mode_timeout{1.5};         // s without a robot state before the mode counts as unknown
  double sensor_timeout{0.5};       // s without cliff or bumper data before it counts as missing
  double cliff_threshold{0.05};     // m; a floor further away than this (or no reading) is a cliff
  double backoff_speed{0.10};       // m/s, reverse speed of the reflex
  double backoff_after_clear{0.4};  // s to keep backing after the hazard is no longer seen
  double backoff_max{2.0};          // s, upper limit on one reflex
  double forward_pause{1.0};        // s after a reflex during which forward motion is refused
  double motion_window{0.5};        // s after the last non-zero wheel command that the robot counts as moving
  double max_linear{0.306};         // m/s
  double max_angular{1.9};          // rad/s
};

struct Inputs
{
  double now{0.0};  // s, from any steadily increasing clock

  std::string mode;           // "idle", "mapping", "navigation", "manual", "cleaning", "docking", "docked", "failsafe"
  bool transitioning{false};  // the mode manager is in the middle of a mode change
  double mode_time{kNever};

  Velocity manual;
  double manual_time{kNever};
  Velocity nav;
  double nav_time{kNever};

  std::array<double, kCliffSensors> cliff_range{{0.0, 0.0, 0.0, 0.0}};  // m to the floor; inf = nothing seen
  std::array<double, kCliffSensors> cliff_time{{kNever, kNever, kNever, kNever}};
  std::uint8_t bumper{0};  // bit mask of pressed zones, 0 = not pressed
  double bumper_time{kNever};
};

struct Output
{
  Velocity command;            // what goes to the wheels
  Source source{Source::None}; // who is allowed to drive
  std::string blocked_reason;  // empty when the source's command went through unchanged
  bool cliff{false};
  bool bumper{false};
  bool off_floor{false};
  bool reflex_active{false};
  std::vector<std::string> stale;  // "mode", "cliff", "bumper"
};

class GateLogic
{
public:
  explicit GateLogic(const Config & config = Config());

  // Decide the wheel command for this instant. Call at a steady rate with increasing `now`.
  Output update(const Inputs & in);

  const Config & config() const {return config_;}

private:
  Config config_;
  bool reflex_active_{false};
  double reflex_start_{0.0};
  bool front_hazard_before_{false};
  double hazard_clear_since_{kNever};  // when the front hazard was last seen to go away
  double forward_pause_until_{kNever};
  double last_motion_{kNever};         // when a non-zero wheel command was last sent
};

const char * toString(Source source);

}  // namespace vacuum_safety

#endif  // VACUUM_SAFETY__GATE_LOGIC_HPP_
