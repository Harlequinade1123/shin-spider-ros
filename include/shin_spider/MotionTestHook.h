// Public test hook: run a motion-generating call on a fresh SpiderController
// and step every generated Motion through all motion_index values.
//
// Golden text format (shared with the Java/Processing dumper):
//   CASE <case-name>
//   MOTION <motion_name>            (one per generated motion, motion_list order)
//   STEP <idx> <time_threshold_ms>  (one per motion_index 0..motion_index_num-1)
//   <[v...] string or NULL>
//   <[k...] string or NULL>
//   <[m...] string or NULL>
// Command strings are written without their trailing "\n".
//
// Case names are calls of the Processing functions, e.g.
//   walkingMotion(0,3)  walkingMotion(PI/2,0)  rotationalMotion(-1.5707964,-PI/4)
//   sweepingMotion(-20)  wavingHandsMotion()  testLegMotion(1)
// Several calls can be chained with ';' (executed in order on the same state).
// Also accepted (format of the Java dumper): named arguments
//   walkingMotion(steering=1.5707964,mode=0)  and sequences  seq[a(...);b(...)]
// Arguments: decimal numbers (parsed as Java Float.parseFloat would) or
// [-]PI[/n], [-]HALF_PI, [-]QUARTER_PI. Separators '(' ',' ')' and spaces.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "shin_spider/Motion.h"
#include "shin_spider/SpiderController.h"

namespace shin_spider
{
namespace test_hook
{

struct StepOutput
{
  int index = 0;
  int time_threshold = 0;
  std::optional<std::string> v, k, m;  // without trailing "\n"; nullopt == NULL
};

struct MotionOutput
{
  std::string motion_name;
  std::vector<StepOutput> steps;
};

// Sets motion_index = idx, command_is_sent = false and collects v, k, m for
// every idx. The Motion's index/sent flag are restored afterwards.
std::vector<StepOutput> stepMotionAllIndices(Motion& motion);

// Executes `call` (see grammar above) on `controller`. Returns false + error on
// unknown function / bad args / exception.
bool runCall(SpiderController& controller, const std::string& call, std::string* error);

// Fresh controller (no clock, no sink), run case, dump all motions.
bool runCase(const std::string& case_name, std::vector<MotionOutput>& out, std::string* error);

std::string formatCase(const std::string& case_name, const std::vector<MotionOutput>& motions);

// Robot state block written at the top of the golden file:
//   ROBOT / LEG i motor_IDs=.. rad_thetas=.. coor_rad_theta=.. origin_steering=.. origin_xyz=.. /
//   HAND i motor_IDs=.. rad_thetas=.. coor_rad_theta=.. / ENDROBOT  (floats as Java Float.toString)
std::vector<std::string> dumpRobot(const Robot& robot);

// Default case list covering every motion function and every argument value
// reachable from gamepad_command (steering in {PI/2, 0, -PI/2}).
std::vector<std::string> defaultCases();

}  // namespace test_hook
}  // namespace shin_spider
