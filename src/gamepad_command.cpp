// Port of gamepad_command.pde. Keyboard inputs (arrow keys, is_simulation test
// keys) are not ported: `keyPressed` is always false here.
#include <cstdlib>

#include "shin_spider/SpiderController.h"

namespace shin_spider
{
using namespace processing;

void SpiderController::readGamePadCommand()
{
  if (slider_FR > slider_f_threshold)
  {
    if (pred_slider_front <= slider_f_threshold && steering_state < 2)
    {
      bool rot_motion_is_moving = false;
      for (size_t i = 0; i < motion_list.motions.size(); i++)
      {
        if (motion_list.motions[i].motion_name.find("rotational+") != std::string::npos)
        {
          rot_motion_is_moving = rot_motion_is_moving || motion_list.motions[i].is_moving;
        }
      }
      if (rot_motion_is_moving == false)
      {
        steering_state++;
        robot_steering = static_cast<float>(PI * (1 - steering_state) / 2);
        if (steering_state == 1)
        {
          rotationalMotion90(robot_steering, PI / 2);
        }
        else
        {
          rotationalMotion90(robot_steering, 0);
        }
      }
    }
    pred_slider_front = slider_FR;
  }
  else if (slider_FR < -slider_f_threshold)
  {
    if (pred_slider_front >= -slider_f_threshold && steering_state > 0)
    {
      bool rot_motion_is_moving = false;
      for (size_t i = 0; i < motion_list.motions.size(); i++)
      {
        if (motion_list.motions[i].motion_name.find("rotational+") != std::string::npos)
        {
          rot_motion_is_moving = rot_motion_is_moving || motion_list.motions[i].is_moving;
        }
      }
      if (rot_motion_is_moving == false)
      {
        steering_state--;
        robot_steering = static_cast<float>(PI * (1 - steering_state) / 2);
        if (steering_state == 1)
        {
          rotationalMotion90(robot_steering, -PI / 2);
        }
        else
        {
          rotationalMotion90(robot_steering, 0);
        }
      }
    }
    pred_slider_front = slider_FR;
  }
  else
  {
    pred_slider_front = slider_FR;
  }

  if (button_LB == 1 && button_RB == 1)
  {
    write("[n]\n");
    delay(10);
    return;
  }

  // buttons
  if (button_A == 1)
  {
    wavingHandsMotion();
  }
  else if (button_B == 1)
  {
    if (button_LB == 1)
    {
      sweepingMotion(-20);
    }
    else if (button_RB == 1)
    {
      sweepingMotion(20);
    }
    else
    {
      sweepingMotion(0);
    }
  }
  else if (button_X == 1)
  {
    foldingMotion();
  }
  else if (button_Y == 1)
  {
    resetMotion(robot_steering);
  }

  // helper replicating the walking_time_counts bookkeeping of each branch
  auto walkBranch = [this](int state, int count_index, auto&& start_motion) {
    if (walking_state_L == state || walking_time_counts[count_index] > walking_time_threshold)
    {
      for (size_t i = 0; i < walking_time_counts.size(); i++)
      {
        walking_time_counts[i] = 0;
      }
      start_motion();
      walking_state_L = state;
    }
    else
    {
      for (size_t i = 0; i < walking_time_counts.size(); i++)
      {
        if (static_cast<int>(i) == count_index)
        {
          walking_time_counts[i]++;
        }
        else
        {
          walking_time_counts[i] = 0;
        }
      }
    }
  };

  // joysticks
  if (slider_LX > slider_x_threshold && std::abs(slider_LY) < slider_y_threshold)  // L down
  {
    walkBranch(0, 0, [this] { walkingMotion(robot_steering, 3); });
  }
  else if (slider_LX < -slider_x_threshold && std::abs(slider_LY) < slider_y_threshold)  // L up
  {
    walkBranch(0, 0, [this] { walkingMotion(robot_steering, 0); });
  }
  else if (slider_LY > slider_y_threshold && std::abs(slider_LX) < slider_x_threshold)  // L right
  {
    walkBranch(3, 3, [this] { sideWalkingMotion(robot_steering, 1); });
  }
  else if (slider_LY < -slider_y_threshold && std::abs(slider_LX) < slider_x_threshold)  // L left
  {
    walkBranch(3, 3, [this] { sideWalkingMotion(robot_steering, 0); });
  }
  else if (slider_LX > slider_x_threshold && slider_LY > slider_y_threshold)  // L down-right
  {
    walkBranch(2, 2, [this] { walkingMotion(robot_steering, 5); });
  }
  else if (slider_LX > slider_x_threshold && slider_LY < -slider_y_threshold)  // L down-left
  {
    walkBranch(1, 1, [this] { walkingMotion(robot_steering, 4); });
  }
  else if (slider_LX < -slider_x_threshold && slider_LY > slider_y_threshold)  // L up-right
  {
    walkBranch(1, 1, [this] { walkingMotion(robot_steering, 1); });
  }
  else if (slider_LX < -slider_x_threshold && slider_LY < -slider_y_threshold)  // L up-left
  {
    walkBranch(2, 2, [this] { walkingMotion(robot_steering, 2); });
  }
  else if (slider_RX > slider_x_threshold && std::abs(slider_RY) < slider_y_threshold)  // R down
  {
    backJumpingMotion(robot_steering, 0);
  }
  else if (slider_RX < -slider_x_threshold && std::abs(slider_RY) < slider_y_threshold)  // R up
  {
    jumpingMotion(robot_steering, 0);
  }
  else if (slider_RY > slider_y_threshold && std::abs(slider_RX) < slider_x_threshold)  // R right
  {
    jumpingMotion(robot_steering, 1);
  }
  else if (slider_RY < -slider_y_threshold && std::abs(slider_RX) < slider_x_threshold)  // R left
  {
    jumpingMotion(robot_steering, 2);
  }
  else if (slider_RX > slider_x_threshold && slider_RY > slider_y_threshold)  // R down-right
  {
    backJumpingMotion(robot_steering, 1);
  }
  else if (slider_RX > slider_x_threshold && slider_RY < -slider_y_threshold)  // R down-left
  {
    backJumpingMotion(robot_steering, 1);
  }
  else if (slider_RX < -slider_x_threshold && slider_RY > slider_y_threshold)  // R up-right
  {
    jumpingMotion(robot_steering, 1);
  }
  else if (slider_RX < -slider_x_threshold && slider_RY < -slider_y_threshold)  // R up-left
  {
    jumpingMotion(robot_steering, 2);
  }

  // D-pad (hat)
  if (hat_XY == 2)
  {
    spreadMotion(robot_steering);
  }
  else if (hat_XY == 8)
  {
    rotationalMotion(robot_steering, -PI / 4);
  }
  else if (hat_XY == 4)
  {
    rotationalMotion(robot_steering, PI / 4);
  }
  else if (hat_XY == 6)
  {
    shrinkMotion(robot_steering);
  }
  // hat_XY 1,3,5,7: no-op in the original
}

}  // namespace shin_spider
