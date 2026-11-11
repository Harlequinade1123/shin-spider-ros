class Motion
{
  // member variable
  String motion_name      = "";
  Target[][] targets      = {}; 
  int time_thresholds[]   = {};
  int motion_index_num    = 0;
  int motion_index        = 0;
  int motion_time         = 0;
  int[] motion_ids_nums   = {-1, 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0,
                                 0, 0, 0, 0, 0, 0};
  int[] motion_ids        = {};
  boolean is_moving       = false;
  boolean command_is_sent = false;

  /*** member function
  * コンストラクタ
  * Motion(String tmp_motion_name, int target_num)
  * 目標値を追加
  * void appendTarget(int motion_order, Target target, int time_threshold)
  * 目標値のOpenCM用コマンド作成
  * String getMotionCommand()
  * 速度制限のOpenCM用コマンド作成
  * String getVelocityCommand()
  * 加速度制限のOpenCM用コマンド作成
  * String getAccelerationCommand()
  * IDが存在しているかどうか
  * boolean idExists(int id)
  * boolean idExists(int[] ids)
  * void setIds()
  * void resetMotionIndex()
  * 動作開始
  * void start()
  * 経過時間がtime_threshold以上なら次の目標値へと移行する
  * void next()
  ***/

  Motion(String tmp_motion_name, int target_num)
  {
    motion_name = tmp_motion_name;
    for (int i = 0; i < target_num; i++)
    {
      targets = (Target[][])append(targets, new Target[0]);
      time_thresholds = (int[])append(time_thresholds, 0);
    }
    motion_index_num = target_num;
    motion_index = target_num;
  }

  void appendTarget(int motion_order, Target target, int time_threshold)
  {
    if (motion_order >= targets.length)
    {
      println("ERROR: motion_order is too large");
    }
    else
    {
      targets[motion_order] = (Target[])append(targets[motion_order], target);
      time_thresholds[motion_order] = time_threshold;
      if (target.target_id == 0)
      {
        for (int i = 0; i < target.target_ids.length; i++)
        {
          motion_ids_nums[target.target_ids[i]]++;
        }
      }
      else
      {
        motion_ids_nums[target.target_id]++;
      }
    }
    setIds();
  }

  String getMotionCommand()
  {
    if (command_is_sent)
    {
      return null;
    }
    String ans_msg = "[m";
    if (targets.length == 0 || motion_index == motion_index_num)
    {
      return null;
    }
    for (int i = 0; i < targets[motion_index].length; i++)
    {
      ans_msg += targets[motion_index][i].getTargetCommand();
    }
    ans_msg += "]\n";
    command_is_sent = true;
    return ans_msg;
  }

  String getVelocityCommand()
  {
    String ans_msg = "[v";
    if (targets.length == 0 || motion_index == motion_index_num)
    {
      return null;
    }
    for (int i = 0; i < targets[motion_index].length; i++)
    {
      ans_msg += targets[motion_index][i].getTargetVelocityCommand();
    }
    ans_msg += "]\n";
    return ans_msg;
  }

  String getAccelerationCommand()
  {
    String ans_msg = "[k";
    if (targets.length == 0 || motion_index == motion_index_num)
    {
      return null;
    }
    for (int i = 0; i < targets[motion_index].length; i++)
    {
      ans_msg += targets[motion_index][i].getTargetAccelerationCommand();
    }
    ans_msg += "]\n";
    return ans_msg;
  }

  boolean idExists(int id)
  {
    if (motion_ids_nums[id] > 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  boolean idExists(int[] ids)
  {
    int ids_num = 0;
    for (int i = 0; i < ids.length; i++)
    {
      ids_num += motion_ids_nums[ids[i]];
    }
    if (ids_num > 0)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void setIds()
  {
    int[] ans_motion_ids = {};
    for(int i = 0; i < motion_ids_nums.length; i++)
    {
      if (motion_ids_nums[i] > 0)
      {
        ans_motion_ids = (int[])append(ans_motion_ids, i);
      }
    }
    motion_ids = ans_motion_ids;
  }

  void resetMotionIndex()
  {
    motion_index = 0;
  }

  void start()
  {
    if (is_moving == false)
    {
      is_moving = true;
      command_is_sent = false;
      motion_index = 0;
      motion_time = millis();
    }
  }

  void next()
  {
    if (motion_index < motion_index_num)
    {
      if (millis() - motion_time >= time_thresholds[motion_index])
      {
        motion_index++;
        motion_time = millis();
        command_is_sent = false;
      }
    }
    else if (command_is_sent && motion_index == motion_index_num - 1)
    {
      is_moving = false;
      motion_index++;
    }
    else if (motion_index == motion_index_num)
    {
      is_moving = false;
    }
  }
};
