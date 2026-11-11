// ROS1 node: /joy -> gamepad_command logic -> [v]/[k]/[m] lines to OpenCM.
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <chrono>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include "shin_spider/FirmwareProtocol.h"
#include "shin_spider/JoyMapper.h"
#include "shin_spider/SerialPort.h"
#include "shin_spider/SpiderController.h"

namespace shin_spider
{

class RosClock : public Clock
{
public:
  RosClock() : t0_(ros::Time::now()) {}
  int millis() override { return static_cast<int>((ros::Time::now() - t0_).toNSec() / 1000000); }

private:
  ros::Time t0_;
};

class NodeSink : public CommandSink
{
public:
  NodeSink(SerialPort* port, ros::Publisher* pub) : port_(port), pub_(pub) {}

  void write(const std::string& data) override
  {
    firmware::Validation v = firmware::validateLine(data);
    for (const std::string& e : v.errors)
    {
      ROS_WARN_THROTTLE(1.0, "firmware format check: %s (line: %s)", e.c_str(), strip(data).c_str());
    }
    for (const std::string& w : v.warnings)
    {
      ROS_WARN_THROTTLE(5.0, "firmware limit check: %s", w.c_str());
    }

    if (port_ && port_->isOpen())
    {
      if (!port_->write(data))
      {
        ROS_WARN("Serial write failed (%s). Device disconnected?", std::strerror(errno));
        port_->close();  // main loop will try to reconnect
      }
    }
    else
    {
      ROS_INFO("[dry-run] %s", strip(data).c_str());
    }
    if (pub_)
    {
      std_msgs::String msg;
      msg.data = strip(data);
      pub_->publish(msg);
    }
  }

  void delay(int ms) override { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

private:
  static std::string strip(const std::string& s)
  {
    std::string r = s;
    while (!r.empty() && r.back() == '\n') r.pop_back();
    return r;
  }
  SerialPort* port_;
  ros::Publisher* pub_;
};

}  // namespace shin_spider

int main(int argc, char** argv)
{
  using namespace shin_spider;
  ros::init(argc, argv, "shin_spider_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  processing::setPrintlnHandler([](const std::string& msg) { ROS_INFO("%s", msg.c_str()); });

  std::string device_name, joy_topic;
  int baudrate;
  double loop_rate;
  bool dry_run, publish_commands;
  pnh.param<std::string>("device_name", device_name, "/dev/ttyUSB0");
  pnh.param<int>("baudrate", baudrate, 57600);
  pnh.param<std::string>("joy_topic", joy_topic, "/joy");
  pnh.param<double>("loop_rate", loop_rate, sketch::frame_rate);
  pnh.param<bool>("dry_run", dry_run, false);
  pnh.param<bool>("publish_commands", publish_commands, true);
  bool reconnect;
  double reconnect_interval, reconnect_wait;
  pnh.param<bool>("reconnect", reconnect, true);
  pnh.param<double>("reconnect_interval", reconnect_interval, 1.0);  // [s] retry period (kingspider: 1 s)
  pnh.param<double>("reconnect_wait", reconnect_wait, 2.0);          // [s] settle time after reopen

  JoyMapping m;
  auto axisParam = [&pnh](const std::string& name, AxisMapping& a) {
    pnh.param<int>("axes/" + name + "/index", a.index, a.index);
    double scale = a.scale;
    pnh.param<double>("axes/" + name + "/scale", scale, scale);
    a.scale = static_cast<float>(scale);
  };
  axisParam("slider_LX", m.slider_LX);
  axisParam("slider_LY", m.slider_LY);
  axisParam("slider_RX", m.slider_RX);
  axisParam("slider_RY", m.slider_RY);
  pnh.param<std::string>("axes/slider_FR/mode", m.fr_mode, m.fr_mode);
  pnh.param<int>("axes/slider_FR/left_trigger_axis", m.fr_left_trigger_axis, m.fr_left_trigger_axis);
  pnh.param<int>("axes/slider_FR/right_trigger_axis", m.fr_right_trigger_axis, m.fr_right_trigger_axis);
  double fr_scale = m.fr_scale;
  pnh.param<double>("axes/slider_FR/scale", fr_scale, fr_scale);
  m.fr_scale = static_cast<float>(fr_scale);
  m.fr_axis.scale = m.fr_scale;
  pnh.param<int>("axes/slider_FR/index", m.fr_axis.index, m.fr_axis.index);

  pnh.param<int>("buttons/A", m.button_A, m.button_A);
  pnh.param<int>("buttons/B", m.button_B, m.button_B);
  pnh.param<int>("buttons/X", m.button_X, m.button_X);
  pnh.param<int>("buttons/Y", m.button_Y, m.button_Y);
  pnh.param<int>("buttons/LB", m.button_LB, m.button_LB);
  pnh.param<int>("buttons/RB", m.button_RB, m.button_RB);
  pnh.param<int>("buttons/Back", m.button_Back, m.button_Back);
  pnh.param<int>("buttons/Start", m.button_Start, m.button_Start);
  pnh.param<int>("buttons/LA", m.button_LA, m.button_LA);
  pnh.param<int>("buttons/RA", m.button_RA, m.button_RA);

  pnh.param<std::string>("hat/mode", m.hat_mode, m.hat_mode);
  pnh.param<int>("hat/axis_x", m.hat_axis_x, m.hat_axis_x);
  pnh.param<int>("hat/axis_y", m.hat_axis_y, m.hat_axis_y);
  pnh.param<bool>("hat/x_left_positive", m.hat_x_left_positive, m.hat_x_left_positive);
  pnh.param<bool>("hat/y_up_positive", m.hat_y_up_positive, m.hat_y_up_positive);
  double hat_threshold = m.hat_threshold;
  pnh.param<double>("hat/threshold", hat_threshold, hat_threshold);
  m.hat_threshold = static_cast<float>(hat_threshold);
  pnh.param<int>("hat/button_up", m.hat_button_up, m.hat_button_up);
  pnh.param<int>("hat/button_down", m.hat_button_down, m.hat_button_down);
  pnh.param<int>("hat/button_left", m.hat_button_left, m.hat_button_left);
  pnh.param<int>("hat/button_right", m.hat_button_right, m.hat_button_right);

  SerialPort port;
  if (dry_run)
  {
    ROS_WARN("dry_run=true: commands are logged/published only");
  }
  else
  {
    std::string err;
    if (port.open(device_name, baudrate, &err))
    {
      ROS_INFO("Serial open: %s @ %d", device_name.c_str(), baudrate);
      ros::Duration(0.5).sleep();  // Processing: delay(500) after clear()
    }
    else
    {
      ROS_WARN("Serial failed %s (%s): running in dry-run mode", device_name.c_str(), err.c_str());
    }
  }

  ros::Publisher pub;
  if (publish_commands)
  {
    pub = pnh.advertise<std_msgs::String>("commands", 100);
  }

  RosClock clock;
  NodeSink sink(&port, publish_commands ? &pub : nullptr);
  SpiderController controller(&clock, &sink);
  JoyMapper mapper(m);
  std::mutex mtx;

  ros::Subscriber sub = nh.subscribe<sensor_msgs::Joy>(joy_topic, 10, [&](const sensor_msgs::Joy::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx);
    mapper.apply(msg->axes, msg->buttons, controller);
  });

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("shin_spider_node started: joy_topic=%s device=%s baud=%d rate=%.1fHz", joy_topic.c_str(),
           device_name.c_str(), baudrate, loop_rate);

  // Reopen the serial device when it was never opened / disconnected (cf. kingspider ensureSerialConnected()).
  ros::Time last_try(0);
  auto ensureSerial = [&]() {
    if (dry_run || !reconnect) return;
    if (port.isOpen())
    {
      // detect unplugging before the next write
      if (access(device_name.c_str(), F_OK) == 0) return;
      ROS_WARN("Serial device %s disappeared.", device_name.c_str());
      port.close();
    }
    ros::Time now = ros::Time::now();
    if ((now - last_try).toSec() < reconnect_interval) return;
    last_try = now;
    std::string err;
    if (!port.open(device_name, baudrate, &err))
    {
      ROS_WARN_THROTTLE(5.0, "Waiting for serial device %s ... (%s)", device_name.c_str(), err.c_str());
      return;
    }
    ROS_INFO("Serial device %s reconnected.", device_name.c_str());
    ros::Duration(reconnect_wait).sleep();
  };

  ros::Rate rate(loop_rate);
  while (ros::ok())
  {
    ensureSerial();
    {
      std::lock_guard<std::mutex> lock(mtx);
      controller.frame();
    }
    rate.sleep();
  }
  spinner.stop();
  return 0;
}
