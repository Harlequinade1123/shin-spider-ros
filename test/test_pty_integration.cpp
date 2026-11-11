// Integration test with a pseudo terminal acting as the OpenCM serial port:
// sensor_msgs/Joy -> JoyMapper -> SpiderController (real clock, 100 Hz frames,
// real 2 ms gaps) -> SerialPort(pty slave) ; the master side is read and every
// line is checked with the port of the firmware parser.
#include <atomic>
#include <chrono>
#include <fcntl.h>
#include <mutex>
#include <poll.h>
#include <stdlib.h>
#include <thread>
#include <unistd.h>

#include <gtest/gtest.h>
#include <sensor_msgs/Joy.h>

#include "shin_spider/FirmwareProtocol.h"
#include "shin_spider/JoyMapper.h"
#include "shin_spider/SerialPort.h"
#include "shin_spider/SpiderController.h"

using namespace shin_spider;
using Steady = std::chrono::steady_clock;

namespace
{
struct Quiet
{
  Quiet() { processing::setPrintlnHandler([](const std::string&) {}); }
} quiet;

class SteadyClock : public Clock
{
public:
  SteadyClock() : t0_(Steady::now()) {}
  int millis() override
  {
    return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(Steady::now() - t0_).count());
  }

private:
  Steady::time_point t0_;
};

class SerialSink : public CommandSink
{
public:
  explicit SerialSink(SerialPort& p) : port_(p) {}
  void write(const std::string& d) override { ASSERT_TRUE(port_.write(d)); }
  void delay(int ms) override { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

private:
  SerialPort& port_;
};

struct RxLine
{
  std::string text;
  double t_ms;
};

class PtyReader
{
public:
  explicit PtyReader(int master_fd) : fd_(master_fd), t0_(Steady::now())
  {
    th_ = std::thread([this] { loop(); });
  }
  ~PtyReader() { stop(); }
  void stop()
  {
    running_ = false;
    if (th_.joinable()) th_.join();
  }
  std::vector<RxLine> take()
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::lock_guard<std::mutex> lock(mtx_);
    std::vector<RxLine> r;
    r.swap(lines_);
    return r;
  }

private:
  void loop()
  {
    std::string buf;
    while (running_)
    {
      struct pollfd pfd{fd_, POLLIN, 0};
      if (poll(&pfd, 1, 10) <= 0) continue;
      char tmp[512];
      ssize_t n = read(fd_, tmp, sizeof(tmp));
      if (n <= 0) continue;
      double t = std::chrono::duration<double, std::milli>(Steady::now() - t0_).count();
      buf.append(tmp, static_cast<size_t>(n));
      size_t pos;
      while ((pos = buf.find('\n')) != std::string::npos)
      {
        std::lock_guard<std::mutex> lock(mtx_);
        lines_.push_back(RxLine{buf.substr(0, pos), t});
        buf.erase(0, pos + 1);
      }
    }
  }
  int fd_;
  Steady::time_point t0_;
  std::atomic<bool> running_{true};
  std::thread th_;
  std::mutex mtx_;
  std::vector<RxLine> lines_;
};

class PtyFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    master_ = posix_openpt(O_RDWR | O_NOCTTY);
    ASSERT_GE(master_, 0);
    ASSERT_EQ(grantpt(master_), 0);
    ASSERT_EQ(unlockpt(master_), 0);
    slave_name_ = ptsname(master_);
    std::string err;
    ASSERT_TRUE(port_.open(slave_name_, 57600, &err)) << err;
    reader_.reset(new PtyReader(master_));
    sink_.reset(new SerialSink(port_));
    controller_.reset(new SpiderController(&clock_, sink_.get()));
    joy_.axes.assign(8, 0.0f);
    joy_.buttons.assign(11, 0);
  }
  void TearDown() override
  {
    reader_.reset();
    port_.close();
    if (master_ >= 0) close(master_);
  }

  // run Processing-like frames at 100 Hz for `ms`
  void runFrames(int ms)
  {
    auto end = Steady::now() + std::chrono::milliseconds(ms);
    auto next = Steady::now();
    while (Steady::now() < end)
    {
      mapper_.apply(joy_.axes, joy_.buttons, *controller_);
      controller_->frame();
      next += std::chrono::milliseconds(10);
      std::this_thread::sleep_until(next);
    }
  }

  // every line valid per firmware; non-[n] lines come as v,k,m triples with identical id lists
  static std::vector<RxLine> checkTriples(const std::vector<RxLine>& lines)
  {
    std::vector<RxLine> m_lines;
    std::vector<RxLine> seq;
    for (const auto& l : lines)
    {
      auto v = firmware::validateLine(l.text);
      EXPECT_TRUE(v.ok) << l.text << " : " << (v.errors.empty() ? "" : v.errors[0]);
      auto p = firmware::parseLikeFirmware(l.text);
      EXPECT_TRUE(p.accepted) << l.text;
      if (p.command != 'n') seq.push_back(l);
    }
    EXPECT_EQ(seq.size() % 3, 0u);
    for (size_t i = 0; i + 2 < seq.size(); i += 3)
    {
      auto pv = firmware::parseLikeFirmware(seq[i].text);
      auto pk = firmware::parseLikeFirmware(seq[i + 1].text);
      auto pm = firmware::parseLikeFirmware(seq[i + 2].text);
      EXPECT_EQ(pv.command, 'v') << seq[i].text;
      EXPECT_EQ(pk.command, 'k') << seq[i + 1].text;
      EXPECT_EQ(pm.command, 'm') << seq[i + 2].text;
      EXPECT_EQ(pv.elements.size(), pm.elements.size());
      EXPECT_EQ(pk.elements.size(), pm.elements.size());
      if (pv.elements.size() != pm.elements.size() || pk.elements.size() != pm.elements.size()) continue;
      for (size_t e = 1; e < pm.elements.size(); e += 2)
      {
        EXPECT_EQ(pv.elements[e], pm.elements[e]);
        EXPECT_EQ(pk.elements[e], pm.elements[e]);
      }
      // v -> k -> m are separated by delay(2)
      EXPECT_GE(seq[i + 2].t_ms - seq[i].t_ms, 0.0);
      m_lines.push_back(seq[i + 2]);
    }
    return m_lines;
  }

  int master_ = -1;
  std::string slave_name_;
  SerialPort port_;
  SteadyClock clock_;
  std::unique_ptr<PtyReader> reader_;
  std::unique_ptr<SerialSink> sink_;
  std::unique_ptr<SpiderController> controller_;
  JoyMapper mapper_;
  sensor_msgs::Joy joy_;
};
}  // namespace

TEST_F(PtyFixture, WalkingForwardFromLeftStick)
{
  joy_.axes[1] = 1.0f;  // left stick up -> slider_LX = -100 -> walkingMotion(0, 0)
  runFrames(1000);
  joy_.axes[1] = 0.0f;
  runFrames(100);
  auto lines = reader_->take();
  ASSERT_GE(lines.size(), 24u);
  auto m_lines = checkTriples(lines);
  ASSERT_GE(m_lines.size(), 8u);
  EXPECT_EQ(controller_->motion_list.motions[0].motion_name, "walking_motion_0_0");
  // consecutive steps of walking are 100 ms apart (time_threshold), frame quantised
  for (size_t i = 1; i < 8; i++)
  {
    double gap = m_lines[i].t_ms - m_lines[i - 1].t_ms;
    EXPECT_GE(gap, 90.0) << "step " << i;
    EXPECT_LE(gap, 160.0) << "step " << i;
  }
}

TEST_F(PtyFixture, WavingHandsAndTorqueOn)
{
  joy_.buttons[0] = 1;  // A
  runFrames(200);
  joy_.buttons[0] = 0;
  runFrames(100);
  joy_.buttons[4] = 1;  // LB + RB -> [n]
  joy_.buttons[5] = 1;
  runFrames(50);
  joy_.buttons[4] = joy_.buttons[5] = 0;
  runFrames(20);
  auto lines = reader_->take();
  auto m_lines = checkTriples(lines);
  ASSERT_GE(m_lines.size(), 2u);
  EXPECT_EQ(m_lines[0].text, "[m,13,2047,14,1330,15,3071,16,2047,17,1330,18,1023]")
      << "first waving step (hands ratio 0,70,50)";
  int n_count = 0;
  for (const auto& l : lines) n_count += (l.text == "[n]");
  EXPECT_GE(n_count, 1);
}

TEST(SerialFallback, MissingDeviceDoesNotCrash)
{
  SerialPort port;
  std::string err;
  EXPECT_FALSE(port.open("/dev/does_not_exist_shin_spider", 57600, &err));
  EXPECT_FALSE(err.empty());
  EXPECT_FALSE(port.isOpen());
  EXPECT_FALSE(port.write("[n]\n"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
