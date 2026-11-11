// Golden-data comparison against the output of the original Processing code
// (dumped by a Java stub). Skips when the golden file does not exist.
#include <fstream>
#include <iostream>
#include <sstream>

#include <gtest/gtest.h>

#include "shin_spider/MotionTestHook.h"

using namespace shin_spider;

namespace
{
struct GoldenStep
{
  int index = 0;
  int time_threshold = 0;
  std::string v, k, m;
};
struct GoldenMotion
{
  std::string name;
  std::vector<GoldenStep> steps;
};
struct GoldenCase
{
  std::string name;
  std::vector<GoldenMotion> motions;
};

std::string rstrip(std::string s)
{
  while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' ' || s.back() == '\t')) s.pop_back();
  return s;
}

bool parseGolden(std::istream& in, std::vector<GoldenCase>& cases, std::string& error,
                 std::vector<std::string>* robot_block = nullptr)
{
  std::string line;
  int lineno = 0;
  auto next = [&](std::string& out) {
    while (std::getline(in, out))
    {
      lineno++;
      out = rstrip(out);
      if (!out.empty() && out[0] != '#') return true;
    }
    return false;
  };
  while (next(line))
  {
    if (line == "ROBOT")
    {
      std::vector<std::string> block{line};
      while (line != "ENDROBOT")
      {
        if (!next(line)) { error = "unterminated ROBOT block"; return false; }
        block.push_back(line);
      }
      if (robot_block) *robot_block = block;
    }
    else if (line.rfind("CASE ", 0) == 0)
    {
      cases.push_back(GoldenCase{line.substr(5), {}});
    }
    else if (line.rfind("MOTION ", 0) == 0)
    {
      if (cases.empty()) { error = "MOTION before CASE at line " + std::to_string(lineno); return false; }
      cases.back().motions.push_back(GoldenMotion{line.substr(7), {}});
    }
    else if (line.rfind("STEP ", 0) == 0)
    {
      if (cases.empty() || cases.back().motions.empty())
      {
        error = "STEP before MOTION at line " + std::to_string(lineno);
        return false;
      }
      GoldenStep s;
      std::istringstream ss(line.substr(5));
      if (!(ss >> s.index >> s.time_threshold)) { error = "bad STEP at line " + std::to_string(lineno); return false; }
      if (!next(s.v) || !next(s.k) || !next(s.m)) { error = "truncated STEP at line " + std::to_string(lineno); return false; }
      cases.back().motions.back().steps.push_back(s);
    }
    else
    {
      error = "unexpected line " + std::to_string(lineno) + ": " + line;
      return false;
    }
  }
  return true;
}

std::string opt(const std::optional<std::string>& s) { return s ? *s : "NULL"; }

struct Quiet
{
  Quiet() { processing::setPrintlnHandler([](const std::string&) {}); }
} quiet;
}  // namespace

// The format writer and the parser above agree (independent of golden data).
TEST(GoldenFormat, RoundTripOwnDump)
{
  std::ostringstream all;
  for (const auto& name : test_hook::defaultCases())
  {
    std::vector<test_hook::MotionOutput> out;
    std::string err;
    ASSERT_TRUE(test_hook::runCase(name, out, &err)) << err;
    all << test_hook::formatCase(name, out);
  }
  std::istringstream in(all.str());
  std::vector<GoldenCase> cases;
  std::string error;
  ASSERT_TRUE(parseGolden(in, cases, error)) << error;
  EXPECT_EQ(cases.size(), test_hook::defaultCases().size());
}

TEST(Golden, MatchesProcessingOutput)
{
  std::ifstream in(SHIN_SPIDER_GOLDEN_PATH);
  if (!in)
  {
    GTEST_SKIP() << "golden file not found: " << SHIN_SPIDER_GOLDEN_PATH
                 << " (generate it with the Java/Processing stub, then re-run)";
  }
  std::vector<GoldenCase> cases;
  std::string error;
  std::vector<std::string> robot_block;
  ASSERT_TRUE(parseGolden(in, cases, error, &robot_block)) << error;
  ASSERT_FALSE(cases.empty()) << "golden file has no CASE";

  if (!robot_block.empty())
  {
    SpiderController fresh(nullptr, nullptr);
    std::vector<std::string> mine = test_hook::dumpRobot(fresh.robot);
    ASSERT_EQ(mine.size(), robot_block.size()) << "ROBOT block size";
    for (size_t i = 0; i < mine.size(); i++) EXPECT_EQ(mine[i], robot_block[i]) << "ROBOT block line " << i;
  }

  int compared_steps = 0;
  for (const GoldenCase& gc : cases)
  {
    SCOPED_TRACE("CASE " + gc.name);
    std::vector<test_hook::MotionOutput> out;
    std::string err;
    if (!test_hook::runCase(gc.name, out, &err))
    {
      ADD_FAILURE() << "cannot run case: " << err;
      continue;
    }
    ASSERT_EQ(out.size(), gc.motions.size()) << "number of motions differs";
    for (size_t i = 0; i < out.size(); i++)
    {
      const auto& gm = gc.motions[i];
      const auto& cm = out[i];
      EXPECT_EQ(cm.motion_name, gm.name);
      ASSERT_EQ(cm.steps.size(), gm.steps.size()) << "steps differ in motion " << gm.name;
      for (size_t s = 0; s < cm.steps.size(); s++)
      {
        const auto& g = gm.steps[s];
        const auto& c = cm.steps[s];
        std::string where = gm.name + " step " + std::to_string(g.index);
        EXPECT_EQ(c.index, g.index) << where;
        EXPECT_EQ(c.time_threshold, g.time_threshold) << where;
        EXPECT_EQ(opt(c.v), g.v) << where;
        EXPECT_EQ(opt(c.k), g.k) << where;
        EXPECT_EQ(opt(c.m), g.m) << where;
        compared_steps++;
      }
    }
  }
  std::cout << "[ INFO     ] golden: " << cases.size() << " cases, " << compared_steps << " steps compared"
            << std::endl;
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
