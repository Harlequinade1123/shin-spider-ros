// Dump motions in the golden-data text format.
//   shin_spider_dump                 -> all default cases
//   shin_spider_dump 'walkingMotion(0,3)' 'sweepingMotion(20)'
#include <iostream>

#include "shin_spider/MotionTestHook.h"
#include "shin_spider/processing.h"

int main(int argc, char** argv)
{
  using namespace shin_spider;
  processing::setPrintlnHandler([](const std::string&) {});  // silence "START: ..."

  std::vector<std::string> cases;
  for (int i = 1; i < argc; i++) cases.push_back(argv[i]);
  if (cases.empty()) cases = test_hook::defaultCases();

  int rc = 0;
  for (const std::string& name : cases)
  {
    std::vector<test_hook::MotionOutput> out;
    std::string err;
    if (!test_hook::runCase(name, out, &err))
    {
      std::cerr << "ERROR: " << err << std::endl;
      rc = 1;
      continue;
    }
    std::cout << test_hook::formatCase(name, out);
  }
  return rc;
}
