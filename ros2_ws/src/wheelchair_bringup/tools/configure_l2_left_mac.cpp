// One-time hardware configuration utility for the wheelchair's left Unitree L2.

#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include "unitree_lidar_sdk.h"

namespace {

constexpr char kApplyArgument[] = "--apply";
constexpr char kLidarIp[] = "192.168.1.63";
constexpr char kJetsonIp[] = "192.168.1.2";
constexpr unsigned short kLidarPort = 6101;
constexpr unsigned short kJetsonPort = 6202;
constexpr std::uint8_t kLeftMac[6] = {0x02, 0x29, 0xab, 0x7c, 0x00, 0x63};

void print_plan() {
  std::cout
      << "Left L2 persistent MAC configuration\n"
      << "  Current lidar endpoint: " << kLidarIp << ':' << kLidarPort << '\n'
      << "  Jetson receive endpoint: " << kJetsonIp << ':' << kJetsonPort << '\n'
      << "  New left-L2 MAC: 02:29:ab:7c:00:63\n"
      << "  Required: right L2 powered off; no process bound to UDP 6202.\n";
}

}  // namespace

int main(int argc, char* argv[]) {
  print_plan();
  if (argc != 2 || std::string(argv[1]) != kApplyArgument) {
    std::cout << "Dry run only. Re-run with --apply to modify the left L2.\n";
    return argc == 1 ? 0 : 2;
  }

  auto* reader = unilidar_sdk2::createUnitreeLidarReader();
  if (reader->initializeUDP(
          kLidarPort,
          kLidarIp,
          kJetsonPort,
          kJetsonIp)) {
    std::cerr << "Could not initialize the left L2 UDP connection.\n";
    return 1;
  }

  std::this_thread::sleep_for(std::chrono::seconds(1));

  unilidar_sdk2::LidarMacAddressConfig config{};
  for (std::size_t index = 0; index < 6; ++index) {
    config.mac[index] = kLeftMac[index];
  }
  config.reserve[0] = 0;
  config.reserve[1] = 0;

  reader->setLidarMacAddressConfig(config);
  std::this_thread::sleep_for(std::chrono::seconds(1));

  std::cout
      << "MAC configuration command sent. Power-cycle the left L2 before "
      << "verification.\n";
  return 0;
}
