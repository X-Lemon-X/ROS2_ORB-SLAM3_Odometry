#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "System.h"

bool
arg_to_bool(char* argv)
{
  std::string vis_arg = argv;
  std::transform(vis_arg.begin(), vis_arg.end(), vis_arg.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return (vis_arg == "true" || vis_arg == "1");
};

int
main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings "
                 "visualization"
              << std::endl;
    return 1;
  }

  rclcpp::init(argc, argv);

  bool visualization = arg_to_bool(argv[3]);

  ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

  auto node = std::make_shared<StereoSlamNode>(&pSLAM);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
