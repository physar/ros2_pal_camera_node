/********************************************************************************
 * MIT License
 *
 * Copyright (c) 2021 Arnoud Visser - creating a PAL-camera capture node for ROS2
 * Copyright (c) 2020 Stereolabs - substantial portions of this code was inspired by StereoLabs zed_wrapper for ROS2
 * Copyright (c) 2020 DreamVU - substantial portions of this code were based on DreamVU orginal ROS1 node
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 ********************************************************************************/

#include "rclcpp/rclcpp.hpp"
#include "dreamvu/pal_camera_node.hpp"


int main(int argc, char** argv)
{

  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Note ROS2 typically uses its parameter system, instead of command-line arguments
  rclcpp::init(argc, argv);

  // the executor will contain the main-loop of the node
  rclcpp::executors::SingleThreadedExecutor exec;

  // the options with parameters from the ROS parameter system
  auto options = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);

  // the pal-camera capture node
  auto capture_node = std::make_shared<dreamvu::PalCameraNode>(options);

  // start the main loop
  exec.add_node(capture_node);
  exec.spin();

  RCLCPP_INFO(capture_node->get_logger(), "Shutting down ");
  rclcpp::shutdown();

  return 0;
}
