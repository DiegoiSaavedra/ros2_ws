/**
 * @file main.cpp
 * @author LDRobot (support@ldrobot.com)
 * @brief  main process App
 *         This code is only applicable to LDROBOT LiDAR LD06 products 
 * sold by Shenzhen LDROBOT Co., LTD    
 * @version 0.1
 * @date 2021-10-28
 *
 * @copyright Copyright (c) 2021  SHENZHEN LDROBOT CO., LTD. All rights
 * reserved.
 * Licensed under the MIT License (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License in the file LICENSE
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ros2_api.h"
#include "ldlidar_driver.h"

namespace {
constexpr float kDefaultScanMinRange = 0.02F;
constexpr float kDiagnosticEncodingMinRange = 0.001F;
constexpr float kScanMaxRange = 12.0F;

bool IsValidScanMinRange(double value) {
  return std::isfinite(value) && value > 0.0 && value < kScanMaxRange;
}
}  // namespace

void  ToLaserscanMessagePublish(ldlidar::Points2D& src, double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub,
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& near_debug_pub,
  ldlidar::Points2D& near_debug_src);

uint64_t GetSystemTimeStamp(void);

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ldlidar_published"); // create a ROS2 Node
  std::string product_name;
	std::string topic_name;
	std::string port_name;
  int serial_port_baudrate = 230400;
  ldlidar::LDType type_name;
  LaserScanSetting setting;
	setting.frame_id = "base_laser";
  setting.laser_scan_dir = true;
  setting.enable_angle_crop_func = false;
  setting.angle_crop_min = 0.0;
  setting.angle_crop_max = 0.0;
  setting.scan_min_range = kDefaultScanMinRange;
  setting.enable_near_debug = false;
  setting.near_debug_topic = "scan_near_debug";
  
  // declare ros2 param
  node->declare_parameter<std::string>("product_name", product_name);
  node->declare_parameter<std::string>("topic_name", topic_name);
  node->declare_parameter<std::string>("frame_id", setting.frame_id);
  node->declare_parameter<std::string>("port_name", port_name);
  node->declare_parameter<int>("port_baudrate", serial_port_baudrate);
  node->declare_parameter<bool>("laser_scan_dir", setting.laser_scan_dir);
  node->declare_parameter<bool>("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->declare_parameter<double>("angle_crop_min", setting.angle_crop_min);
  node->declare_parameter<double>("angle_crop_max", setting.angle_crop_max);
  node->declare_parameter<double>("scan_min_range", setting.scan_min_range);
  node->declare_parameter<bool>("enable_near_debug", setting.enable_near_debug);
  node->declare_parameter<std::string>("near_debug_topic", setting.near_debug_topic);

  // get ros2 param
  node->get_parameter("product_name", product_name);
  node->get_parameter("topic_name", topic_name);
  node->get_parameter("frame_id", setting.frame_id);
  node->get_parameter("port_name", port_name);
  node->get_parameter("port_baudrate", serial_port_baudrate);
  node->get_parameter("laser_scan_dir", setting.laser_scan_dir);
  node->get_parameter("enable_angle_crop_func", setting.enable_angle_crop_func);
  node->get_parameter("angle_crop_min", setting.angle_crop_min);
  node->get_parameter("angle_crop_max", setting.angle_crop_max);
  node->get_parameter("scan_min_range", setting.scan_min_range);
  node->get_parameter("enable_near_debug", setting.enable_near_debug);
  node->get_parameter("near_debug_topic", setting.near_debug_topic);

  if (!IsValidScanMinRange(setting.scan_min_range)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Invalid scan_min_range %.6f; using safe default %.3f m",
      setting.scan_min_range, kDefaultScanMinRange);
    setting.scan_min_range = kDefaultScanMinRange;
  }
  if (setting.near_debug_topic.empty()) {
    RCLCPP_ERROR(node->get_logger(),
      "near_debug_topic is empty; disabling near-range diagnostic output");
    setting.enable_near_debug = false;
  }

  ldlidar::LDLidarDriver* ldlidarnode = new ldlidar::LDLidarDriver();

  RCLCPP_INFO(node->get_logger(), "LDLiDAR SDK Pack Version is: %s", ldlidarnode->GetLidarSdkVersionNumber().c_str());
  RCLCPP_INFO(node->get_logger(), "<product_name>: %s", product_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<topic_name>: %s", topic_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<frame_id>: %s", setting.frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_name>: %s", port_name.c_str());
  RCLCPP_INFO(node->get_logger(), "<port_baudrate>: %d", serial_port_baudrate);
  RCLCPP_INFO(node->get_logger(), "<laser_scan_dir>: %s", (setting.laser_scan_dir?"Counterclockwise":"Clockwise"));
  RCLCPP_INFO(node->get_logger(), "<enable_angle_crop_func>: %s", (setting.enable_angle_crop_func?"true":"false"));
  RCLCPP_INFO(node->get_logger(), "<angle_crop_min>: %f", setting.angle_crop_min);
  RCLCPP_INFO(node->get_logger(), "<angle_crop_max>: %f", setting.angle_crop_max);
  RCLCPP_INFO(node->get_logger(), "<scan_min_range>: %.3f m", setting.scan_min_range);
  RCLCPP_INFO(node->get_logger(), "<enable_near_debug>: %s",
    setting.enable_near_debug ? "true" : "false");

  if (product_name == "LDLiDAR_LD06") {
    type_name = ldlidar::LDType::LD_06;
  } else if (product_name == "LDLiDAR_LD19") {
    type_name = ldlidar::LDType::LD_19;
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error, input <product_name> is illegal.");
    exit(EXIT_FAILURE);
  }

  ldlidarnode->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); 

  ldlidarnode->EnableFilterAlgorithnmProcess(true);

  if (ldlidarnode->Start(type_name, port_name, serial_port_baudrate, ldlidar::COMM_SERIAL_MODE)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar node start is success");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar node start is fail");
    exit(EXIT_FAILURE);
  }

  if (ldlidarnode->WaitLidarCommConnect(3000)) {
    RCLCPP_INFO(node->get_logger(), "ldlidar communication is normal.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "ldlidar communication is abnormal.");
    exit(EXIT_FAILURE);
  }

  // create ldlidar data topic and publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher = 
      node->create_publisher<sensor_msgs::msg::LaserScan>(topic_name, 10);
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr near_debug_publisher;
  if (setting.enable_near_debug) {
    near_debug_publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
      setting.near_debug_topic, 10);
    RCLCPP_WARN(node->get_logger(),
      "Near-range diagnostic enabled on <%s>; production <%s> remains filtered at %.3f m",
      setting.near_debug_topic.c_str(), topic_name.c_str(), setting.scan_min_range);
  }
  
  rclcpp::WallRate r(10); //10hz

  ldlidar::Points2D laser_scan_points;
  ldlidar::Points2D raw_laser_scan_points;
  double lidar_scan_freq;
  RCLCPP_INFO(node->get_logger(), "Publish topic message:ldlidar scan data.");
  while (rclcpp::ok()) {
    ldlidar::LidarStatus scan_status;
    if (setting.enable_near_debug) {
      scan_status = ldlidarnode->GetLaserScanData(
        laser_scan_points, raw_laser_scan_points, 1500);
    } else {
      scan_status = ldlidarnode->GetLaserScanData(laser_scan_points, 1500);
    }
    switch (scan_status) {
      case ldlidar::LidarStatus::NORMAL: 
        ldlidarnode->GetLidarScanFreq(lidar_scan_freq);
        ToLaserscanMessagePublish(
          laser_scan_points, lidar_scan_freq, setting, node, publisher,
          near_debug_publisher, raw_laser_scan_points);
        break;
      case ldlidar::LidarStatus::DATA_TIME_OUT:
        RCLCPP_ERROR(node->get_logger(), "get ldlidar data is time out, please check your lidar device.");
        break;
      case ldlidar::LidarStatus::DATA_WAIT:
        break;
      default:
        break;
    }

    r.sleep();
  }

  ldlidarnode->Stop();

  delete ldlidarnode;
  ldlidarnode = nullptr;

  RCLCPP_INFO(node->get_logger(), "ldlidar_published is end");
  rclcpp::shutdown();

  return 0;
}

void  ToLaserscanMessagePublish(ldlidar::Points2D& src,  double lidar_spin_freq, LaserScanSetting& setting,
  rclcpp::Node::SharedPtr& node, rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& lidarpub,
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr& near_debug_pub,
  ldlidar::Points2D& near_debug_src) {
  float angle_min, angle_max, range_min, range_max, angle_increment;
  double scan_time;
  rclcpp::Time start_scan_time;
  static rclcpp::Time end_scan_time;
  static bool first_scan = true;

  start_scan_time = node->now();
  scan_time = (start_scan_time.seconds() - end_scan_time.seconds());

  if (first_scan) {
    first_scan = false;
    end_scan_time = start_scan_time;
    return;
  }
  // Adjust the parameters according to the demand
  angle_min = 0;
  angle_max = (2 * M_PI);
  // El valor de produccion coincide con el minimo nominal del LD19. El topico
  // diagnostico separado conserva la vuelta anterior al filtro del SDK.
  range_min = static_cast<float>(setting.scan_min_range);
  range_max = kScanMaxRange;
  int beam_size = static_cast<int>(src.size());
  if (beam_size <= 1) {
    RCLCPP_WARN(node->get_logger(), "Discarding scan with %d beam(s)", beam_size);
    return;
  }
  angle_increment = (angle_max - angle_min) / (float)(beam_size -1);
  // Calculate the number of scanning points
  if (lidar_spin_freq > 0) {
    sensor_msgs::msg::LaserScan output;
    output.header.stamp = start_scan_time;
    output.header.frame_id = setting.frame_id;
    output.angle_min = angle_min;
    output.angle_max = angle_max;
    output.range_min = range_min;
    output.range_max = range_max;
    output.angle_increment = angle_increment;
    output.time_increment = static_cast<float>(scan_time / (double)(beam_size - 1));
    output.scan_time = scan_time;
    // First fill all the data with Nan
    output.ranges.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    output.intensities.assign(beam_size, std::numeric_limits<float>::quiet_NaN());
    sensor_msgs::msg::LaserScan near_debug_output = output;
    near_debug_output.range_min = kDiagnosticEncodingMinRange;

    auto store_point = [](sensor_msgs::msg::LaserScan& scan, int index,
                          float range, float intensity) {
      if (std::isnan(scan.ranges[index]) ||
          (!std::isnan(range) && range < scan.ranges[index])) {
        scan.ranges[index] = range;
        scan.intensities[index] = intensity;
      }
    };

    for (auto point : src) {
      float range = point.distance / 1000.f;  // distance unit transform to meters
      float intensity = point.intensity;      // laser receive intensity 
      float dir_angle = point.angle;

      // El SDK representa los puntos descartados con distancia cero.
      if (point.distance == 0) {
        range = std::numeric_limits<float>::quiet_NaN();
        intensity = std::numeric_limits<float>::quiet_NaN();
      }

      float production_range = range;
      float production_intensity = intensity;
      if (!std::isnan(production_range) && production_range < range_min) {
        production_range = std::numeric_limits<float>::quiet_NaN();
        production_intensity = std::numeric_limits<float>::quiet_NaN();
      }

      if (setting.enable_angle_crop_func) { // Angle crop setting, Mask data within the set angle range
        if ((dir_angle >= setting.angle_crop_min) && (dir_angle <= setting.angle_crop_max)) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
          production_range = std::numeric_limits<float>::quiet_NaN();
          production_intensity = std::numeric_limits<float>::quiet_NaN();
        }
      }

      float angle = ANGLE_TO_RADIAN(dir_angle); // Lidar angle unit form degree transform to radian
      int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
      if (index < beam_size) {
        if (index < 0) {
          RCLCPP_ERROR(node->get_logger(), "error index: %d, beam_size: %d, angle: %f, output.angle_min: %f, output.angle_increment: %f", 
            index, beam_size, angle, angle_min, angle_increment);
        }

        if (setting.laser_scan_dir) {
          int index_anticlockwise = beam_size - index - 1;
          store_point(output, index_anticlockwise,
            production_range, production_intensity);
        } else {
          store_point(output, index, production_range, production_intensity);
        }
      }
    }

    // La salida diagnostica se forma con la copia de la misma vuelta anterior
    // al NearFilter del SDK. No modifica ni reemplaza la rama de produccion.
    if (near_debug_pub) {
      for (auto point : near_debug_src) {
        float range = point.distance / 1000.f;
        float intensity = point.intensity;
        float dir_angle = point.angle;

        if (point.distance == 0) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }
        if (setting.enable_angle_crop_func &&
            dir_angle >= setting.angle_crop_min &&
            dir_angle <= setting.angle_crop_max) {
          range = std::numeric_limits<float>::quiet_NaN();
          intensity = std::numeric_limits<float>::quiet_NaN();
        }

        float angle = ANGLE_TO_RADIAN(dir_angle);
        int index = static_cast<int>(ceil((angle - angle_min) / angle_increment));
        if (index >= 0 && index < beam_size) {
          int output_index = setting.laser_scan_dir ? beam_size - index - 1 : index;
          store_point(near_debug_output, output_index, range, intensity);
        }
      }
    }
    lidarpub->publish(output);
    if (near_debug_pub) {
      near_debug_pub->publish(near_debug_output);
    }
    end_scan_time = start_scan_time;
  } 
}

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

/********************* (C) COPYRIGHT SHENZHEN LDROBOT CO., LTD *******END OF
 * FILE ********/
