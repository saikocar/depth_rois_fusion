#include <memory>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <image_geometry/pinhole_camera_model.h>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::placeholders::_1;

class DepthImageGenerator : public rclcpp::Node
{
public:
  DepthImageGenerator() : Node("depth_image_generator")
  {
    using QoS = rclcpp::QoS;
    using ReliabilityPolicy = rclcpp::ReliabilityPolicy;

    // パラメータ宣言と取得
    declare_parameter("pointcloud_topic", "/sensing/lidar/concatenated/pointcloud");
    declare_parameter("camera_info_topic", "/sensing/camera/roscube/front_wide/camera_info");
    declare_parameter("image_topic", "/sensing/camera/roscube/front_wide/image_rect");
    declare_parameter("rois_topic", "/perception/object_recognition/detection/rois0");
    declare_parameter("depth_image_topic", "/camera/depth_image");
    declare_parameter("depth_rois_image_topic", "/camera/depth_rois_image");
    declare_parameter("out_camera_info_topic", "/camera/camera_info");
    declare_parameter("rectangle_color", std::vector<int64_t>{0, 255, 0});
    declare_parameter("text_color", std::vector<int64_t>{255, 0, 0});
    declare_parameter("rois_depth_msg_topic", "/camera/rois_depth");
    declare_parameter("depth_rois_marker_topic", "/camera/rois_depth_marker");

    get_parameter("pointcloud_topic", pointcloud_topic_);
    get_parameter("camera_info_topic", camera_info_topic_);
    get_parameter("image_topic", image_topic_);
    get_parameter("rois_topic", rois_topic_);
    get_parameter("depth_image_topic", depth_image_topic_);
    get_parameter("depth_rois_image_topic", depth_rois_image_topic_);
    get_parameter("out_camera_info_topic", out_camera_info_topic_);
    get_parameter("rectangle_color", rectangle_color_);
    get_parameter("text_color", text_color_);
    get_parameter("rois_depth_msg_topic", rois_depth_msg_topic_);
    get_parameter("depth_rois_marker_topic", depth_rois_marker_topic_);

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Publisher
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_image_topic_, 10);
    depth_rois_pub_ = create_publisher<sensor_msgs::msg::Image>(depth_rois_image_topic_, 10);
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(out_camera_info_topic_, 10);
    rois_depth_msg_pub_ = create_publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(rois_depth_msg_topic_, 10);
    depth_rois_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(depth_rois_marker_topic_, 10);

    // Subscriber
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic_, 10, std::bind(&DepthImageGenerator::camera_info_callback, this, _1));

    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, QoS(10).best_effort(), std::bind(&DepthImageGenerator::pointcloud_callback, this, _1));

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_, 10, std::bind(&DepthImageGenerator::image_callback, this, _1));

    rois_sub_ = create_subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>(
      rois_topic_, 10, std::bind(&DepthImageGenerator::rois_callback, this, _1));
  }

private:
  std::string pointcloud_topic_, camera_info_topic_, image_topic_, rois_topic_;
  std::string depth_image_topic_, depth_rois_image_topic_, out_camera_info_topic_;
  std::string depth_rois_marker_topic_, rois_depth_msg_topic_;
  std::vector<int64_t> rectangle_color_, text_color_;

  sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
  sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
  cv::Mat undistorted_depth_, latest_debug_image_, latest_depth_float_image_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_, depth_rois_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr rois_depth_msg_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr depth_rois_marker_pub_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<tier4_perception_msgs::msg::DetectedObjectsWithFeature>::SharedPtr rois_sub_;
  void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    //RCLCPP_INFO(get_logger(), "camera_info_callback: %.3f", rclcpp::Time(msg->header.stamp).seconds());
    camera_info_ = msg;
    camera_info_pub_->publish(*msg);
  }

  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_pointcloud_ = msg;
  }

  void image_callback(const sensor_msgs::msg::Image::SharedPtr image_msg)
  {
    if (!camera_info_ || !latest_pointcloud_) {
      RCLCPP_WARN(get_logger(), "Missing camera info or pointcloud");
      return;
    }

    // 時間差が大きすぎるならスキップ（例：0.2秒以上）
    rclcpp::Time pc_time(latest_pointcloud_->header.stamp);
    rclcpp::Time img_time(image_msg->header.stamp);
    if (std::abs((pc_time - img_time).seconds()) > 0.2) {
      RCLCPP_WARN(get_logger(), "Timestamp mismatch: pc=%.3f, img=%.3f", pc_time.seconds(), img_time.seconds());
      return;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        camera_info_->header.frame_id,
        latest_pointcloud_->header.frame_id,
        latest_pointcloud_->header.stamp,
        rclcpp::Duration::from_seconds(0.2));
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "Transform error: %s", e.what());
      return;
    }

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*camera_info_);
    int width = camera_info_->width;
    int height = camera_info_->height;

    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*latest_pointcloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*latest_pointcloud_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*latest_pointcloud_, "z");

    cv::Mat depth_image(height, width, CV_32FC1, cv::Scalar(0));

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      Eigen::Vector3d pt_cam = eigen_transform * Eigen::Vector3d(*iter_x, *iter_y, *iter_z);
      if (pt_cam.z() <= 0) continue;
      cv::Point2d uv = cam_model.project3dToPixel(cv::Point3d(pt_cam.x(), pt_cam.y(), pt_cam.z()));
      int u = std::round(uv.x);
      int v = std::round(uv.y);
      if (u >= 0 && u < width && v >= 0 && v < height) {
        float & current = depth_image.at<float>(v, u);
        if (current == 0 || pt_cam.z() < current) {
          current = pt_cam.z();
        }
      }
    }

    undistorted_depth_ = depth_image.clone();
    latest_depth_float_image_ = undistorted_depth_.clone();
    depth_image.convertTo(latest_debug_image_, CV_8UC1, 255.0 / (cv::minMaxLoc(depth_image, nullptr, nullptr), 1.0));

    auto out_msg = cv_bridge::CvImage(image_msg->header, "32FC1", undistorted_depth_).toImageMsg();
    out_msg->header.frame_id = camera_info_->header.frame_id;
    depth_pub_->publish(*out_msg);
  }

  void rois_callback(const tier4_perception_msgs::msg::DetectedObjectsWithFeature::SharedPtr msg)
  {
    if (latest_debug_image_.empty() || latest_depth_float_image_.empty()) {
      RCLCPP_INFO(get_logger(), "Debug image not available yet");
      return;
    }

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*camera_info_);

    cv::Mat debug_image_with_rois;
    cv::cvtColor(latest_debug_image_, debug_image_with_rois, cv::COLOR_GRAY2BGR);

    visualization_msgs::msg::MarkerArray marker_array;
    auto output_msg = *msg;

    int id_counter = 0;
    for (auto & obj : output_msg.feature_objects) {
      const auto & roi = obj.feature.roi;
      int x = roi.x_offset, y = roi.y_offset, w = roi.width, h = roi.height;
      if (x < 0 || y < 0 || x + w > latest_depth_float_image_.cols || y + h > latest_depth_float_image_.rows) continue;

      cv::Mat roi_depth = latest_depth_float_image_(cv::Rect(x, y, w, h));

      cv::Mat temp_valid;
      cv::threshold(roi_depth, temp_valid, 0, 1, cv::THRESH_BINARY);
      cv::Mat valid;
      temp_valid.convertTo(valid, CV_8U);

      double minVal;
      cv::minMaxLoc(roi_depth, &minVal, nullptr, nullptr, nullptr, valid);

      std::vector<float> valid_values;
      for (int i = 0; i < roi_depth.rows; ++i) {
        for (int j = 0; j < roi_depth.cols; ++j) {
          float val = roi_depth.at<float>(i, j);
          if (val > 0) valid_values.push_back(val);
        }
      }

      float depth = minVal;
      // float depth = 0.0;
      // if (!valid_values.empty()) {
      //   std::nth_element(valid_values.begin(), valid_values.begin() + valid_values.size() / 2, valid_values.end());
      //   depth = valid_values[valid_values.size() / 2];
      // }

      cv::Point2d center_pixel(x + w / 2.0, y + h / 2.0);
      cv::Point3d center_ray = cam_model.projectPixelTo3dRay(center_pixel);
      cv::Point3d center_xyz = center_ray * depth;

      obj.object.kinematics.pose_with_covariance.pose.position.x = center_xyz.x;
      obj.object.kinematics.pose_with_covariance.pose.position.y = center_xyz.y;
      obj.object.kinematics.pose_with_covariance.pose.position.z = center_xyz.z;
      obj.object.kinematics.pose_with_covariance.pose.orientation.w = 1.0;

      double fx = cam_model.fx();
      double fy = cam_model.fy();
      double width_m = (w * depth) / fx;
      double height_m = (h * depth) / fy;
      double depth_m = 0.4;//半分がmargin相当

      obj.object.shape.dimensions.x = depth_m;
      obj.object.shape.dimensions.y = width_m;
      obj.object.shape.dimensions.z = height_m;
      obj.object.shape.type = autoware_auto_perception_msgs::msg::Shape::BOUNDING_BOX;

      visualization_msgs::msg::Marker marker;
      marker.header = msg->header;
      marker.ns = "rois_depth";
      marker.id = id_counter++;
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose = obj.object.kinematics.pose_with_covariance.pose;
      marker.scale.x = width_m;
      marker.scale.y = height_m;
      marker.scale.z = depth_m;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(marker);

      cv::rectangle(debug_image_with_rois, cv::Rect(x, y, w, h), cv::Scalar(rectangle_color_[2], rectangle_color_[1], rectangle_color_[0]), 2);
      char text[100];
      snprintf(text, sizeof(text), "Min: %.2f m", minVal);
      cv::putText(debug_image_with_rois, text, cv::Point(x, std::max(10, y - 5)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(text_color_[2], text_color_[1], text_color_[0]), 1);
    }

    auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", debug_image_with_rois).toImageMsg();
    out_msg->header.frame_id = msg->header.frame_id;
    depth_rois_pub_->publish(*out_msg);

    depth_rois_marker_pub_->publish(marker_array);
    rois_depth_msg_pub_->publish(output_msg);
  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DepthImageGenerator>());
  rclcpp::shutdown();
  return 0;
}
