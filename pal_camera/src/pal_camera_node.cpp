/********************************************************************************
 * MIT License
 *
 * Copyright (c) 2021 Arnoud Visser - creating a PAL-camera capture node for ROS2
 * Copyright (c) 2020 Stereolabs - substantial portions of this code was inspired by StereoLabs zed_components for ROS2
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

#include <memory>
#include <string>
#include <thread>

#include <pwd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "dreamvu/PAL.h"
#include "dreamvu/pal_camera_node.hpp"

using namespace std::chrono_literals;

using namespace PAL;

#include <sstream>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/make_shared.hpp>

#include <fcntl.h>
#include <unistd.h>


using namespace std;
using namespace cv;

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <cv_bridge/cv_bridge.h>

namespace dreamvu {

/**
 * Creating a ROS2-node for the PAL camera
 **/
  
PalCameraNode::PalCameraNode(const rclcpp::NodeOptions& options)
  : Node("pal_camera_node", options)
  , mVideoQos(1)
{
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), "     DreamVU PAL Camera v1.1.9.2");
  RCLCPP_INFO(get_logger(), "********************************");
  RCLCPP_INFO(get_logger(), " * namespace: %s", get_namespace());
  RCLCPP_INFO(get_logger(), " * node name: %s", get_name());
  RCLCPP_INFO(get_logger(), "********************************");

  if (startCamera() != true)
  {
    RCLCPP_ERROR(get_logger(), "Failed to start pal-camera node");
    exit(EXIT_FAILURE);
  }
 
}
  
/**
 * Cleaning up when destroying a ROS2-node for the PAL camera
**/
    
PalCameraNode::~PalCameraNode()
{
  RCLCPP_INFO(get_logger(), "Destroying node: %s", get_name());
}

/**
 * Specify the local file path from which the settings are to be read.
 *
 * If the specified file can't be opened, default properties from the API are used.
 *See PAL Documentation for more information.
**/
# define PROPERTIES_FILE_PATH ".local/etc/dreamvu/SavedPalProperties.txt"

/**
 * The intitialization routines specific for the PAL camera
 **/
bool PalCameraNode::startCamera()
{

  RCLCPP_INFO(get_logger(), "***** STARTING CAMERA *****");

  PAL::Acknowledgement ack1 = PAL::Init(mCamWidth, mCamHeight);

  if(ack1 != PAL::SUCCESS)
  {
      RCLCPP_ERROR(get_logger(), "Failed to init the pal-camera");
      RCLCPP_WARN(get_logger(), "Is the pal-camera connected via an USB-3.0 slot?");
      RCLCPP_WARN(get_logger(), "Please check with command 'lsusb | grep See3CAM'");
      return false;
  }
  RCLCPP_INFO(get_logger(), "Default width x height: %d x %d", mCamWidth, mCamHeight);

  PAL::CameraProperties default_properties;
  PAL::GetCameraProperties(&default_properties);

  std::string home_dir = getenv("HOME");
  std::string properties_file = home_dir + "/" + PROPERTIES_FILE_PATH;

  //Loading properties from the file
  PAL::Acknowledgement ack2 = PAL::LoadProperties(properties_file.c_str(),&mCameraProperties);
  
 if(ack2 != PAL::SUCCESS)
  {

    RCLCPP_WARN(get_logger(), "Not able to load PAL settings from properties file at default location:");
    RCLCPP_WARN(get_logger(), properties_file);

    mCameraProperties = default_properties;
  }
  mCamWidth = mCameraProperties.resolution.width;
  mCamHeight = mCameraProperties.resolution.height;

  RCLCPP_INFO(get_logger(), "Current resolution set to width x height: %d x %d", mCamWidth, mCamHeight);

  initListeners();
  initPublishers();

  RCLCPP_INFO(get_logger(), "***** STARTING GRAB LOOP *****");
  grab_loop();

  return true;
}
  
/**
 * the PAL camera also listens, for instance to checks if there 
   a coordination transformation between the base_link of the robot 
   and the center of the map.
 **/
void PalCameraNode::initListeners()
{

  RCLCPP_INFO(get_logger(), "*** LISTENING to TF messages ***");

  mTfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  mTfListener = std::make_shared<tf2_ros::TransformListener>(*mTfBuffer);
}
/**
 * the PAL camera can publish panoramic images, depth images and point clouds
 **/
void PalCameraNode::initPublishers()
{

  RCLCPP_INFO(get_logger(), "*** PUBLISHED TOPICS ***");

  std::string topicPrefix = "/dreamvu/";
  topicPrefix += "pal/";

  // The format as published by the ROS1 node "/dreamvu/pal/get/left";

  std::string left_topic = "/dreamvu/pal/get/left";
  std::string rightTopicRoot = "right";
  std::string depthTopicRoot = "depth";
  std::string cloudTopicRoot = "point_cloud";
  std::string right_topic = topicPrefix + "get/" + rightTopicRoot;
  std::string depth_topic = topicPrefix + "get/" + depthTopicRoot;
  std::string cloud_topic = topicPrefix + "get/" + cloudTopicRoot;


  rclcpp::QoS mVideoQos(1);
  mVideoQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // alternative RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT
  mVideoQos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE); // alternative RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL

  mPubLeft = image_transport::create_camera_publisher(this, left_topic, mVideoQos.get_rmw_qos_profile());
  mPubRight = image_transport::create_camera_publisher(this, right_topic, mVideoQos.get_rmw_qos_profile());
  mPubDepth = image_transport::create_camera_publisher(this, depth_topic, mVideoQos.get_rmw_qos_profile());

  rclcpp::QoS mDepthQos(1); // recommended to use another QoS for the large point cloud
  mPubCloud = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, mDepthQos);

  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubLeft.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubRight.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubDepth.getTopic());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised on topic: " << mPubCloud->get_topic_name());

  mLeftCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();
  mRightCamInfoMsg = std::make_shared<sensor_msgs::msg::CameraInfo>();

  // left and right are the same, except for a 30 deg rotation
  defineCamInfo(mLeftCamInfoMsg, mRightCamInfoMsg, mCameraCenterFrameId, mCameraCenterFrameId);

  mDepthCamInfoMsg = mLeftCamInfoMsg;

  mTfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}
  
/**
 * the panoramic images and depth images are published together with camera model
 *
 *     This function creates the same camInfo for left & right, except for the baseline.
 *     The baseline is the distance in m between left and right image. Estimated on 4cm.
 *     Instead, a rotation over 30 deg (0.523598776 rad) would be more appropriate.
 **/

void PalCameraNode::defineCamInfo(camInfoMsgPtr leftCamInfoMsg,
                                   camInfoMsgPtr rightCamInfoMsg,
                                   std::string leftFrameId,
                                   std::string rightFrameId,
                                   float baseline /* = 0.04 */)
{
  float k1 = -0.225495; // distortion_coefficients of the pal camera
  float k2 = -0.363048;
  float k3 = -0.000477994;
  float p1 = -0.000132753;
  float p2 =  0.0;

  float fy = 1322.0; // camera matrix of the pal camera for resolution 1322x454
  float cx =  454.0;
  float fx = 1322.0; // equal to width
  float cy =  454.0; // equal to height

  leftCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  rightCamInfoMsg->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

  leftCamInfoMsg->d.resize(5); // distortion_coefficients
  rightCamInfoMsg->d.resize(5);
  leftCamInfoMsg->d[0] = k1;
  leftCamInfoMsg->d[1] = k2;
  leftCamInfoMsg->d[2] = k3;
  leftCamInfoMsg->d[3] = p1;
  leftCamInfoMsg->d[4] = p2;
  rightCamInfoMsg->d[0] = k1;
  rightCamInfoMsg->d[1] = k2;
  rightCamInfoMsg->d[2] = k3;
  rightCamInfoMsg->d[3] = p1;
  rightCamInfoMsg->d[4] = p2;

  leftCamInfoMsg->k.fill(0.0); // camera_matrix  3x3
  rightCamInfoMsg->k.fill(0.0);
  leftCamInfoMsg->k[0] = fx;
  leftCamInfoMsg->k[2] = cx;
  leftCamInfoMsg->k[4] = fy;
  leftCamInfoMsg->k[5] = cy;
  leftCamInfoMsg->k[8] = 1.0;
  rightCamInfoMsg->k[0] = fx;
  rightCamInfoMsg->k[2] = cx;
  rightCamInfoMsg->k[4] = fy;
  rightCamInfoMsg->k[5] = cy;
  rightCamInfoMsg->k[8] = 1.0;

  leftCamInfoMsg->r.fill(0.0); // rectification_matrix
  rightCamInfoMsg->r.fill(0.0);
  
  for (size_t i = 0; i < 3; i++) // create identity matrix
  {
    rightCamInfoMsg->r[i + i * 3] = 1;
    leftCamInfoMsg->r[i + i * 3] = 1;
  }


  leftCamInfoMsg->p.fill(0.0);
  rightCamInfoMsg->p.fill(0.0);
  leftCamInfoMsg->p[0] = fx;
  leftCamInfoMsg->p[2] = cx;
  leftCamInfoMsg->p[5] = fy;
  leftCamInfoMsg->p[6] = cy;
  leftCamInfoMsg->p[10] = 1.0;
  // http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html
  rightCamInfoMsg->p[3] = fx * baseline;
  rightCamInfoMsg->p[0] = fx;
  rightCamInfoMsg->p[2] = cx;
  rightCamInfoMsg->p[5] = fy;
  rightCamInfoMsg->p[6] = cy;
  rightCamInfoMsg->p[10] = 1.0;

  leftCamInfoMsg->width = rightCamInfoMsg->width = fx;
  leftCamInfoMsg->height = rightCamInfoMsg->height = cy;

  leftCamInfoMsg->header.frame_id = leftFrameId;
  rightCamInfoMsg->header.frame_id = rightFrameId;
}

/**
 ** the panoramic and depth images published with image_transport, which includes the CameraInfo
 **/
    
void PalCameraNode::publishImageWithInfo(cv::Mat& imgmat, image_transport::CameraPublisher& pubImg, camInfoMsgPtr& camInfoMsg,
                                     std::string imgFrameId, rclcpp::Time t)
{
  //auto image = sl_tools::imageToROSmsg(img, imgFrameId, t);
  sensor_msgs::msg::Image::SharedPtr imgMsg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", imgmat).toImageMsg();

  imgMsg->header.stamp = t;
  imgMsg->header.frame_id = imgFrameId;
  imgMsg->encoding = sensor_msgs::image_encodings::BGR8;

  camInfoMsg->header.stamp = t;

  pubImg.publish(imgMsg, camInfoMsg);  // TODO CHECK FOR ZERO-COPY
}

/**
 * Define the transformation between the center of the camera and the mounting plate of the sensor (6cm difference).
 **/

void PalCameraNode::publishPalCameraMounting2CenterTransform(rclcpp::Time stamp)
{
     transfMsgPtr transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>();

     transformStamped->header.stamp = stamp;
     transformStamped->header.frame_id = "pal_camera_center";
     transformStamped->child_frame_id = "pal_mounting_link";

     // At the moment, message filled by a tranformation defaulted to Identity()
     // TBF, inspired by line 3828 of zed_camera_component

     tf2::Transform mPalCameraMounting2CenterTransf; // Coordinates of the camera frame in base frame

     mPalCameraMounting2CenterTransf.setIdentity();
     tf2::Vector3 translation = mPalCameraMounting2CenterTransf.getOrigin();
     tf2::Quaternion quat = mPalCameraMounting2CenterTransf.getRotation();

     transformStamped->transform.translation.x = translation.x();
     transformStamped->transform.translation.y = translation.y();
     transformStamped->transform.translation.z = translation.z() + 0.06;
     transformStamped->transform.rotation.x = quat.x();
     transformStamped->transform.rotation.y = quat.y();
     transformStamped->transform.rotation.z = quat.z();
     transformStamped->transform.rotation.w = quat.w();

     mTfBroadcaster->sendTransform(*(transformStamped.get()));
}
  
/**
 * Define the transformation between the mounting point of the camera and the base of the robot.
 **/

void PalCameraNode::publishBase2PalCameraTransform(rclcpp::Time stamp)
{
     transfMsgPtr transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>();

     transformStamped->header.stamp = stamp;
     transformStamped->header.frame_id = "pal_mounting_link";
     transformStamped->child_frame_id = "base_link";

     // At the moment, message filled by a tranformation defaulted to Identity()
     // TBF, inspired by line 3828 of zed_camera_component

     tf2::Transform mBase2PalCameraTransf; // Coordinates of the camera frame in base frame

     mBase2PalCameraTransf.setIdentity();
     tf2::Vector3 translation = mBase2PalCameraTransf.getOrigin();
     tf2::Quaternion quat = mBase2PalCameraTransf.getRotation();

     transformStamped->transform.translation.x = translation.x();
     transformStamped->transform.translation.y = translation.y();
     transformStamped->transform.translation.z = translation.z();
     transformStamped->transform.rotation.x = quat.x();
     transformStamped->transform.rotation.y = quat.y();
     transformStamped->transform.rotation.z = quat.z();
     transformStamped->transform.rotation.w = quat.w();

     mTfBroadcaster->sendTransform(*(transformStamped.get()));
}

/**
 * Define the transformation between the base of the robot and the center of the map. Only needed for rviz2 when the PAL camera is not mounted on a robot.
 Assume in that case that the camera is on a table of 1m height.
 **/

void PalCameraNode::publishMap2BaseTransform(rclcpp::Time stamp)
{
     try
  {
    // Save the transformation
    geometry_msgs::msg::TransformStamped m2c =
        mTfBuffer->lookupTransform("map", mCameraCenterFrameId, TIMEZERO_SYS, rclcpp::Duration(0.1));
  }
  catch (tf2::TransformException& ex)
  {
     rclcpp::Clock steady_clock(RCL_STEADY_TIME);
     RCLCPP_DEBUG_THROTTLE(get_logger(), steady_clock, 1.0, "Transform error: %s", ex.what());
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1.0, "The tf from '%s' to '%s' is not available.",
                           "map", mCameraCenterFrameId.c_str());
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1.0, "Normally the tf-chain from from '%s' to '%s' is published by the robot.",
                           "map", "base_link");
      RCLCPP_WARN_THROTTLE(get_logger(), steady_clock, 1.0, "Now the sensor will publish this transformation!");


     transfMsgPtr transformStamped = std::make_shared<geometry_msgs::msg::TransformStamped>();

     transformStamped->header.stamp = stamp;
     transformStamped->header.frame_id = "base_link";
     transformStamped->child_frame_id = "map";

     // At the moment, message filled by a tranformation defaulted to Identity()
     // TBF, inspired by line 3828 of zed_camera_component

     tf2::Transform mMap2BaseTransf; // Coordinates of the camera frame in base frame

     mMap2BaseTransf.setIdentity();
     tf2::Vector3 translation = mMap2BaseTransf.getOrigin();
     tf2::Quaternion quat = mMap2BaseTransf.getRotation();

     transformStamped->transform.translation.x = translation.x();
     transformStamped->transform.translation.y = translation.y();
     transformStamped->transform.translation.z = translation.z() + 1.0;
     transformStamped->transform.rotation.x = quat.x();
     transformStamped->transform.rotation.y = quat.y();
     transformStamped->transform.rotation.z = quat.z();
     transformStamped->transform.rotation.w = quat.w();

     mTfBroadcaster->sendTransform(*(transformStamped.get()));

  } // end of catch
}
  
  
/**
 * The main loop of this sensor node
 **/

void PalCameraNode::grab_loop()
{
  RCLCPP_INFO(get_logger(), "First grab_loop started");

  // infinite while(1) to be stopped by Ctrl+C
  do
  {
     size_t leftSubnumber = 0;
     size_t rightSubnumber = 0;
     size_t depthSubnumber = 0;
     size_t cloudSubnumber = 0;


     try
     {
       leftSubnumber = count_subscribers(mPubLeft.getTopic());
       rightSubnumber = count_subscribers(mPubRight.getTopic());
       depthSubnumber = count_subscribers(mPubDepth.getTopic());
       cloudSubnumber = count_subscribers(mPubCloud->get_topic_name());
     }
     catch (...)
     {
       rcutils_reset_error();
       RCLCPP_DEBUG(get_logger(), "grab_loop: Exception while counting subscribers");
     }

     // Grab images and/or if needed the point cloud (takes longer)
     PAL::Acknowledgement ack;
     PAL::Image g_imgLeft, g_imgRight, g_imgDepth;
     std::vector<PAL::Point> pc;

     if (cloudSubnumber > 0)
     {
         ack = PAL::GetPointCloud(&pc, 0, &g_imgLeft, &g_imgRight, &g_imgDepth, 0);
     } else {
         ack = GrabFrames(&g_imgLeft, &g_imgRight, &g_imgDepth, 0, false, false);
     }
     if(ack != PAL::SUCCESS)
     {
         RCLCPP_WARN(get_logger(), "Not able to grab a frame from the  PAL camera");
         continue; // don't publish
     }
     else
     {
         RCLCPP_INFO_ONCE(get_logger(), "Able to grab first frame from the PAL camera");
     }
     // Publish all that is grabbed
     rclcpp::Time timeStamp = get_clock()->now();
     RCLCPP_INFO_ONCE(get_logger(), "Publishing a transform from camera center to camera mounting");
     publishPalCameraMounting2CenterTransform(timeStamp);
     RCLCPP_INFO_ONCE(get_logger(), "Publishing a transform from the camera mounting to base_link of the robot");
     publishBase2PalCameraTransform(timeStamp);
     RCLCPP_INFO_ONCE(get_logger(), "Publishing a transform base_link of the robot to the center of the map");
     publishMap2BaseTransform(timeStamp);

     // ----> Publish the left image if someone has subscribed to
      if (leftSubnumber > 0)
      {
          RCLCPP_INFO_ONCE(get_logger(), "Publishing first left-image of the  PAL camera");
          cv::Mat mat_left = cv::Mat(g_imgLeft.rows, g_imgLeft.cols, CV_8UC3, g_imgLeft.Raw.u8_data);
          publishImageWithInfo(mat_left, mPubLeft, mLeftCamInfoMsg, mCameraCenterFrameId, timeStamp); // should rotate 90 deg to get image coordinates
      }

     // ----> Publish the right image if someone has subscribed to
      if (rightSubnumber > 0)
      {
          RCLCPP_INFO_ONCE(get_logger(), "Publishing a first right-image of the  PAL camera");
          cv::Mat mat_right = cv::Mat(g_imgRight.rows, g_imgRight.cols, CV_8UC3, g_imgRight.Raw.u8_data);
          publishImageWithInfo(mat_right, mPubRight, mRightCamInfoMsg, mCameraCenterFrameId, timeStamp); // should add an additional 30 deg around the z-axis
      }

     // ----> Publish the depth image if someone has subscribed to
      if (depthSubnumber > 0)
      {
          RCLCPP_INFO_ONCE(get_logger(), "Publishing a first depth-image of the  PAL camera");

          imageMsgPtr depthptr;
          depthptr.reset(new sensor_msgs::msg::Image);

          depthptr->header.stamp = timeStamp;
          depthptr->header.frame_id = mCameraCenterFrameId;

          depthptr->height = g_imgDepth.rows;
          depthptr->width = g_imgDepth.cols;

          int num = 1; // for endianness detection
          depthptr->is_bigendian = !(*(char*)&num == 1);

          depthptr->step = depthptr->width * sizeof(float);
          size_t size = depthptr->step * depthptr->height;
          depthptr->data.resize(size);

          depthptr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;

          memcpy((float*)(&depthptr->data[0]), g_imgDepth.Raw.f32_data, size);

          mDepthCamInfoMsg->header.stamp = timeStamp;

          mPubDepth.publish(depthptr, mDepthCamInfoMsg);  // TODO CHECK FOR ZERO-COPY
      }

// ----> Publish the point cloud if someone has subscribed to
      if (cloudSubnumber > 0)
      {
          RCLCPP_INFO_ONCE(get_logger(), "Publishing a point-cloud of the  PAL camera");
          // Following the logic of DreamVu,
          // see as alternative ZedCamera::publishPointCloud() in zed_camera_component
          std::vector<PAL::Point>* point_data = &pc; // necessary?

          //sensor_msgs::msg::PointCloud2Ptr pointcloudMsg;
          //pointcloudMsg->reset(new sensor_msgs::msg::PointCloud2);
          //pointcloudMsgPtr pointcloudMsg = std::make_shared<sensor_msgs::msg::PointCloud2>();
          sensor_msgs::msg::PointCloud2 pointcloudMsg;

          pointcloudMsg.header.stamp = timeStamp;
          pointcloudMsg.header.frame_id = mCameraCenterFrameId;

          pointcloudMsg.is_bigendian = false;
          pointcloudMsg.is_dense = false;

          sensor_msgs::PointCloud2Modifier modifier(pointcloudMsg);
          pointcloudMsg.point_step = 4 * sizeof(float);

          pointcloudMsg.width = point_data->size();
          pointcloudMsg.height = 1;
          pointcloudMsg.row_step = sizeof(PAL::Point) * point_data->size();

          modifier.setPointCloud2Fields(4,
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "rgb", 1, sensor_msgs::msg::PointField::FLOAT32
          );

          PAL::Point* pointcloudPtr = (PAL::Point*)(&pointcloudMsg.data[0]);

          unsigned long int i;
          PAL::Point *pc_points = &pc[0];
          for (i = 0; i < point_data->size(); i++)
          {
                pointcloudPtr[i].a = pc_points[i].a;
                pointcloudPtr[i].g = pc_points[i].g;
                pointcloudPtr[i].b = pc_points[i].r;
                pointcloudPtr[i].r = pc_points[i].b;

                //unit conversion from centimeter to meter
                pointcloudPtr[i].x = (pc_points[i].x) *0.01;
                pointcloudPtr[i].z = (pc_points[i].y)  *0.01;
                pointcloudPtr[i].y = -(pc_points[i].z) *0.01;

          }

          mPubCloud->publish(pointcloudMsg);
      }

      RCLCPP_INFO_ONCE(get_logger(), "End of first Grab loop");

  } while( rclcpp::ok() ); // end of while(1)

  RCLCPP_INFO(get_logger(), "Grab loop finished");
}

} // end of namespace dreamvu
  
     
 


  

