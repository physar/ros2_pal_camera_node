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

#ifndef PAL_CAMERA_NODE_HPP
#define PAL_CAMERA_NODE_HPP

#include "dreamvu/PAL.h"

#include "rclcpp/rclcpp.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <opencv2/highgui/highgui.hpp>

namespace dreamvu {

// ----> Typedefs to simplify declarations
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::Image>> imagePub; // without CamInfo
typedef std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pointcloudPub;

typedef std::shared_ptr<sensor_msgs::msg::Image> imageMsgPtr;
typedef std::shared_ptr<sensor_msgs::msg::CameraInfo> camInfoMsgPtr;
typedef std::shared_ptr<sensor_msgs::msg::PointCloud2> pointcloudMsgPtr;

typedef std::shared_ptr<geometry_msgs::msg::TransformStamped> transfMsgPtr;

// <---- Typedefs to simplify declarations

#define TIMEZERO_SYS rclcpp::Time(0,0,RCL_SYSTEM_TIME)

class PalCameraNode : public rclcpp::Node
{
public:

    explicit PalCameraNode(const rclcpp::NodeOptions & options);

    virtual ~PalCameraNode();

protected:

    bool startCamera();

    void initParameters();
    void initPublishers();
    void initListeners();

    void grab_loop(); // while-loop to be stopped by CTLR-C

    void publishImageWithInfo(cv::Mat& img,
                              image_transport::CameraPublisher& pubImg,
                              camInfoMsgPtr& camInfoMsg,
                              std::string imgFrameId, rclcpp::Time t);

    // Creates the same camInfo for left & right, except for the baseline
    // the baseline is the distance in m between left and right image. Estimated on 4cm
    // Instead, a rotation over 30 deg (0.523598776 rad) would be more appropriate

    void defineCamInfo(camInfoMsgPtr leftCamInfoMsg,
                       camInfoMsgPtr rightCamInfoMsg,
                       std::string leftFrameId, std::string rightFrameId,
                       float baseline = 0.04);

    void publishPalCameraMounting2CenterTransform(rclcpp::Time t);
    void publishBase2PalCameraTransform(rclcpp::Time t);
    void publishMap2BaseTransform(rclcpp::Time t);

private:

    // ----> Utility functions
    template<typename T>
    void getParam(std::string paramName, T defValue, T& outVal, std::string log_info=std::string());
    // <---- Utility functions

    // ----> pal-camera initialization Info
    int mCamWidth;  // Camera frame width
    int mCamHeight; // Camera frame height
    PAL::CameraProperties mCameraProperties;
    // <---- pal-camera initialization Info

    // ----> Parameter variables
    std::string mCameraModel = "pal_usb";
    std::string mCameraName = "/dreamvu/pal/";
    // <---- Parameter variables

 
    // ----> Variables initiated during Class creation
    rclcpp::QoS mVideoQos;
    // <---- Variables initiated during Class creation

    // ----> Threads and Timers
    // std::thread mGrabThread;
    // <---- Threads and Timers

    // ----> Publishers
    image_transport::CameraPublisher mPubLeft; // Image with CamInfo
    image_transport::CameraPublisher mPubRight; //
    image_transport::CameraPublisher mPubDepth; //
    pointcloudPub                    mPubCloud; //
    // <---- Publishers

    // ----> Camera infos
    camInfoMsgPtr mLeftCamInfoMsg;
    camInfoMsgPtr mRightCamInfoMsg;
    camInfoMsgPtr mDepthCamInfoMsg;
    // <---- Camera infos
    
   // ----> TF Transforms Flags
    // bool mCamera2BaseTransfValid = false;
    // <---- TF Transforms Flags

       // ----> Coordination Frame IDs
    std::string mMapFrameId = "map"; // three default coordination frames
    std::string mOdomFrameId = "odom";
    std::string mBaseFrameId = "base_link";

    std::string mCameraCenterFrameId = "camera_center";
    std::string mMountingBottomFrameId = "mounting_bottom";
    //std::string mLeftCamFrameId;
    //std::string mRightCamFrameId;
    //std::string mDepthFrameId;
    //std::string mPointCloudFrameId;
    // <---- Coordination Frame IDs

    // ----> initialization Transform listener & broadcaster
    std::unique_ptr<tf2_ros::Buffer> mTfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> mTfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> mTfBroadcaster;
    // <---- initialization Transform listener

}; // end of class PalCameraNode

} // end of namespace dreamvu

#endif // PAL_CAMERA_NODE_HPP

