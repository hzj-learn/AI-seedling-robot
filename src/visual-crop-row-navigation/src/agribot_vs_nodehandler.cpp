/***************************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields              */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Chis McCool, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany                        */
/* maintainer: Alireza Ahmadi                                                          */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)                    */
/***************************************************************************************/

#include "agribot_vs_nodehandler.h"
#include "agribot_vs.h"
#include <time.h>

namespace agribot_vs {

AgribotVSNodeHandler::AgribotVSNodeHandler(ros::NodeHandle& nodeHandle): nodeHandle_(nodeHandle){
  ROS_ERROR("Visual Servoing core is running...");
  if (!agribotVS.readRUNParmas(nodeHandle_)) {
     ROS_ERROR("Could not read parameters.");
     ros::requestShutdown();
  }

  // Subscribers
  image_front_sub = nodeHandle_.subscribe("/front_camera/image_raw", 2, &AgribotVSNodeHandler::imageFrontCalllBack,this);
  image_back_sub = nodeHandle_.subscribe("/rear_camera/image_raw", 2, &AgribotVSNodeHandler::imageBackCalllBack,this);
  Mocap_sub = nodeHandle_.subscribe("/amcl_pose", 1, &AgribotVSNodeHandler::amclPoseCallBack,this);
  Odom_sub = nodeHandle_.subscribe("/odom", 10, &AgribotVSNodeHandler::odomCallBack,this);
  IMU_sub = nodeHandle_.subscribe("/imu", 1000, &AgribotVSNodeHandler::imuCallBack,this);
  odom_fuse_sub = nodeHandle_.subscribe("/odom_combined",100,&AgribotVSNodeHandler::odomFuseCallBack,this);
  
  // Publishers
  Time_pub = nodeHandle_.advertise<rosgraph_msgs::Clock>("/clock", 1);
  VSVelocityPub = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel/input", 1);
  Log_pub = nodeHandle_.advertise<visual_crop_row_navigation::vs_msg>("/vs_msg", 1);
  pathPub = nodeHandle_.advertise<nav_msgs::Path>("/move_path",50);

  agribotVS.VelocityMsg.linear.x =0.0;
  agribotVS.VelocityMsg.angular.z =0.0;

  state = 0;
  in_state = 0;
}
AgribotVSNodeHandler::~AgribotVSNodeHandler() {
}
void AgribotVSNodeHandler::CropRow_Tracking(camera& src){
    // finding contour from image baed on crops in rows
    src.contours = agribotVS.CropRowFeatures(src);
    if(!agribotVS.mask_tune && src.contours.size() != 0){

      src.points = agribotVS.getContureCenters(src.image, src.contours);
      
      // src.nh_points = agribotVS.filterContures(src.image, src.contours);  // 可省略

      agribotVS.is_in_neigbourhood(src);

      src.lines =  agribotVS.FitLineOnContures(src.image, src.nh_points);
    }else{
      cout << "Number of contures: " << src.contours.size() << endl;
      //publishVelocity(0);
    }
}
void AgribotVSNodeHandler::imageFrontCalllBack(const sensor_msgs::ImageConstPtr& msg) {
  try {
    agribotVS.front_cam.image = cv_bridge::toCvShare(msg, "bgr8")->image;
    CropRow_Tracking(agribotVS.front_cam);
    // cv::imwrite("/home/xag/data/temp/1.jpg", agribotVS.front_cam.image);   //采集图片

    string str;
    stringstream stream,stream1,stream2;
    stream << agribotVS.front_cam.points.size(); 
    stream1 << agribotVS.front_cam.nh_points.size();
    stream2 << agribotVS.camera_ID;
    str = "Number of Points: " + stream.str() + " nh_points: " + stream1.str() + " Cam ID: " + stream2.str();
    cv::putText(agribotVS.front_cam.image, str, cv::Point(40, 20),  // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL,       // Font
                0.75,                                 // Scale. 2.0 = 2x bigger
                cv::Scalar(0, 0, 255),                // BGR Color
                1);                                   // Line Thickness (Optional)

  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void AgribotVSNodeHandler::imageBackCalllBack(const sensor_msgs::ImageConstPtr& msg) {
  try {
    agribotVS.back_cam.image = cv_bridge::toCvShare(msg, "bgr8")->image;
    CropRow_Tracking(agribotVS.back_cam);

    string str;
    stringstream stream,stream1,stream2;
    stream << agribotVS.back_cam.points.size(); 
    stream1 << agribotVS.back_cam.nh_points.size(); 
    stream2 << agribotVS.camera_ID;
    str = "Number of Points: " + stream.str() + " nh_points: " + stream1.str() + " Cam ID: " + stream2.str();
    cv::putText(agribotVS.back_cam.image, str, cv::Point(40, 20),  // Coordinates
                cv::FONT_HERSHEY_COMPLEX_SMALL,       // Font
                0.75,                                 // Scale. 2.0 = 2x bigger
                cv::Scalar(0, 0, 255),                // BGR Color
                1);                                   // Line Thickness (Optional)
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
void AgribotVSNodeHandler::imuCallBack(const sensor_msgs::Imu::ConstPtr& msg){
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3 m(quat);
  m.getRPY(imu_roll, imu_pitch, imu_yaw);
}
void AgribotVSNodeHandler::amclPoseCallBack(const geometry_msgs::PoseWithCovarianceStamped& msg) {
  tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                   msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(mocap_roll, mocap_pitch, mocap_yaw);
  agribotVS.RotationVec[0] = 0;    
  agribotVS.RotationVec[1] = 0;    
  agribotVS.RotationVec[2] = mocap_yaw;  
  agribotVS.TransVec[0] = msg.pose.pose.position.x;
  agribotVS.TransVec[1] = msg.pose.pose.position.y;
  agribotVS.TransVec[2] = msg.pose.pose.position.z;
}
void AgribotVSNodeHandler::odomCallBack(const nav_msgs::Odometry::ConstPtr& msg) {
  agribotVS.RobotPose[0] = msg->pose.pose.position.x;
  agribotVS.RobotPose[1] = msg->pose.pose.position.y;
  agribotVS.RobotPose[2] = msg->pose.pose.position.z;

  std::vector<double> Orineration = agribotVS.getEulerAngles(msg);

  agribotVS.RobotPose[3] = Orineration[0];  // x oreintation
  agribotVS.RobotPose[4] = Orineration[1];  // y oreintation
  agribotVS.RobotPose[5] = Orineration[2];  // z oreintation

  agribotVS.RobotLinearVelocities[0] = msg->twist.twist.linear.x;
  agribotVS.RobotLinearVelocities[1] = msg->twist.twist.linear.y;
  agribotVS.RobotLinearVelocities[2] = msg->twist.twist.linear.z;

  agribotVS.RobotAngularVelocities[0] = msg->twist.twist.angular.x;
  agribotVS.RobotAngularVelocities[1] = msg->twist.twist.angular.y;
  agribotVS.RobotAngularVelocities[2] = msg->twist.twist.angular.z;

}

void AgribotVSNodeHandler::odomFuseCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{
    //publish path
    geometry_msgs::PoseStamped currentPose;
    currentPose.header.frame_id = "odom_combined";
    currentPose.header.stamp = msg->header.stamp;
    currentPose.pose.position.x = msg->pose.pose.position.x;
    currentPose.pose.position.y = msg->pose.pose.position.y;
    currentPose.pose.position.z = msg->pose.pose.position.z;
  
    current_path_.header = currentPose.header;
    current_path_.poses.push_back(currentPose);
    pathPub.publish(current_path_);
}

void AgribotVSNodeHandler::StopForSec(float delay) {
  agribotVS.VelocityMsg.angular.z = 0.0;
  agribotVS.VelocityMsg.linear.x = 0.0;
  if(agribotVS.publish_cmd_vel)VSVelocityPub.publish(agribotVS.VelocityMsg);
  ros::Duration(delay).sleep();  // sleep for half a second
}
void AgribotVSNodeHandler::publishVelocity(int _in) {
  if(!agribotVS.publish_linear_vel)agribotVS.VelocityMsg.linear.x = 0.0;
  if(_in == 0){
    agribotVS.VelocityMsg.linear.x = 0.0;  //线速度
    agribotVS.VelocityMsg.angular.z = 0.0;  //角速度
    if(agribotVS.publish_cmd_vel)
    VSVelocityPub.publish(agribotVS.VelocityMsg);
  }else{
    if(agribotVS.publish_cmd_vel){
      VSVelocityPub.publish(agribotVS.VelocityMsg);
    }
  }
  Log_pub.publish(agribotVS.VSMsg);
}

}   // namespace agribot_vs
