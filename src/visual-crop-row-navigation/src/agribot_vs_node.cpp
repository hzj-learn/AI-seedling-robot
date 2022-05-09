/***************************************************************************************/
/* Paper: Visual-Servoing based Navigation for Monitoring Row-Crop Fields              */
/*    Alireza Ahmadi, Lorenzo Nardi, Nived Chebrolu, Chis McCool, Cyrill Stachniss     */
/*         All authors are with the University of Bonn, Germany                        */
/* maintainer: Alireza Ahmadi                                                          */
/*          (Alireza.Ahmadi@uni-bonn.de / http://alirezaahmadi.xyz)                    */
/***************************************************************************************/
#include <ros/ros.h>
#include "agribot_vs_nodehandler.h"
#include "agribot_vs.h"

#include "std_msgs/String.h"
#include <sstream>
#include <time.h>

int main(int argc, char** argv) {
  // initialize node
  ros::init(argc, argv, "agribot_vs");

  // node handler
  ros::NodeHandle nodeHandle;
  agribot_vs::AgribotVSNodeHandler vs_NodeH(nodeHandle);
  ros::Rate loop_rate(vs_NodeH.agribotVS.fps);

  if(!vs_NodeH.agribotVS.mask_tune)cout << "Mask Tune Mode ..." << endl;
  if(vs_NodeH.agribotVS.single_camera_mode)cout << "Single Camera Mode (Front Camera will only be used..)" << endl;

  agribot_vs::camera *I_primary,*I_secondary;
  if(vs_NodeH.agribotVS.camera_ID == 1){
    I_primary = &vs_NodeH.agribotVS.front_cam;
    I_secondary = &vs_NodeH.agribotVS.back_cam;
  }else {
    I_primary = &vs_NodeH.agribotVS.back_cam;
    I_secondary = &vs_NodeH.agribotVS.front_cam;
  }
  vs_NodeH.agribotVS.initialize_neigbourhood(*I_primary);
  vs_NodeH.agribotVS.initialize_neigbourhood(*I_secondary);
  int cnt =0;
  
  while(ros::ok()){

    if(cnt < vs_NodeH.agribotVS.max_row_num){           //max_row_num=3000

      if(vs_NodeH.agribotVS.single_camera_mode){    // single_camera_mode = false
        I_secondary = I_primary;
      }
      if(!vs_NodeH.agribotVS.mask_tune && I_primary->lines.size()){     // mask_tune = false
        vs_NodeH.agribotVS.switching_controller(*I_primary, *I_secondary, vs_NodeH.agribotVS.min_points_switch);    //谢伟方

        if(vs_NodeH.agribotVS.camera_ID == 1){
          I_primary = &vs_NodeH.agribotVS.front_cam;
          I_secondary = &vs_NodeH.agribotVS.back_cam;
        }else {
          I_primary = &vs_NodeH.agribotVS.back_cam;
          I_secondary = &vs_NodeH.agribotVS.front_cam;
        }

        vs_NodeH.agribotVS.compute_feature_point(*I_primary);  
        vs_NodeH.agribotVS.Controller(*I_primary,*I_secondary);
      
        if(!I_primary->image.empty())     // 当有图像帧存在时执行
        {
          vs_NodeH.agribotVS.draw_neighbourhood(*I_primary);  //绘制检测矩形框
          vs_NodeH.agribotVS.draw_features(*I_primary, vs_NodeH.agribotVS.F_des, cv::Scalar(0, 255, 0));  //绘制绿色箭头线
          vs_NodeH.agribotVS.draw_features(*I_primary, vs_NodeH.agribotVS.F, cv::Scalar(0, 0, 255));   //绘制红色箭头线

          // draw plant centers in image (in neighbourhood)
          for(size_t i = 0; i < I_primary->nh_points.size(); i++){
            // cv::circle(I_primary->image, Point(I_primary->nh_points[i].x,I_primary->nh_points[i].y),5, Scalar(0, 204, 255), CV_FILLED, 8,0);
            //将在矩形框内检测到的作物目标点用红色圆点显示。
            cv::circle(I_primary->image, Point(I_primary->nh_points[i].x,I_primary->nh_points[i].y),5, Scalar(0, 0, 255), CV_FILLED, 8,0);             
          }

          Mat des_comp;
          cv::resize(I_primary->image, des_comp, cv::Size(), vs_NodeH.agribotVS.Scale, vs_NodeH.agribotVS.Scale); // 0.5倍size缩放
          imshow("Cameras", des_comp);
          waitKey(1);
        }else{
          cout << "Is Image empty? Camera-Primary: " << I_primary->image.empty() <<  " , Camera-Secondary "<< I_secondary->image.empty() << endl;
        }
      }

      if(I_primary->points.size() == 0){
        vs_NodeH.publishVelocity(0);
      }else{
        vs_NodeH.publishVelocity();
      }
      
      ros::Time curr_time = ros::Time::now();
      rosgraph_msgs::Clock curr_time_msg;
      curr_time_msg.clock = curr_time;      
      vs_NodeH.Time_pub.publish(curr_time_msg);
    }
    
    if(cnt < 1000)cnt++;
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
