/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017-2018, AUBO Robotics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *       * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *       * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *       * Neither the name of the Southwest Research Institute, nor the names
 *       of its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */



#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <aubo_msgs/JointPos.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include "tf/tf.h"
#include "geometry_msgs/PoseArray.h"

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

using namespace aubo_driver;
#define MAX_JOINT_ACC 100.0/180.0*M_PI  //unit rad/s^2
#define MAX_JOINT_VEL 20.0/180.0*M_PI   //unit rad/s
#define MAX_END_ACC    4                // unit m/s^2
#define MAX_END_VEL    0.02                // unit m/s
class ROSNode
{
public:
  ROSNode(ros::NodeHandle &_n) {
    n = &_n;
  };
  ros::NodeHandle *n;
};
ROSNode *rn;

sensor_msgs::JointState js;

double currentjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double targetjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double zeropos[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree

geometry_msgs::PoseArray pp;
int state = 0;

void GetIK(geometry_msgs::Pose &_ps);
geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose);
 
void jointCallback(const sensor_msgs::JointState msg){
     ROS_INFO("%f,%f,%f,%f,%f,%f", msg.position[0],msg.position[1],msg.position[2],msg.position[3],msg.position[4],msg.position[5]);
     for(int i=0; i<6; i++) {
        currentjoint[i] = msg.position[i];
             
    }
}
void joyCallback(const sensor_msgs::Joy msg){
    ROS_INFO("joyCallback : %s", msg.header.frame_id.c_str());

    //ปุ่มอนาล็อคซ้าย ขยับซ้าย << หมายถึงปุ่มไม่ใช่หุ่น (ขยับฐาน)
    if(msg.axes[0] > 0.25 && state == 0){
        state = 1;
    }
     //ปุ่มอนาล็อคซ้าย ขยับขวา ขยับฐาน
    else if (msg.axes[0] < -0.25 && state == 0){
        state = 2;
    }
    //ปุ่มอนาล็อคซ้ายขยับขึ้น joint2 ขยับขึ้น
    else if (msg.axes[1] > 0.25 && state == 0){

        state = 3;    
    }
    //ปุ่มอนาล็อคซ้ายขยับลง joint2 ขยับลง
    else if (msg.axes[1] < -0.25 && state == 0){

        state = 4; 
    }
    //ปุ่มอนาล็อคขวาขยับขึ้น joint3 ขยับขึ้น
    else if  (msg.axes[4] > 0.25 && state == 0){

        state = 5;
    }
    //ปุ่มอนาล็อคขวาขยับลง joint3 ขยับลง
    else if (msg.axes[4] < -0.25 && state == 0){

        state = 6;
    }
    //ปุ่มอนาล็อคขวาขยับ ขวา joint5 ขยับขวา
    else if (msg.axes[3] > 0.25 && state == 0){

        state = 7;    
    }
    //ปุ่มอนาล็อคขวาขยับ ซ้าย joint 5 ขยับซ้าย
    else if (msg.axes[3] < -0.25 && state == 0){

        state = 8;
    }
    //ปุ่มลูกศรขึ้นjoint4 ขยับขึ้น
    else if (msg.axes[7] == 1 && state == 0){

        state = 9;    
    }
    //ปุ่มลูกศรลงjoint4ขยับลง
    else if (msg.axes[7] == -1 && state == 0){

        state = 10;    
    }
    //ปุ่มลูกศรซ้าย joint6 ขยับซ้าย
    else if (msg.axes[6] == 1 && state == 0){

        state = 11;
    }
    //ปุ่มลูกศรขวา joint6ขยับขวา
    else if (msg.axes[6] == -1 && state == 0){

        state = 12;
    }
    else if (msg.buttons[0] == 1 && state ==0){
        state = 13;
    }
    else{
        state = 0;
    }

}


int main(int argc, char **argv)
{
 
  
  
  ros::init(argc, argv, "testAuboAPI");
  ros::NodeHandle nn;
  rn= new ROSNode(nn);



  ros::Subscriber joy_sub = rn->n->subscribe("joy", 1000, joyCallback );   //subscribe joy
  ros::Subscriber jointState = rn->n->subscribe("/joint_states",100, jointCallback); //subscribe jointstate
 // ros::ServiceClient srv_fk = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
//  ros::ServiceClient srv_ik = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  


  AuboDriver robot_driver;
  bool ret = robot_driver.connectToRobotController();
  


  /** If connect to a real robot, then you need initialize the dynamics parameters　**/
  aubo_robot_namespace::ROBOT_SERVICE_STATE result;
  //tool parameters
  aubo_robot_namespace::ToolDynamicsParam toolDynamicsParam;
  memset(&toolDynamicsParam, 0, sizeof(toolDynamicsParam));

  robot_driver.robot_send_service_.rootServiceRobotStartup(toolDynamicsParam/**tool dynamics paramters**/,
                                             6        /*collision class*/,
                                             true     /* Is allowed to read robot pose*/,
                                             true,    /*default */
                                             1000,    /*default */
                                             result); /*initialize*/

  ros::Rate loop_rate(1000);
  while (ros::ok()){
        for(int i=0; i<6; i++) {
          targetjoint[i] = currentjoint[i];         
            } 
      if(ret && state == 1)
      {
        ROS_INFO("state 1");

 
        targetjoint[0] = currentjoint[0]-(1.0/180.0*M_PI);	

        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);
        
      }
      else if(ret && state == 2){
        ROS_INFO("state 2");

        targetjoint[0] = currentjoint[0]+(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 3)
      {
        ROS_INFO("state 3");

        targetjoint[1] = currentjoint[1]-(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);


      }
      else if(ret && state == 4)
      {
        ROS_INFO("state 4");

        targetjoint[1] = currentjoint[1]+(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 5)
      {
        ROS_INFO("state 5");

        targetjoint[2] = currentjoint[2]+(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 6)
      {
        ROS_INFO("state 6");

        targetjoint[2] = currentjoint[2]-(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 7)
      {
        ROS_INFO("state 7");

        targetjoint[4] = currentjoint[4]-(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 8)
      {
        ROS_INFO("state 8");

        targetjoint[4] = currentjoint[4]+(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 9)
      {
        ROS_INFO("state 9");

        targetjoint[3] = currentjoint[3]-(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 10)
      {
        ROS_INFO("state 10");

        targetjoint[3] = currentjoint[3]+(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 11)
      {
        ROS_INFO("state 11");

        targetjoint[5] = currentjoint[5]+(1.0/180.0*M_PI);
        robot_driver.robot_send_service_.robotServiceJointMove(targetjoint, true);

      }
      else if(ret && state == 12)
      {
        ROS_INFO("state 12");
        //std::vector<double> rpy = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //geometry_msgs::Pose cartesian = jsCartesian(js, rpy);
      
        for(int i = 0;i<6;i++){
          
          GetIK(pp.poses[i]);
        }
      }


   
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}
void GetIK(geometry_msgs::Pose &_ps){
      moveit_msgs::GetPositionIK msg;
      msg.request.ik_request.group_name = "wrist3_Link";
      msg.request.ik_request.pose_stamped.pose = _ps;
      msg.request.ik_request.attempts = 0;
      ros::ServiceClient srv_ik = rn->n->serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
      if(srv_ik.call(msg)){
          ROS_INFO("%f,%f,%f,%f,%f,%f",msg.response.solution.joint_state.position[0],msg.response.solution.joint_state.position[1],msg.response.solution.joint_state.position[2],msg.response.solution.joint_state.position[3],msg.response.solution.joint_state.position[4],msg.response.solution.joint_state.position[5]);
       
      }
      else{
        ROS_ERROR("Failed to call srv ");
      }
  

  }
geometry_msgs::Pose jsCartesian(const sensor_msgs::JointState &_js, std::vector<double> &_pose){
  moveit_msgs::GetPositionFK msg;
  msg.request.header.stamp = ros::Time::now();
  msg.request.fk_link_names = {"wrist3_Link"};
  msg.request.robot_state.joint_state = _js;
  ros::ServiceClient srv_fk = rn->n->serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
  if(srv_fk.call(msg)){
    _pose[0] = msg.response.pose_stamped[0].pose.position.x;
    _pose[1] = msg.response.pose_stamped[0].pose.position.y;
    _pose[2] = msg.response.pose_stamped[0].pose.position.z;  
    tf::Quaternion q_ori;
    tf::quaternionMsgToTF(msg.response.pose_stamped[0].pose.orientation , q_ori);
    tf::Matrix3x3 m(q_ori);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    _pose[3] = roll;
    _pose[4] = pitch;
    _pose[5] = yaw;
  }
  else{
    ROS_ERROR("Failed to call srv");

  }
  return msg.response.pose_stamped[0].pose;
}
