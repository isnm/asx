

#include "aubo_driver/aubo_driver.h"
#include "aubo_driver.cpp"
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include "geometry_msgs/PoseArray.h"
#include <aubo_msgs/JointPos.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include "tf/tf.h"

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include <stdio.h>
#include <sstream>
#include <fstream>

using namespace aubo_driver;


double currentjoint[6] = {90.0/180*M_PI,45.0/180*M_PI,45.0/180*M_PI,90.0/180*M_PI,90.0/180*M_PI,90.0/180*M_PI}; //degree
double targetjoint[6] = {0.0,0.0,0.0,0.0,0.0,0.0}; //degree
double pose[6] ={0.0};
double roll,pitch,yaw;
int state = 0;

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
    //ปุ่มลูกศรขวา joint6ขยับขวา
    else if (msg.axes[6] == 1 && state == 0){

        state = 11;
    }
    //ปุ่มลูกศรซ้าย joint6 ขยับซ้าย
    else if (msg.axes[6] == -1 && state == 0){

        state = 12;
    }

}


int main(int argc, char **argv)
{
 
	  
	  
	ros::init(argc, argv, "testAuboAPI");

	ros::NodeHandle n;

	ros::Subscriber joy_sub = n.subscribe("joy", 1000, joyCallback );   //subscribe joy
	ros::Subscriber jointState = n.subscribe("/joint_states",100, jointCallback); //subscribe jointstate

	AuboDriver robot_driver;
	bool ret = robot_driver.connectToRobotController();



	moveit_msgs::GetPositionIK msg2;
	
	ros::ServiceClient srv_fk = n.serviceClient<moveit_msgs::GetPositionFK>("/compute_fk");
	ros::ServiceClient srv_ik = n.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");

	  
	  

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
		//robot_driver.robot_send_service_.robotServiceJointMove(currentjoint, true);
		ROS_INFO("%f,%f,%f,%f,%f,%f",currentjoint[0],currentjoint[1],currentjoint[2],currentjoint[3],currentjoint[4],currentjoint[5]);
		moveit_msgs::GetPositionFK msg1;
		msg1.request.header.stamp = ros::Time::now();
		msg1.request.fk_link_names = {"wrist3_Link"};
		for(int i=0;i<6;i++){
		msg1.request.robot_state.joint_state.position[i] = currentjoint[i];
		}
	
		if(srv_fk.call(msg1))
		{
			pose[0] = msg1.response.pose_stamped[0].pose.position.x;
			pose[1] = msg1.response.pose_stamped[0].pose.position.y;
			pose[2] = msg1.response.pose_stamped[0].pose.position.z;
			tf::Quaternion q_ori;
			tf::quaternionMsgToTF(msg1.response.pose_stamped[0].pose.orientation , q_ori);
			tf::Matrix3x3 m(q_ori);
			m.getRPY(roll, pitch, yaw);
			pose[3] = roll;
			pose[4] = pitch;
			pose[5] = yaw;
		

		}
		ROS_INFO("%f,%f,%f,%f,%f,%f",pose[0],pose[1],pose[2],pose[3],pose[4],pose[5]);
		ros::spinOnce();
		loop_rate.sleep();
	
	
	}
	return 0;
}
