#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

#define RUN_PERIOD_DEFAULT 0.1
#define NAME_OF_THIS_NODE "set_point_node"

//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber velSub;
    ros::Publisher left_rpmPub;
    ros::Publisher right_rpmPub;
    double wheel_diameter,track_width;
    
    //bool position, orientation, gotPose;
    std_msgs::Float64 left_rpm_msg;
    std_msgs::Float64 right_rpm_msg;
    
    void vel_msg_Callback(const geometry_msgs::Twist& cmd_vel_msg);
public:    
    bool Prepare();
    void RunContinuously();
    void Shutdown();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

bool ROSnode::Prepare() {
    
    velSub = Handle.subscribe("/vel_topic", 10, &ROSnode::vel_msg_Callback, this);
    left_rpmPub = Handle.advertise<std_msgs::Float64>("/left_wheel/rpm_cmd_setPoint", 10);
    right_rpmPub = Handle.advertise<std_msgs::Float64>("/right_wheel/rpm_cmd_setPoint", 10);
    
    
    
    // load parameter from launch file
    if(!Handle.getParam(ros::this_node::getName()+"/wheel_diameter",wheel_diameter)) return false;
    if(!Handle.getParam(ros::this_node::getName()+"/track_width", track_width)) return false;
    
    // print information 
    std::cout<<"Node "    <<ros::this_node::getName().c_str() <<"ready to run."<< std::endl;
    std::cout<<"Parameter"<<std::endl;
    std::cout<<"wheel diameter: " << wheel_diameter <<"(m)" <<std::endl;
    std::cout<<"track width: "    << track_width    <<"(m)" <<std::endl;
    return true;
}


void ROSnode::RunContinuously() {
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());
  ros::spin();
}

void ROSnode::Shutdown() {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

void ROSnode::vel_msg_Callback(const geometry_msgs::Twist& msg) {
    
    //declaration
    double rpm_req1;
    double rpm_req2;

    // convert linear and angular velocity to rmp.
    double linear_vel_x  = msg.linear.x /5;
    double angular_vel_z = msg.angular.z /2;
    
    if (angular_vel_z == 0) {     // go straight
        // convert m/s to rpm
        ROS_INFO("Arrivato comando di velocità.");
        rpm_req1 = linear_vel_x*60/(M_PI*wheel_diameter);
	// rpm_req1 = rpm_req2; una volta sistemato il problema di alimentazione congiunta motore/encoder        
	rpm_req2 = - rpm_req1;
    }
    else if (linear_vel_x == 0) {
        // convert rad/s to rpm
	
        rpm_req2 = - angular_vel_z*track_width*60/(wheel_diameter*M_PI*2);
	// rpm_req1 = - rpm_req2; una volta sistemato il problema di alimentazione congiunta motore/encoder
        rpm_req1 = rpm_req2;
    }
    else {
       
        rpm_req1 = linear_vel_x*60/(M_PI*wheel_diameter)-angular_vel_z*track_width*60/(wheel_diameter*M_PI*2);
	// togliere il meno una volta sistemate le alimentazioni
        rpm_req2 = -(linear_vel_x*60/(M_PI*wheel_diameter)+angular_vel_z*track_width*60/(wheel_diameter*M_PI*2));
    }
	
    
    left_rpm_msg.data = rpm_req1;
    right_rpm_msg.data = rpm_req2;
    
    left_rpmPub.publish(left_rpm_msg);
    right_rpmPub.publish(right_rpm_msg);
    
 }

//-----------------------------------------------------------------
//-----------------------------------------------------------------

int main(int argc, char **argv) {
  ros::init(argc, argv, NAME_OF_THIS_NODE);
  ROSnode mNode;
  
  mNode.Prepare();
  mNode.RunContinuously();
  mNode.Shutdown();
  
  return (0);
}
