#include <stdio.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

#define NAME_OF_THIS_NODE "odom_node"
#define RATE 1.0
//-----------------------------------------------------------------
//-----------------------------------------------------------------

class ROSnode {
private: 
    ros::NodeHandle Handle;
    ros::Subscriber right_plant_state_sub;
    ros::Subscriber left_plant_state_sub;
    ros::Publisher odom_pub;
    tf::TransformBroadcaster odom_broadcaster;

    double wheel_diameter,track_width;
    // variabili di storage per i plant_state
    double right_ps, left_ps;
    // velocità lineari e angolare del robot, calcolate attraverso gli RPM (plant state)
    double vel, omega;
    // parametri di posizione e orientamento e velocità:
    double x, y, yaw, vel_x, vel_y, z_ang;

    // funzioni di callback per i subscriber dei plant_state dalle due ruote.
    void ps_right_msg_Callback(const std_msgs::Float64& ps_right_msg);
    void ps_left_msg_Callback(const std_msgs::Float64& ps_left_msg);
    // la funzione che calcola le velocità partendo dai plant_states
    void computeVelocities();

public:    
    bool Prepare();
    void RunContinuously();
    void Shutdown();
};

//-----------------------------------------------------------------
//-----------------------------------------------------------------

bool ROSnode::Prepare() {
    
    right_plant_state_sub = Handle.subscribe("/right_wheel/rpm_plant_state_from_arduino", 10, &ROSnode::ps_right_msg_Callback, this);
    left_plant_state_sub = Handle.subscribe("/left_wheel/rpm_plant_state_from_arduino", 10, &ROSnode::ps_left_msg_Callback, this);
    odom_pub = Handle.advertise<nav_msgs::Odometry>("odom", 50);
    
    // load parameter from launch file
    if(!Handle.getParam("/robot/wheel_diameter",wheel_diameter)) return false;
    if(!Handle.getParam("/robot/track_width", track_width)) return false;
    
    // print information 
    std::cout<<"Node "    <<ros::this_node::getName().c_str() <<"ready to run."<< std::endl;
    std::cout<<"Parameter"<<std::endl;
    std::cout<<"wheel diameter: " << wheel_diameter <<"(m)" <<std::endl;
    std::cout<<"track width: "    << track_width    <<"(m)" <<std::endl;
    return true;
}


void ROSnode::RunContinuously() {
  ROS_INFO("Node %s running continuously.", ros::this_node::getName().c_str());

  double x_pos = 0.0;
  double y_pos = 0.0;
  double yaw = 0.0;

  double x_vel = 0.1;
  double y_vel = -0.1;
  double z_angular = 0.1;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(RATE);
  while(Handle.ok()){

    
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();
    
    computeVelocities();
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();

    vel_x = vel*cos(yaw);
    vel_y = vel*sin(yaw);
    z_ang = omega;

    double delta_x = vel_x* dt;
    double delta_y = vel_y* dt;
//    double delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt;
//    double delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt;
    double delta_yaw = omega * dt;
    
    x += delta_x;
    y += delta_y;
    yaw += delta_yaw;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = omega;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}

void ROSnode::Shutdown() {
  ROS_INFO("Node %s shutting down.", ros::this_node::getName().c_str());
}

// le callback settano i parametri di storage per i plant_state
void ROSnode::ps_right_msg_Callback(const std_msgs::Float64& msg) {
    right_ps = msg.data;
    std::cout << "right_ps: " << right_ps << "\n";  
}
void ROSnode::ps_left_msg_Callback(const std_msgs::Float64& msg) {
    left_ps = msg.data;
    std::cout << "left_ps: " << left_ps << "\n";  
}

void ROSnode::computeVelocities() {
    double left_wheel_rads = left_ps/60*2*M_PI;
    double right_wheel_rads = -right_ps/60*2*M_PI;

    double left_center_vel = left_wheel_rads*wheel_diameter/2;
    double right_center_vel = right_wheel_rads*wheel_diameter/2;

    // la velocità lineare del robot è la media delle due velocità dei centri delle ruote
    vel = (left_center_vel + right_center_vel) /2;
    // la velocità angolare del robot (v_r - v_l) / L 
    omega = (right_center_vel - left_center_vel) / track_width; 
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
