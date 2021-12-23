#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "Eigen/Eigen"
#include <geometry_msgs/Vector3.h>
#include <cmath>
#include <std_msgs/Float64.h>


#include <iostream>
#include <sstream>

ros::Publisher* twist_pub_glob = NULL; 
ros::Publisher* error_pub_glob = NULL;

static double smallestDeltaAngle(const double& x, const double& y)
{
  // From https://stackoverflow.com/questions/1878907/the-smallest-difference-between-2-angles
  return atan2(sin(x - y), cos(x - y));
}

void callback(const geometry_msgs::Pose::ConstPtr& msg){
    ROS_INFO_STREAM("Recieved: " << msg);
    Eigen::VectorXd pos(3,1); //Insert the angle here too
    pos(0) = msg->position.x;
    pos(1) = msg->position.y;
    pos(2) = msg->position.z;
    
    Eigen::Quaterniond quat;
    quat.x() = msg->orientation.x;
    quat.y() = msg->orientation.y;
    quat.z() = msg->orientation.z;
    quat.w() = msg->orientation.w;
    
    //convert from quaternion to rotation matrix
    Eigen::Matrix3d DCM = quat.normalized().toRotationMatrix().transpose();

    //get rotation angle around x-axis
    double theta = atan2(DCM(1,0), DCM(0,0));
    
    
    //calculate controller variables
    double rho = sqrt(pow(pos(0), 2)+pow(pos(1), 2));
    double gamma = smallestDeltaAngle(atan2(pos(1), pos(0)),theta+ M_PI);
    double delta = gamma + theta;
    
    
    // Gains
    double k_1 = 0.5;
    double k_2 = 2.5;
    double k_3 = 2.5;
    
    // Control law
    double v = k_1*rho*cos(gamma);
    double omega = k_2*gamma + k_1*((sin(gamma)*cos(gamma))/gamma)*(gamma+k_3*delta);
    
    
    //Publish
    std_msgs::Float64 error_msg;
    geometry_msgs::Twist twist; 
    
    
    error_msg.data = rho;
   
    //stop if close enough
    if(rho <= 0.12 )
    {
    	v = 0;
    	omega = 0;
    }
  

    
    //Limit linear velocity
    if (v > 0.26) 
    {
    	v = 0.26;
    }
    
    
     
    twist.linear.x = v;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    
    //Limit angular velocity
    if (omega > 1.82)
    {
    	omega = 1.82;
    }
    
    twist.angular.z = omega;
    
    error_pub_glob->publish(error_msg);
    twist_pub_glob->publish(twist);
}

int main (int argc, char **argv){
    ros::init(argc, argv, "sample");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber sub = n.subscribe("/pose", 1000, callback);
    ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000); //waffle2/cmd_vel
    twist_pub_glob = &pub_twist;
    
    ros::Publisher pub_error = n.advertise<std_msgs::Float64>("/error", 1000);
    error_pub_glob = &pub_error;
    
    ros::spin();
    
    return 0;
}
