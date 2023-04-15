#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <cmath>

using namespace std;

int nodeRate = 100;
nav_msgs::Path path;
void receive_path(const nav_msgs::Path &received_path){
    path = received_path;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "controller");
    ros::NodeHandle handler;
    ros::Subscriber systemFeedback = handler.subscribe("/path", 10, receive_path);
    ros::Publisher controllerOutput = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    float kpr, kpt, wheelbase;
    ros::param::get("/rotational_constant", kpr);
    ros::param::get("/rotational_constant", kpt);
    ros::param::get("/wheel_base", wheelbase);
    ros::Time t = ros::Time::now();
    ros::Rate rate(nodeRate);
    geometry_msgs::Twist output;
    float dt;
    float robot_x = 0;
    float robot_y = 0;
    float robot_orientation = 0;

    while(ros::ok){
        for (auto coord : path.poses){
            float desired_x = coord.pose.position.x;
            float desired_y = coord.pose.position.y;
            float desired_angle;
            float distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));

            while (distance>0.1){
                desired_angle = atan2((desired_y-robot_y),(desired_x -robot_x));
                distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
                
                float angle_error = (robot_orientation - desired_angle);
                if(angle_error > M_PI) angle_error = angle_error - 2*M_PI;
                else if(angle_error < -M_PI)angle_error = angle_error + 2*M_PI;
                
                
                float angularVelocity = -kpr*angle_error;
                // velocity = kpt*distance;
                float velocity = kpt*tanh(distance/0.1); //Sigmoid
                output.linear.x = velocity*(cos(robot_orientation));
                output.linear.y = velocity*(sin(robot_orientation));
                output.linear.z = 0;

                output.angular.x = 0;
                output.angular.y = 0;
                output.angular.z = angularVelocity;

                dt = ros::Time::now().toSec() - t.toSec();
                t = ros::Time::now();
                robot_x += velocity*cos(robot_orientation)*dt;
                robot_y += velocity*sin(robot_orientation)*dt;
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

}