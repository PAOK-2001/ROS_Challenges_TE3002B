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
    bool hasFinished = false;
    ros::Subscriber systemFeedback = handler.subscribe("/path", 10, receive_path);
    ros::Publisher controllerOutput = handler.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher estimated_pose = handler.advertise<geometry_msgs::PoseStamped>("/estimated_pose", 10);

    float kpr, kpt, wheelbase, wheel_radius;
    ros::param::get("/rotational_constant", kpr);
    ros::param::get("/translational_constant", kpt);
    ros::param::get("/wheel_base", wheelbase);
    ros::param::get("/wheel_radius", wheel_radius);
    ros::Rate rate(nodeRate);
    geometry_msgs::Twist output;
    geometry_msgs::PoseStamped puzzlePose;
    float dt;
    float robot_x = 0;
    float robot_y = 0;
    float robot_orientation = 0;

    float w_max = 8;
    float v_max = (w_max*wheel_radius)*0.6;
    float angularV_max = 1;

    while(ros::ok){
        if(!hasFinished){
            for (auto coord : path.poses){
                float desired_x = coord.pose.position.x;
                float desired_y = coord.pose.position.y;
                float desired_angle;

                cout << "X Goal :" << desired_x << endl;
                cout << "Y Goal :" << desired_y << endl;
                cout << "Theta :" << desired_angle << endl;

                float distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));

                while (distance>0.1){
                    float t = ros::Time::now().toSec();
                    usleep(100000);
                    desired_angle = atan2((desired_y-robot_y),(desired_x -robot_x));
                    distance = sqrt(pow(desired_y-robot_y,2)+pow(desired_x-robot_x,2));
                
                    float angle_error = (robot_orientation - desired_angle);
                    if(angle_error > M_PI) angle_error = angle_error - 2*M_PI;
                    else if(angle_error < - M_PI)angle_error = angle_error + 2*M_PI;
                    
                    float angularVelocity = -kpr*angle_error;
                    if(angularVelocity > angularV_max) angularVelocity = angularV_max;
                    else if (angularVelocity < -angularV_max) angularVelocity = -angularV_max;
                    
                    float velocity = kpt*distance; 
                    velocity = v_max * tanh(velocity);
            
                    output.linear.x = velocity;
                    output.linear.y = 0;
                    output.linear.z = 0;

                    output.angular.x = 0;
                    output.angular.y = 0;
                    output.angular.z = angularVelocity;

                    dt = ros::Time::now().toSec() - t;
                    t = ros::Time::now().toSec();
                
                    robot_orientation += angularVelocity*dt;
                    robot_x += velocity*dt*cos(robot_orientation);
                    robot_y += velocity*dt*sin(robot_orientation);

                    puzzlePose.pose.position.x = robot_x;
                    puzzlePose.pose.position.y = robot_y;
                    puzzlePose.pose.orientation.z = robot_orientation;

                    controllerOutput.publish(output);
                    estimated_pose.publish(puzzlePose);
                }
            }
            ros::spinOnce();
            rate.sleep();
            hasFinished = robot_x || robot_y;
        }
        output.linear.x = 0;
        output.linear.y = 0;
        output.linear.z = 0;

        output.angular.x = 0;
        output.angular.y = 0;
        output.angular.z = 0;
        controllerOutput.publish(output);
    }

}