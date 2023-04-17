#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

using namespace std;

int nodeRate = 100;
vector<int> path;
vector<int> prevPath;

nav_msgs::Path generate_path(vector <int> path) {
    nav_msgs::Path generatedPath;
    for(int coord = 0; coord < path.size() - 2; coord+=2){
        geometry_msgs::PoseStamped currentCoord;
        currentCoord.pose.position.x = path[coord];
        currentCoord.pose.position.y = path[coord+1];
        generatedPath.poses.push_back(currentCoord);
    }
    return generatedPath;
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "path_generator");
    ros::NodeHandle handler;
    ros::Publisher pathPublisher = handler.advertise<nav_msgs::Path>("/path",10);
    ros::Rate rate(nodeRate);
    while (ros::ok()) {
        ros::param::get("/desired_path", path);
        pathPublisher.publish(generate_path(path));
        rate.sleep();
    }
    return 0;

}