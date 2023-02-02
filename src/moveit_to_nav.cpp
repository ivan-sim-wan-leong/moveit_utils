#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "moveit_msgs/DisplayTrajectory.h"
#include <tf2/LinearMath/Quaternion.h>


using namespace std;

class MoveitToNav{
public:

    MoveitToNav(ros::NodeHandle* n){
        n_ = n;
        nav_pub_ = n_->advertise<nav_msgs::Path>("moveit_to_nav/path", 1, true);
        moveit_sub_ = n_->subscribe("/move_group/display_planned_path", 1, &MoveitToNav::moveit_path_cb, this);
    }

protected:

    void moveit_path_cb(const moveit_msgs::DisplayTrajectory::Ptr& msg){
        nav_msgs::Path nav_path;

        trajectory_msgs::JointTrajectory* traj = &msg->trajectory[0].joint_trajectory;

        nav_path.header = traj->header;

        for(int i=0; i<traj->points.size(); i++){
            // geometry_msgs::Quaternion q;
            geometry_msgs::PoseStamped pose;
            rpy_to_quaternion(0., 0., traj->points[i].positions[2], pose.pose.orientation);

            pose.pose.position.x = traj->points[i].positions[0];
            pose.pose.position.y = traj->points[i].positions[1];
            pose.pose.position.z = 0.;

            pose.header.stamp = traj->header.stamp + traj->points[i].time_from_start;
            pose.header.frame_id = traj->header.frame_id;

            nav_path.poses.push_back(pose);
        }


        nav_pub_.publish(nav_path);
        
    }

    void rpy_to_quaternion(double r, double p, double y, geometry_msgs::Quaternion& quaternion){
        tf2::Quaternion q;

        q.setRPY(r, p, y);
        q.normalize();

        quaternion.x = q.getX();
        quaternion.y = q.getY();
        quaternion.z = q.getZ();
        quaternion.w = q.getW();

    }

    ros::NodeHandle* n_;
    ros::Publisher nav_pub_;
    ros::Subscriber moveit_sub_;
};

int main(int argc, char** argv){

    ros::init(argc, argv, "moveit_to_nav");
    ros::NodeHandle n;
    ros::Rate rate(10);

    MoveitToNav moveit_to_nav(&n);

    ROS_INFO("Running moveit_to_nav_node");

    while(ros::ok()){


        rate.sleep();
        ros::spinOnce();
    }


    return 0;
}