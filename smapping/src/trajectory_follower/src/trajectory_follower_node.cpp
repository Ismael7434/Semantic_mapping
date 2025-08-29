#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Point {
    double x;
    double y;
    double w;
};

std::map<std::string, std::vector<Point>> loadTrajectories(const std::string& filename) {
    std::map<std::string, std::vector<Point>> traj_map;
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& traj : config["trajectoires"]) {
        std::string name = traj["name"].as<std::string>();
        std::vector<Point> points;
        for (const auto& p : traj["points"]) {
            Point pt;
            pt.x = p["x"].as<double>();
            pt.y = p["y"].as<double>();
            pt.w = p["w"] ? p["w"].as<double>() : 1.0;
            points.push_back(pt);
        }
        traj_map[name] = points;
    }
    return traj_map;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "trajectory_follower_node");
    ros::NodeHandle nh;

    std::string path_file;
    nh.param<std::string>("trajectory_yaml_file", path_file, "config/trajectoires.yaml");

    ros::Publisher executed_path_pub = nh.advertise<nav_msgs::Path>("/path_executed", 10);
    tf::TransformListener listener;

    MoveBaseClient ac("move_base", true);
    while (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_INFO("Waiting for the move_base action server...");
    }

    ROS_INFO("Waiting for /path_ready...");
    ros::topic::waitForMessage<std_msgs::Empty>("/path_ready", nh);
    ROS_INFO("/path_ready received. Publishing preview...");

    std::map<std::string, std::vector<Point>> traj_map = loadTrajectories(path_file);

    // Publication de la trajectoire à suivre avant mouvement
    nav_msgs::Path preview_path;
    preview_path.header.frame_id = "map";
    preview_path.header.stamp = ros::Time::now();

    for (const auto& [traj_name, points] : traj_map) {
        for (const auto& pt : points) {
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(pt.w);
            preview_path.poses.push_back(pose);
        }
    }
    executed_path_pub.publish(preview_path);
    ROS_INFO("Published planned trajectory to /path_executed");

    // Exécution réelle
    std::ofstream duration_file("/home/ismael/smapping/src/trajectory_follower/trajectory_duration.txt");
    if (!duration_file.is_open()) {
        ROS_ERROR("Cannot open trajectory_duration.txt for writing");
        return 1;
    }

    for (const auto& [traj_name, points] : traj_map) {
        ROS_INFO("Executing trajectory: %s", traj_name.c_str());
        ros::Time start_time = ros::Time::now();

        for (const auto& pt : points) {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = pt.x;
            goal.target_pose.pose.position.y = pt.y;
            goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pt.w);

            ROS_INFO("Sending goal: (%.2f, %.2f)", pt.x, pt.y);
            ac.sendGoal(goal);
            ac.waitForResult();

            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("Goal reached.");
            } else {
                ROS_WARN("Goal failed.");
            }
        }

        ros::Duration duration = ros::Time::now() - start_time;
        duration_file << traj_name << ": " << duration.toSec() << " seconds" << std::endl;
    }

    duration_file.close();
    ROS_INFO("All trajectories completed. Durations saved.");

    return 0;
}
