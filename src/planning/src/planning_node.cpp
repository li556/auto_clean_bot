#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <dynamic_reconfigure/server.h>
#include "artificial_potential_field_planner/APFPlannerConfig.h"
#include "navigation/global_planner/APFPlanner.h"
#include "navigation/types/Vector2D.h"
#include "custom_msgs/LocalizationInfo.h"
#include "custom_msgs/Obstacles.h"
#include "custom_msgs/Lanes.h"
#include "custom_msgs/ADCTrajectory.h"
#include "custom_msgs/LaneInfo.h"
#include "custom_msgs/TrajectoryPoint.h"

ros::NodeHandlePtr nh;
navigation::global_planner::APFPlanner apf_planner;
double forcemap_width = 10, forcemap_height = 10;

void callback(artificial_potential_field_planner::APFPlannerConfig &config, uint32_t level)
{
    forcemap_width = config.forcemap_width;
    forcemap_height = config.forcemap_height;
    apf_planner.setParameters(config.attraction_gain, config.repulsion_gain, config.radius);
}

custom_msgs::ADCTrajectory convert_Vector2D_arr_to_trajectory(std::vector<navigation::types::Vector2D> vectList) {
    custom_msgs::ADCTrajectory trajectory;
    for(const auto& v: vectList) {
        custom_msgs::TrajectoryPoint point;
        point.position.x = v.x;
        point.position.y = v.y;
        point.position.z = 0;
        trajectory.points.push_back(point);
    }
    return trajectory;
}

void obstaclesCallback(const custom_msgs::Obstacles::ConstPtr& obstacles) {
    std::vector<navigation::types::Vector2D> obstacle_positions;
    for (const auto& obs : obstacles->obstacles) {
        obstacle_positions.emplace_back(obs.position.x, obs.position.y);
    }
    apf_planner.setObstacles(obstacle_positions);
}

void lanesCallback(const custom_msgs::Lanes::ConstPtr& lanes) {
    std::vector<navigation::types::Vector2D> lane_waypoints;
    for (const auto& waypoint : lanes->left_lane.waypoints) {
        lane_waypoints.emplace_back(waypoint.x, waypoint.y);
    }
    apf_planner.setLanes(lane_waypoints);
}

visualization_msgs::Marker createForceArrowMarker(Vector2D& pose, Vector2D& force) {
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "map";
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.action = visualization_msgs::Marker::ADD;
    arrow.pose.position.x = pose.x;
    arrow.pose.position.y = pose.y;
    arrow.pose.position.z = 1;
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, atan2(force.getUnitVector().y, force.getUnitVector().x));
    arrow.pose.orientation.x = myQuaternion.x();
    arrow.pose.orientation.y = myQuaternion.y();
    arrow.pose.orientation.z = myQuaternion.z();
    arrow.pose.orientation.w = myQuaternion.w();
    arrow.scale.x = 0.5;
    arrow.scale.y = 0.05;
    arrow.scale.z = 0.05;
    arrow.color.g = 1.0f;
    arrow.color.a = 1.0;
    return arrow;
}

ros::Publisher marker_pub;
void drawForceMap(std::vector<std::pair<Vector2D , Vector2D>>& poseForcePairList) {
    visualization_msgs::MarkerArray markerArray;
    int count = 1;
    for (auto& poseForcePair : poseForcePairList) {
        visualization_msgs::Marker force_marker = createForceArrowMarker(poseForcePair.first, poseForcePair.second);
        force_marker.id = count++;
        markerArray.markers.push_back(force_marker);
    }
    marker_pub.publish(markerArray);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "apf_planner");
    nh = boost::make_shared<ros::NodeHandle>();

    ros::Publisher trajectory_publisher = nh->advertise<custom_msgs::ADCTrajectory>("/trajectory", 1000);
    marker_pub = nh->advertise<visualization_msgs::MarkerArray>("/arrows", 100000);

    ros::Subscriber sub_obstacles = nh->subscribe<custom_msgs::Obstacles>("/obstacles", 10, obstaclesCallback);
    ros::Subscriber sub_lanes = nh->subscribe<custom_msgs::Lanes>("/lanes", 10, lanesCallback);
    ros::Subscriber sub_localization = nh->subscribe<custom_msgs::LocalizationInfo>("/localization_info", 10, 
                                    [&](const custom_msgs::LocalizationInfo::ConstPtr& locInfo) {
                                        // 使用东北天坐标系中的坐标位置
                                        navigation::types::Vector2D currentPosition(locInfo->north, locInfo->east);
                                        apf_planner.setCurrentPose(currentPosition);
                                        // 规划路径并转换为ADCTrajectory格式
                                        custom_msgs::ADCTrajectory trajectory = convert_Vector2D_arr_to_trajectory(apf_planner.plan());
                                        trajectory.header.frame_id = "map";
                                        trajectory_publisher.publish(trajectory);
                                        
                                        // 绘制力场图
                                        std::vector<std::pair<Vector2D, Vector2D>> forces = apf_planner.getForceMap(forcemap_height, forcemap_width);
                                        drawForceMap(forces);
                                    });

    dynamic_reconfigure::Server<artificial_potential_field_planner::APFPlannerConfig> server;
    dynamic_reconfigure::Server<artificial_potential_field_planner::APFPlannerConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
}


