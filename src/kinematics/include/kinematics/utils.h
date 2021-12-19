//
// Created by karadalex on 3/4/21.
//

#ifndef SURGERY_ROBOTICS_KUKA_BARRETT_UTILS_H
#define SURGERY_ROBOTICS_KUKA_BARRETT_UTILS_H

#include <Eigen/Dense>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

/**
 * Transform trajectory points inside task space to waypoints, by applying the fulcrum effect
 * @param taskPoints, list of taskpoints expressed as Pose
 * @param L, the length of the surgical tool or the total amount of insertion inside the task space
 * @return
 */
vector<geometry_msgs::Pose> fulcrumEffectTransformation(vector<geometry_msgs::Pose> taskPoints, double L, Eigen::Matrix4d left_mat = Eigen::Matrix4d::Identity(), Eigen::Matrix4d right_mat = Eigen::Matrix4d::Identity());

/**
 * Convert an Eigen::Matrix4d to a geometry_msgs::Pose pose with position vector and quaternion rotation
 * @param T
 * @return geometry_msgs::Pose pose
 */
geometry_msgs::Pose matrixTransformToQuaternionPose(Eigen::Matrix4d T);

#endif //SURGERY_ROBOTICS_KUKA_BARRETT_UTILS_H
