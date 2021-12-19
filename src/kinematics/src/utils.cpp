//
// Created by karadalex on 3/4/21.
//

#include <tf/LinearMath/Matrix3x3.h>
#include "kinematics/utils.h"

vector<geometry_msgs::Pose> fulcrumEffectTransformation(vector<geometry_msgs::Pose> taskPoints, double L, Eigen::Matrix4d left_mat, Eigen::Matrix4d right_mat) {
	vector<geometry_msgs::Pose> transformedPoints;

	// TODO: Improve following code to reduce calculations (optimization)
	geometry_msgs::Pose left = matrixTransformToQuaternionPose(left_mat);
	tf2::Matrix3x3 left_orientation;
	auto lq = left.orientation;
	left_orientation.setRotation(tf2::Quaternion(lq.x, lq.y, lq.z, lq.w));
	auto lp = left.position;
	tf2::Vector3 left_position = tf2::Vector3(lp.x, lp.y, lp.z);
	tf2::Transform left_T = tf2::Transform(left_orientation, left_position);

	geometry_msgs::Pose right = matrixTransformToQuaternionPose(right_mat);
	tf2::Matrix3x3 right_orientation;
	auto rq = right.orientation;
	right_orientation.setRotation(tf2::Quaternion(rq.x, rq.y, rq.z, rq.w));
	auto rp = right.position;
	tf2::Vector3 right_position = tf2::Vector3(rp.x, rp.y, rp.z);
	tf2::Transform right_T = tf2::Transform(right_orientation, right_position);

	for (geometry_msgs::Pose global_pose : taskPoints) {
		auto gp = global_pose.position;
		tf2::Vector3 globalPosePosition = tf2::Vector3(gp.x, gp.y, gp.z);
		auto gq = global_pose.orientation;
		tf2::Quaternion globalPoseOrientation = tf2::Quaternion(gq.x, gq.y, gq.z, gq.w);
		tf2::Transform pose_tf = left_T.inverse() * tf2::Transform(globalPoseOrientation, globalPosePosition) * right_T.inverse();
		geometry_msgs::Pose pose;
		tf2::toMsg(pose_tf, pose);

		double px = pose.position.x;
		double py = pose.position.y;
		double pz = pose.position.z;
		double r = sqrt(px*px + py*py + pz*pz);
		double th = atan2(sqrt(px*px + py*py), pz);
		double phi = atan2(py, px);

		tf2::Vector3 position = tf2::Vector3(px, py, pz);
		double xx = cos(th)*cos(phi); double xy = cos(th)*sin(phi); double xz = -sin(th);
		double yx = -sin(phi); double yy = cos(phi); double yz = 0;
		double zx = -sin(th)*cos(phi); double zy = -sin(th)*sin(phi); double zz = cos(th);
		tf2::Matrix3x3 orientation = tf2::Matrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz);

		tf2::Transform T = tf2::Transform(orientation, position);
		tf2::Vector3 dx = (r-L)/r * position;
		tf2::Transform Td = tf2::Transform(tf2::Matrix3x3(), dx);

		tf2::Vector3 posePosition = tf2::Vector3(px, py, pz);
		auto q = pose.orientation;
		tf2::Quaternion poseOrientation = tf2::Quaternion(q.x, q.y, q.z, q.w);
		tf2::Transform transformation = left_T * Td * T.inverse() * tf2::Transform(poseOrientation, posePosition) * right_T;

		geometry_msgs::Pose fulcrumPose;
		tf2::toMsg(transformation, fulcrumPose);
		transformedPoints.push_back(fulcrumPose);
	}

	return transformedPoints;
}

geometry_msgs::Pose matrixTransformToQuaternionPose(Eigen::Matrix4d T) {
	// Generate pose in Quaternion Form
	geometry_msgs::Pose pose;
	tf2::Quaternion quaternion;

	pose.position.x = T(0, 3);
	pose.position.y = T(1, 3);
	pose.position.z = T(2, 3);

	float roll = atan2(T(1, 0), T(0, 0));
	float pitch = atan2(-T(2, 0), sqrt(T(0, 0) * T(0, 0) + T(1, 0) * T(1, 0)));
	float yaw = atan2(T(2, 1), T(2, 2));

	quaternion.setRPY(roll, pitch, yaw);
	pose.orientation.w = quaternion.getW();
	pose.orientation.x = quaternion.getX();
	pose.orientation.y = quaternion.getY();
	pose.orientation.z = quaternion.getZ();

	return pose;
}

Eigen::Matrix4d quaternionPoseToMatrixTransform(geometry_msgs::Pose pose) {
	return Eigen::Matrix4d::Identity();
}

