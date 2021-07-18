//
// Created by karadalex on 3/4/21.
//

#include "kinematics/utils.h"

vector<geometry_msgs::Pose> fulcrumEffectTransformation(vector<geometry_msgs::Pose> taskPoints, double L) {
	vector<geometry_msgs::Pose> transformedPoints;

	for (int i = 0; i < taskPoints.size(); i++) {
		geometry_msgs::Pose pose = taskPoints.at(i);
		double px = pose.position.x;
		double py = pose.position.y;
		double pz = pose.position.z;
		double r = sqrt(px*px + py*py + pz*pz);
		double th = atan2(sqrt(px*px + py*py), pz);
		double phi = atan2(py, px);

		tf2::Vector3 position = tf2::Vector3(r*sin(th)*cos(phi), r*sin(th)*sin(phi), r*cos(th));
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
		tf2::Transform transformation = Td * T.inverse() * tf2::Transform(poseOrientation, posePosition);

		geometry_msgs::Pose fulcrumPose;
		tf2::toMsg(transformation, fulcrumPose);
		transformedPoints.push_back(fulcrumPose);
	}

	return transformedPoints;
}