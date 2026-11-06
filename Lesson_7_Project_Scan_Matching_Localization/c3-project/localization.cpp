#include "localization.h"

#include <algorithm>

#include <pcl/filters/voxel_grid.h>

namespace {
// Faster than this between two Scans means the previous match went wrong.
constexpr double kMaxPlausibleSpeed = 25.0;      // m/s
// Intervals shorter than this give a velocity dominated by matching noise.
constexpr double kMinVelocityInterval = 0.02;    // s
// Never extrapolate further than this, however long a stall lasts.
constexpr double kMaxPredictionHorizon = 1.0;    // s

// Scales a rigid motion by `fraction` (translation linearly, rotation by angle).
Eigen::Matrix4d scaleMotion(const Eigen::Matrix4d& motion, double fraction) {
	Eigen::AngleAxisd rotation(Eigen::Matrix3d(motion.block<3, 3>(0, 0)));
	rotation.angle() *= fraction;
	Eigen::Matrix4d scaled = Eigen::Matrix4d::Identity();
	scaled.block<3, 3>(0, 0) = rotation.toRotationMatrix();
	scaled.block<3, 1>(0, 3) = motion.block<3, 1>(0, 3) * fraction;
	return scaled;
}
}  // namespace

PointCloudT::Ptr voxelFilter(const PointCloudT::Ptr& cloud, float leafSize) {
	PointCloudT::Ptr filtered(new PointCloudT);
	pcl::VoxelGrid<PointT> grid;
	grid.setInputCloud(cloud);
	grid.setLeafSize(leafSize, leafSize, leafSize);
	grid.filter(*filtered);
	return filtered;
}

Eigen::Matrix4d poseToMatrix(const Pose& pose) {
	return transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
	                   pose.position.x, pose.position.y, pose.position.z);
}

Eigen::Matrix4d predictPose(const Eigen::Matrix4d& previous, const Eigen::Matrix4d& current,
                            double interval, double horizon) {
	if (interval < kMinVelocityInterval || horizon <= 0.0) {
		return current;
	}
	// Motion expressed in the car's frame, so it stays valid while turning.
	const Eigen::Matrix4d motion = previous.inverse() * current;
	if (motion.block<3, 1>(0, 3).norm() / interval > kMaxPlausibleSpeed) {
		return current;
	}
	return current * scaleMotion(motion, std::min(horizon, kMaxPredictionHorizon) / interval);
}

NdtLocalizer::NdtLocalizer(const PointCloudT::Ptr& map, const NdtConfig& config) {
	ndt_.setResolution(config.resolution);
	ndt_.setStepSize(config.stepSize);
	ndt_.setTransformationEpsilon(config.epsilon);
	ndt_.setMaximumIterations(config.maxIterations);
	ndt_.setInputTarget(map);
}

Eigen::Matrix4d NdtLocalizer::align(const PointCloudT::Ptr& scan, const Eigen::Matrix4d& initialGuess) {
	ndt_.setInputSource(scan);
	PointCloudT aligned;  // required by the PCL API; we transform the Scan ourselves
	ndt_.align(aligned, initialGuess.cast<float>());
	return ndt_.getFinalTransformation().cast<double>();
}
