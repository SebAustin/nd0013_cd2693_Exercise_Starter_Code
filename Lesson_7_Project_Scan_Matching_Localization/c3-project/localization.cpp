#include "localization.h"

#include <pcl/filters/voxel_grid.h>

namespace {
// The car cannot plausibly move further than this between two Scans at
// medium speed; a larger step means the previous match went wrong.
constexpr double kMaxPlausibleStepMetres = 3.0;
}

PointCloudT::Ptr voxelFilter(const PointCloudT::Ptr& cloud, float leafSize) {
	// EXPERIMENT: drop road-surface returns (lidar is 1.8 m up) so ground rings can't bias NDT.
	PointCloudT::Ptr aboveGround(new PointCloudT);
	for (const auto& p : cloud->points) {
		if (p.z > -1.4f) aboveGround->points.push_back(p);
	}
	aboveGround->width = aboveGround->points.size();
	aboveGround->height = 1;

	PointCloudT::Ptr filtered(new PointCloudT);
	pcl::VoxelGrid<PointT> grid;
	grid.setInputCloud(aboveGround);
	grid.setLeafSize(leafSize, leafSize, leafSize);
	grid.filter(*filtered);
	return filtered;
}

Eigen::Matrix4d poseToMatrix(const Pose& pose) {
	return transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll,
	                   pose.position.x, pose.position.y, pose.position.z);
}

Eigen::Matrix4d predictNextTransform(const Eigen::Matrix4d& previous, const Eigen::Matrix4d& current) {
	// Motion expressed in the car's frame, so it stays valid while turning.
	const Eigen::Matrix4d motion = previous.inverse() * current;
	const double step = motion.block<3, 1>(0, 3).norm();
	if (step > kMaxPlausibleStepMetres) {
		return current;
	}
	return current * motion;
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
