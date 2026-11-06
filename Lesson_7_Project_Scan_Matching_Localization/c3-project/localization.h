#ifndef LOCALIZATION_H
#define LOCALIZATION_H

#include "helper.h"
#include <pcl/registration/ndt.h>

// Tuning for the NDT scan matcher.
struct NdtConfig {
	double resolution = 1.0;      // NDT voxel (cell) size in metres
	double stepSize = 0.1;        // max More-Thuente line-search step
	double epsilon = 1e-4;        // convergence threshold on transform change
	int maxIterations = 60;       // hard cap per Scan to bound latency
};

// Downsamples a cloud with a voxel grid. Returns a new cloud; the input is untouched.
PointCloudT::Ptr voxelFilter(const PointCloudT::Ptr& cloud, float leafSize);

// Converts a Pose (yaw/pitch/roll + position) into a homogeneous 4x4 transform.
Eigen::Matrix4d poseToMatrix(const Pose& pose);

// Constant-velocity motion model: applies the last inter-Scan motion once more
// to produce the Initial guess for the next Scan. Falls back to "no motion" if
// the last step is implausibly large (a sign the previous match diverged).
Eigen::Matrix4d predictNextTransform(const Eigen::Matrix4d& previous, const Eigen::Matrix4d& current);

// Advances `current` by `fraction` times the motion from `previous` to `current`
// (translation and rotation both scaled). fraction = 1 repeats the last motion once;
// fraction = latency / scanInterval moves a scan-time Pose estimate up to "now".
Eigen::Matrix4d extrapolateMotion(const Eigen::Matrix4d& previous, const Eigen::Matrix4d& current, double fraction);

// Aligns lidar Scans to the Map with PCL's Normal Distributions Transform.
// The Map's NDT grid is built once at construction, which is the costly part.
class NdtLocalizer {
public:
	NdtLocalizer(const PointCloudT::Ptr& map, const NdtConfig& config);

	// Returns the Scan-to-Map transform (i.e. the Pose estimate as a matrix),
	// starting the search from `initialGuess`.
	Eigen::Matrix4d align(const PointCloudT::Ptr& scan, const Eigen::Matrix4d& initialGuess);

	bool hasConverged() const { return ndt_.hasConverged(); }
	int iterations() const { return ndt_.getFinalNumIteration(); }
	double fitnessScore() { return ndt_.getFitnessScore(); }

private:
	pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
};

#endif // LOCALIZATION_H
