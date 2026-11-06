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

// Constant-velocity motion model in *time*: velocity is the motion from `previous`
// to `current` divided by `interval` (seconds between them), and the result is
// `current` advanced by that velocity for `horizon` seconds (translation and
// rotation, in the car frame). Used both for the NDT Initial guess (horizon = time
// to the next Scan) and to bring a scan-time Pose estimate up to the display frame.
// Returns `current` unchanged if the interval is degenerate or the implied speed is
// implausible (a sign the last match diverged); the horizon is capped.
Eigen::Matrix4d predictPose(const Eigen::Matrix4d& previous, const Eigen::Matrix4d& current,
                            double interval, double horizon);

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
