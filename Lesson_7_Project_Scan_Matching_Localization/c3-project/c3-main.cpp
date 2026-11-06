
#include <carla/client/Client.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Map.h>
#include <carla/geom/Location.h>
#include <carla/geom/Transform.h>
#include <carla/client/Sensor.h>
#include <carla/sensor/data/LidarMeasurement.h>
#include <thread>

#include <carla/client/Vehicle.h>

//pcl code
//#include "render/render.h"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using namespace std;

#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include "helper.h"
#include <sstream>
#include <chrono> 
#include <ctime> 
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "localization.h"

// Voxel leaf size (m) applied to each Scan before matching: fewer points = faster NDT.
constexpr float kScanLeafSize = 0.5f;

PointCloudT pclCloud;
cc::Vehicle::Control control;
std::chrono::time_point<std::chrono::system_clock> currentTime;
vector<ControlState> cs;

bool refresh_view = false;
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer)
{

  	//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	if (event.getKeySym() == "Right" && event.keyDown()){
		cs.push_back(ControlState(0, -0.02, 0));
  	}
	else if (event.getKeySym() == "Left" && event.keyDown()){
		cs.push_back(ControlState(0, 0.02, 0)); 
  	}
  	if (event.getKeySym() == "Up" && event.keyDown()){
		cs.push_back(ControlState(0.1, 0, 0));
  	}
	else if (event.getKeySym() == "Down" && event.keyDown()){
		cs.push_back(ControlState(-0.1, 0, 0)); 
  	}
	if(event.getKeySym() == "a" && event.keyDown()){
		refresh_view = true;
	}
}

void Accuate(ControlState response, cc::Vehicle::Control& state){

	if(response.t > 0){
		if(!state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = false;
			state.throttle = min(response.t, 1.0f);
		}
	}
	else if(response.t < 0){
		response.t = -response.t;
		if(state.reverse){
			state.throttle = min(state.throttle+response.t, 1.0f);
		}
		else{
			state.reverse = true;
			state.throttle = min(response.t, 1.0f);

		}
	}
	state.steer = min( max(state.steer+response.s, -1.0f), 1.0f);
	state.brake = response.b;
}

void drawCar(Pose pose, int num, Color color, double alpha, pcl::visualization::PCLVisualizer::Ptr& viewer){

	BoxQ box;
	box.bboxTransform = Eigen::Vector3f(pose.position.x, pose.position.y, 0);
    box.bboxQuaternion = getQuaternion(pose.rotation.yaw);
    box.cube_length = 4;
    box.cube_width = 2;
    box.cube_height = 2;
	renderBox(viewer, box, num, color, alpha);
}

int main(){

	auto client = cc::Client("localhost", 2000);
	client.SetTimeout(2s);
	auto world = client.GetWorld();

	auto blueprint_library = world.GetBlueprintLibrary();
	auto vehicles = blueprint_library->Filter("vehicle");

	auto map = world.GetMap();
	auto transform = map->GetRecommendedSpawnPoints()[1];
	auto ego_actor = world.SpawnActor((*vehicles)[12], transform);

	//Create lidar
	auto lidar_bp = *(blueprint_library->Find("sensor.lidar.ray_cast"));
	// CANDO: Can modify lidar values to get different scan resolutions
	lidar_bp.SetAttribute("upper_fov", "15");
    lidar_bp.SetAttribute("lower_fov", "-25");
    lidar_bp.SetAttribute("channels", "32");
    lidar_bp.SetAttribute("range", "30");
	lidar_bp.SetAttribute("rotation_frequency", "60");
	lidar_bp.SetAttribute("points_per_second", "500000");

	auto user_offset = cg::Location(0, 0, 0);
	auto lidar_transform = cg::Transform(cg::Location(-0.5, 0, 1.8) + user_offset);
	auto lidar_actor = world.SpawnActor(lidar_bp, lidar_transform, ego_actor.get());
	auto lidar = boost::static_pointer_cast<cc::Sensor>(lidar_actor);
	bool new_scan = true;
	std::chrono::time_point<std::chrono::system_clock> lastScanTime, startTime;

	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	viewer->setBackgroundColor (0, 0, 0);
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	auto vehicle = boost::static_pointer_cast<cc::Vehicle>(ego_actor);
	Pose pose(Point(0,0,0), Rotate(0,0,0));

	// Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile("map.pcd", *mapCloud);
  	cout << "Loaded " << mapCloud->points.size() << " data points from map.pcd" << endl;
	renderPointCloud(viewer, mapCloud, "map", Color(0,0,1)); 

	typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
	typename pcl::PointCloud<PointT>::Ptr scanCloud (new pcl::PointCloud<PointT>);

	lidar->Listen([&new_scan, &lastScanTime, &scanCloud](auto data){

		if(new_scan){
			auto scan = boost::static_pointer_cast<csd::LidarMeasurement>(data);
			static int rawDumps = 0;  // DIAG: inspect raw point layout
			if (rawDumps < 2 && scan->size() > 12) {
				rawDumps++;
				cout << "RAW size=" << scan->size() << " channels=" << scan->GetChannelCount() << " :";
				for (size_t k = 0; k < 12; ++k) { auto d = (*scan)[k]; cout << " (" << d.x << "," << d.y << "," << d.z << ")"; }
				cout << endl;
			}
			for (auto detection : *scan){
				if((detection.x*detection.x + detection.y*detection.y + detection.z*detection.z) > 8.0){
					pclCloud.points.push_back(PointT(detection.x, detection.y, detection.z));
				}
			}
			if(pclCloud.points.size() > 5000){ // CANDO: Can modify this value to get different scan resolutions
				lastScanTime = std::chrono::system_clock::now();
				*scanCloud = pclCloud;
				new_scan = false;
			}
		}
	});
	
	Pose poseRef(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180));
	double maxError = 0;

	// Scan matcher: the Map's NDT grid is built once here, not per Scan.
	NdtLocalizer localizer(mapCloud, NdtConfig{});
	pcl::KdTreeFLANN<PointT> mapTree; mapTree.setInputCloud(mapCloud);  // DIAG
	PointCloudT::Ptr transformedScan(new PointCloudT);
	// Ground-truth pose is used exactly once, to seed the first Pose estimate.
	bool isPoseInitialized = false;
	Eigen::Matrix4d previousTransform = Eigen::Matrix4d::Identity();
	Eigen::Matrix4d currentTransform = Eigen::Matrix4d::Identity();

	while (!viewer->wasStopped())
  	{
		while(new_scan){
			std::this_thread::sleep_for(0.1s);
			world.Tick(1s);
		}
		if(refresh_view){
			viewer->setCameraPosition(pose.position.x, pose.position.y, 60, pose.position.x+1, pose.position.y+1, 0, 0, 0, 1);
			refresh_view = false;
		}
		
		viewer->removeShape("box0");
		viewer->removeShape("boxFill0");
		Pose truePose = Pose(Point(vehicle->GetTransform().location.x, vehicle->GetTransform().location.y, vehicle->GetTransform().location.z), Rotate(vehicle->GetTransform().rotation.yaw * pi/180, vehicle->GetTransform().rotation.pitch * pi/180, vehicle->GetTransform().rotation.roll * pi/180)) - poseRef;
		drawCar(truePose, 0,  Color(1,0,0), 0.7, viewer);
		double theta = truePose.rotation.yaw;
		double stheta = control.steer * pi/4 + theta;
		viewer->removeShape("steer");
		renderRay(viewer, Point(truePose.position.x+2*cos(theta), truePose.position.y+2*sin(theta),truePose.position.z),  Point(truePose.position.x+4*cos(stheta), truePose.position.y+4*sin(stheta),truePose.position.z), "steer", Color(0,1,0));


		ControlState accuate(0, 0, 1);
		if(cs.size() > 0){
			accuate = cs.back();
			cs.clear();

			Accuate(accuate, control);
			vehicle->ApplyControl(control);
		}

  		viewer->spinOnce ();
		
		if(!new_scan){
			
			// Snapshot the Scan and empty the lidar buffer *before* re-arming the
			// listener, so the lidar thread never writes while we are reading.
			PointCloudT::Ptr currentScan(new PointCloudT(*scanCloud));
			pclCloud.points.clear();
			new_scan = true;

			if(!isPoseInitialized){
				pose = truePose;
				currentTransform = poseToMatrix(pose);
				previousTransform = currentTransform;
				isPoseInitialized = true;
			}

			// Step 1: downsample the Scan with a voxel grid.
			cloudFiltered = voxelFilter(currentScan, kScanLeafSize);

			// Step 2: NDT-match the Scan to the Map, seeded by a constant-velocity Initial guess.
			const Eigen::Matrix4d initialGuess = currentTransform;  // EXPERIMENT: no motion prediction
			const Eigen::Matrix4d estimate = localizer.align(cloudFiltered, initialGuess);
			previousTransform = currentTransform;
			currentTransform = estimate;
			pose = getPose(estimate);
			{   // DIAG: mean NN distance of the scan to the map at truth vs at the estimate, and sweep coverage
				auto meanNN = [&](const Eigen::Matrix4d& T){
					PointCloudT moved; pcl::transformPointCloud(*cloudFiltered, moved, T);
					double sum = 0; std::vector<int> idx(1); std::vector<float> d2(1);
					for (const auto& p : moved.points){ mapTree.nearestKSearch(p, 1, idx, d2); sum += std::sqrt(d2[0]); }
					return sum / std::max<size_t>(1, moved.size());
				};
				int bins[8] = {0};
				for (const auto& p : currentScan->points){ int b = int((std::atan2(p.y, p.x) + pi) / (2*pi) * 8) % 8; bins[b]++; }
				static bool searched = false;
				if (!searched) {  // one-time grid search: which frame variant makes the scan fit the map at truth?
					searched = true;
					auto inlierFrac = [&](const Eigen::Matrix4d& Tm){
						PointCloudT moved; pcl::transformPointCloud(*cloudFiltered, moved, Tm);
						int in = 0; std::vector<int> idx(1); std::vector<float> d2(1);
						for (const auto& p : moved.points){ mapTree.nearestKSearch(p, 1, idx, d2); if (d2[0] < 0.09f) in++; }
						return double(in) / std::max<size_t>(1, moved.size());
					};
					const Eigen::Matrix4d T0 = poseToMatrix(truePose);
					cout << "GRID truth inliers=" << inlierFrac(T0) << endl;
					double best = -1; std::string bestDesc;
					for (int flip = 0; flip < 2; ++flip) for (int yawFlip = 0; yawFlip < 2; ++yawFlip) for (int fz = 0; fz < 2; ++fz)
					for (double dx = -2; dx <= 2.01; dx += 0.25) for (double dy = -2; dy <= 2.01; dy += 0.25) {
						Eigen::Matrix4d F = Eigen::Matrix4d::Identity(); if (flip) F(1,1) = -1; if (fz) F(2,2) = -1;
						Eigen::Matrix4d R = transform3D(yawFlip ? pi : 0, 0, 0, 0, 0, 0);
						Eigen::Matrix4d D = transform3D(0, 0, 0, dx, dy, 0);
						const double f = inlierFrac(D * T0 * R * F);
						if (f > best) { best = f; bestDesc = "flipY=" + to_string(flip) + " yawPi=" + to_string(yawFlip) + " flipZ=" + to_string(fz) + " dx=" + to_string(dx) + " dy=" + to_string(dy); }
					}
					cout << "GRID best inliers=" << best << " " << bestDesc << endl;
				}
				Eigen::Matrix4d flipY = Eigen::Matrix4d::Identity(); flipY(1,1) = -1;
				Eigen::Matrix4d down = Eigen::Matrix4d::Identity(); down(2,3) = -1.8;
				const Eigen::Matrix4d T = poseToMatrix(truePose);
				cout << "VAR plain=" << meanNN(T) << " flipY=" << meanNN(T*flipY) << " down=" << meanNN(down*T)
				     << " flipYdown=" << meanNN(down*T*flipY) << endl;
				cout << "DIAG nnTruth=" << meanNN(poseToMatrix(truePose)) << " nnEst=" << meanNN(estimate) << " sectors=";
				for (int b : bins) cout << b << ",";
				cout << endl;
			}
			cout << "scan pts=" << cloudFiltered->size()
			     << " conv=" << localizer.hasConverged() << " it=" << localizer.iterations()
			     << " fit=" << localizer.fitnessScore()
			     << " guess=(" << initialGuess(0,3) << "," << initialGuess(1,3) << ")"
			     << " est=(" << pose.position.x << "," << pose.position.y << "," << pose.position.z << " yaw " << pose.rotation.yaw << ")"
			     << " true=(" << truePose.position.x << "," << truePose.position.y << "," << truePose.position.z << " yaw " << truePose.rotation.yaw << ")" << endl;

			// Step 3: move the Scan into the Map frame using the Pose estimate and render it.
			pcl::transformPointCloud(*cloudFiltered, *transformedScan, estimate);
			viewer->removePointCloud("scan");
			renderPointCloud(viewer, transformedScan, "scan", Color(1,0,0) );

			viewer->removeAllShapes();
			drawCar(pose, 1,  Color(0,1,0), 0.35, viewer);
          
          	double poseError = sqrt( (truePose.position.x - pose.position.x) * (truePose.position.x - pose.position.x) + (truePose.position.y - pose.position.y) * (truePose.position.y - pose.position.y) );
			if(poseError > maxError)
				maxError = poseError;
			double distDriven = sqrt( (truePose.position.x) * (truePose.position.x) + (truePose.position.y) * (truePose.position.y) );
			viewer->removeShape("maxE");
			viewer->addText("Max Error: "+to_string(maxError)+" m", 200, 100, 32, 1.0, 1.0, 1.0, "maxE",0);
			viewer->removeShape("derror");
			viewer->addText("Pose error: "+to_string(poseError)+" m", 200, 150, 32, 1.0, 1.0, 1.0, "derror",0);
			viewer->removeShape("dist");
			viewer->addText("Distance: "+to_string(distDriven)+" m", 200, 200, 32, 1.0, 1.0, 1.0, "dist",0);

			if(maxError > 1.2 || distDriven >= 170.0 ){
				viewer->removeShape("eval");
			if(maxError > 1.2){
				viewer->addText("Try Again", 200, 50, 32, 1.0, 0.0, 0.0, "eval",0);
			}
			else{
				viewer->addText("Passed!", 200, 50, 32, 0.0, 1.0, 0.0, "eval",0);
			}
		}
		}
  	}
	return 0;
}
