# Glossary — Scan Matching Localization

**Ground-truth pose**
The simulator's true position and orientation of the car. Used only to seed the very first Pose estimate.

**Pose estimate**
The car's position and orientation as inferred from lidar Scans matched against the Map (drawn as the green box).

**Pose error**
The horizontal (x, y) distance between the Ground-truth pose and the Pose estimate. A run fails once it exceeds 1.2 m.

**Scan**
One lidar sweep, expressed in the car's own (local) frame.

**Map**
The prebuilt point cloud of the environment (`map.pcd`), expressed in the world frame anchored at the car's start.

**Initial guess**
The transform handed to the scan matcher as its starting point for aligning a Scan to the Map.
