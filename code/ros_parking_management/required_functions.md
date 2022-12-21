# Required Functions, Libraries and Classes

Not all libraries used in this package are open source. Therefore some functions or libraries have to be implemented on your own. This file documents the non-open source functions used in this package that have to implemented in order to run this package:

# geom Library

The geom library is used to define geometric functions and objects.

### geom::LineStrip2<T>

A linestrip is a sequence of line segments. The library stores it internally as a series of points

### geom::Ring2

A two-dimensional closed polygon without holes. It consists of a single linestrip of consecutive edges. The edges are ordered in counter clockwise orientation and the first and last point are equal.

### geom::Point2

A two-dimensional point

### geom::Pose<T,T> and geom::Pose2d

A two-dimensional pose with x, y and the yaw angle. Pose2d uses the datatype double for x,y and yaw.

### geom::Polygon2<T>

A polygon consist of an outer bound and a list of inner rings (holes).

### geom::Polygon2<T>::rectangle (Scalar length, Scalar width)

Creates a polygon with a rectangle shape with the given length and width.

### geom::Polygon2Vector<T>

A std::vector out of 2D polygons.

### ros_geom::Pose2Conversions<T,T>::from()

Converts a geometry_msgs::Pose to geom::Pose

### ros_geom::Pose2Conversions<T,T>::to()

Converts a geom::Pose to geometry_msgs::Pose

### ros_geom::Ring2Conversions<T>::toMarker()

Visualizes a ring with a visualization_msgs::MarkerArray

### ros_geom::Polygon2Conversions<double>::toMarkerArray

Visualizes a polygon with a visualization_msgs::MarkerArray

## math Library

### math::angle

Calculates the angle from an vector orientation and returns it in radian from [-$\pi$,$\pi$]

### math::openmp::RandomNumberEngine

Engine to generate random numbers

### math::interpolateLinear(value1, value2, fraction)

Interpolate linearly by fraction between two values.

### math::interpolateRadAngleLinear(angle1, angle2, fraction)

Interpolate between two angles in rad linearly, uses values between [$0$,$2\pi$]

### math::normalizeAngleUnsigned

Normalizes a rad angle to [$0$,$2\pi$]


# ros_scene_prediction

### ros_scene_prediction::Scene

Object describing the current scene.

### ros_scene_prediction::Obstacle

Obstacles contain attributes like their pose, shape, classification and an ID. They are used to populate the scene with objects that have to be avoided. They can be either static or dynamic.

### ros_scene_prediction::Obstacle::Components

Components of the Obstacle, specifically: ID, POSE, DIMENSIONS, CLASSIFICATION, VELOCITY and ACCELERATION

### ros_scene_prediction::ObstacleClassification::Static

A static obstacle that does not move, therefore there is no prediction of its movement.

### ros_scene_prediction::ObstacleClassification::Vehicle

Use this to classify an object as a vehicle.

### ros_scene_prediction::VehicleControlParameter

Use this struct to define vehicle behavior like maximum braking distance and desired acceleration or braking strength

### ros_scene_prediction::updateRouteFromPosition

The current route is updated with the current object position

### ros_scene_prediction::initializePredictionsLMM

Initialize predictions for obstacles that have a route and shall be predicted with the lanelet
motion model afterwards

### ros_scene_prediction::initializePredictionsCVM

Initialize predictions for obstacles that shall be predicted with a prediction model that only
requires the dynamic state of the object

### ros_scene_prediction::CVMotionModel::MotionFilter

This is a special extended Kalman Filter

### ros_scene_prediction::MixedMotionModel::predict

Predict the future trajectory of all obstacles

### ros_scene_prediction::toCenterline

Given a Lanelet map and a vector of Lanelet IDs, return the centerline of all specified lanelets as a geom::Linestrip

### ros_scene_prediction::LaneletMatch

A struct used to match positions with lanelets. A matching is can either be "is on". This means a position (or object) is inside a lanelet aka between the left and right bound. Otherwise a matching can be "following". This matching is performed to find out, which lanelet an object is currently following and its direction of movement is considered.

### ros_scene_prediction::LaneletMapper::isOnLanelets

Returns True if the position "is on" the given Lanelets or inside the allowed tolerance zone

### ros_scene_prediction::LaneletMapper::LaneletPobabilisticMapping

Matches the objects to the Lanelets in the Lanelet Map

### ros_scene_prediction::LaneletMapper::isFollowingLaneletsProbabilistic

Returns True if object follows Lanelet with certain probability

### ros_scene_prediction::LaneletMotionModel::MotionFilter

EKF or KF used for motion updates in motion model that follows Lanelets

### ros_scene_prediction::Scene::geomCenterline()

Returns a geom::LineStrip2d consisting of the center lines of lanelets

### ros_scene_prediction::Prediction

Struct that contains all information about prediction of an object like the future positions, mappings and route.
