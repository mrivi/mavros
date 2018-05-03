/**
 * @brief Trajectory plugin
 * @file trajectory.cpp
 * @author Martina Rivizzigno <martina@rivizzigno.it>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Martina Rivizzigno.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.mdA
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Trajectory.h>
#include <mavros_msgs/PositionTarget.h>
#include <nav_msgs/Path.h>

//! Point array length
static constexpr int LEN = 11;

namespace mavros {
namespace extra_plugins {
//! Mavlink MAV_TRAJECTORY_REPRESENTATION enumeration
using mavlink::common::MAV_TRAJECTORY_REPRESENTATION;

/**
 * @brief Trajectory plugin to receive planned path from the FCU and
 * send back to the FCU a corrected path (collision free, smoothed)
 *
 * @see trajectory_cb()
 */
class TrajectoryPlugin : public plugin::PluginBase {
public:
	TrajectoryPlugin() : PluginBase(),
		trajectory_nh("~trajectory")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		trajectory_generated_sub = trajectory_nh.subscribe("generated", 10, &TrajectoryPlugin::trajectory_cb, this);
		path_sub = trajectory_nh.subscribe("path", 10, &TrajectoryPlugin::path_cb, this);
		trajectory_desired_pub = trajectory_nh.advertise<mavros_msgs::Trajectory>("desired", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&TrajectoryPlugin::handle_trajectory)
		};
	}

private:
	ros::NodeHandle trajectory_nh;

	ros::Subscriber trajectory_generated_sub;
	ros::Subscriber path_sub;

	ros::Publisher trajectory_desired_pub;

	// auto fill_points_variant = [](std::array<float, N?> &point, const float yaw) {
	//      //point[9] = yaw;

	//      print("%f %f \n", point[0], yaw);
	// };

	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#TRAJECTORY
	 * @param req	received Trajectory msg
	 */
	void trajectory_cb(const mavros_msgs::Trajectory::ConstPtr &req)
	{
		mavlink::common::msg::TRAJECTORY trajectory {};
		trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		trajectory.type = req->type;	//!< trajectory type (waypoints, bezier)

		auto fill_points_position = [] (std::array<float, LEN> &point_output, const float x, const float y, const float z) {
			// [[[cog:
			//     cog.outl("auto position_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(x, y, z));")
			//     for index, axis in zip ("012", "xyz"):
			//         cog.outl("point_output[{index}] = position_ned.{axis}();".format(**locals()))
			// ]]]
			auto position_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(x, y, z));
			point_output[0] = position_ned.x();
			point_output[1] = position_ned.y();
			point_output[2] = position_ned.z();
			// [[[end]]] (checksum: 36ff3775b62c418569f350f85ad777ba) (checksum: ) 
		};

		auto fill_points_velocity = [] (std::array<float, LEN> &point_output, const float vx, const float vy, const float vz) {
			// [[[cog:
			//     cog.outl("auto velocity_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(vx, vy, vz));")
			//     for index, axis in zip ("345", "xyz"):
			//         cog.outl("point_output[{index}] = velocity_ned.{axis}();".format(**locals()))
			// ]]]
			auto velocity_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(vx, vy, vz));
			point_output[3] = velocity_ned.x();
			point_output[4] = velocity_ned.y();
			point_output[5] = velocity_ned.z();
			// [[[end]]] (checksum: 6303daf8d94ad15651c2724c504eeb6a) 
		};

		auto fill_points_acceleration = [] (std::array<float, LEN> &point_output, const float ax, const float ay, const float az) {
			// [[[cog:
			//     cog.outl("auto acceleration_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(ax, ay, az));")
			//     for index, axis in zip ("678", "xyz"):
			//         cog.outl("point_output[{index}] = acceleration_ned.{axis}();".format(**locals()))
			// ]]]
			auto acceleration_ned = ftf::transform_frame_enu_ned(Eigen::Vector3d(ax, ay, az));
			point_output[6] = acceleration_ned.x();
			point_output[7] = acceleration_ned.y();
			point_output[8] = acceleration_ned.z();
			// [[[end]]] (checksum: 1d50f8d1225d33a50e130aa98ec7a9cc) 
		};

		auto fill_points_yaw_wp = [this](std::array<float, LEN> &point, float yaw) {
			// cog...
			point[9] = wrap_pi(-yaw + (M_PI / 2.0f));
		};

		auto fill_points_yaw_bezier = [this](std::array<float, LEN> &point, float yaw) {
			// cog...
			point[4] = wrap_pi(-yaw + (M_PI / 2.0f));
		};

		auto fill_points_yaw_speed = [] (std::array<float, LEN> &point, float yaw_speed) {
			// cog...
			point[10] = yaw_speed;
		};

		auto fill_points_time_horizon = [] (std::array<float, LEN> &point, float time_horizon) {
			// cog...
			point[3] = time_horizon;
		};

		auto fill_points_unused_bezier = [] (std::array<float, LEN> &point) {
			// [[[cog:
			// for p in range(5, 11):
			//     cog.outl("point[{p}] = NAN;".format(**locals()))
			// ]]]
			point[5] = NAN;
			point[6] = NAN;
			point[7] = NAN;
			point[8] = NAN;
			point[9] = NAN;
			point[10] = NAN;
			// [[[end]]] (checksum: 0a9f760963830f9af1afb1b68ebc8089)
		};

		// [[[cog:
		// for p in "12345":
		//     cog.outl("fill_points_position(trajectory.point_{p}, req->point_{p}[0], req->point_{p}[1], req->point_{p}[2]);".format(**locals()))
		// ]]]
		fill_points_position(trajectory.point_1, req->point_1[0], req->point_1[1], req->point_1[2]);
		fill_points_position(trajectory.point_2, req->point_2[0], req->point_2[1], req->point_2[2]);
		fill_points_position(trajectory.point_3, req->point_3[0], req->point_3[1], req->point_3[2]);
		fill_points_position(trajectory.point_4, req->point_4[0], req->point_4[1], req->point_4[2]);
		fill_points_position(trajectory.point_5, req->point_5[0], req->point_5[1], req->point_5[2]);
		// [[[end]]] (checksum: 06829bdb3c2fd0fd257f8d6676b9d5f6)

		if (req->type == utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS)) {
			// [[[cog:
			// for p in "12345":
			//     cog.outl("fill_points_velocity(trajectory.point_{p}, req->point_{p}[3], req->point_{p}[4], req->point_{p}[5]);".format(**locals()))
			//     cog.outl("fill_points_acceleration(trajectory.point_{p}, req->point_{p}[6], req->point_{p}[7], req->point_{p}[8]);".format(**locals()))
			//     cog.outl("fill_points_yaw_wp(trajectory.point_{p}, req->point_{p}[9]);".format(**locals()))
			//     cog.outl("fill_points_yaw_speed(trajectory.point_{p}, req->point_{p}[10]);\n".format(**locals()))
			// ]]]
			fill_points_velocity(trajectory.point_1, req->point_1[3], req->point_1[4], req->point_1[5]);
			fill_points_acceleration(trajectory.point_1, req->point_1[6], req->point_1[7], req->point_1[8]);
			fill_points_yaw_wp(trajectory.point_1, req->point_1[9]);
			fill_points_yaw_speed(trajectory.point_1, req->point_1[10]);

			fill_points_velocity(trajectory.point_2, req->point_2[3], req->point_2[4], req->point_2[5]);
			fill_points_acceleration(trajectory.point_2, req->point_2[6], req->point_2[7], req->point_2[8]);
			fill_points_yaw_wp(trajectory.point_2, req->point_2[9]);
			fill_points_yaw_speed(trajectory.point_2, req->point_2[10]);

			fill_points_velocity(trajectory.point_3, req->point_3[3], req->point_3[4], req->point_3[5]);
			fill_points_acceleration(trajectory.point_3, req->point_3[6], req->point_3[7], req->point_3[8]);
			fill_points_yaw_wp(trajectory.point_3, req->point_3[9]);
			fill_points_yaw_speed(trajectory.point_3, req->point_3[10]);

			fill_points_velocity(trajectory.point_4, req->point_4[3], req->point_4[4], req->point_4[5]);
			fill_points_acceleration(trajectory.point_4, req->point_4[6], req->point_4[7], req->point_4[8]);
			fill_points_yaw_wp(trajectory.point_4, req->point_4[9]);
			fill_points_yaw_speed(trajectory.point_4, req->point_4[10]);

			fill_points_velocity(trajectory.point_5, req->point_5[3], req->point_5[4], req->point_5[5]);
			fill_points_acceleration(trajectory.point_5, req->point_5[6], req->point_5[7], req->point_5[8]);
			fill_points_yaw_wp(trajectory.point_5, req->point_5[9]);
			fill_points_yaw_speed(trajectory.point_5, req->point_5[10]);

			// [[[end]]] (checksum: d3f0c8dff4a7a8015e9e0e08b860e737) (checksum: )
		} else {
			// [[[cog:
			// for p in "12345":
			//     cog.outl("fill_points_time_horizon(trajectory.point_{p}, req->point_{p}[3]);".format(**locals()))
			//     cog.outl("fill_points_yaw_bezier(trajectory.point_{p}, req->point_{p}[4]);".format(**locals()))
			//     cog.outl("fill_points_unused_bezier(trajectory.point_{p});\n".format(**locals()))
			// ]]]
			fill_points_time_horizon(trajectory.point_1, req->point_1[3]);
			fill_points_yaw_bezier(trajectory.point_1, req->point_1[4]);
			fill_points_unused_bezier(trajectory.point_1);

			fill_points_time_horizon(trajectory.point_2, req->point_2[3]);
			fill_points_yaw_bezier(trajectory.point_2, req->point_2[4]);
			fill_points_unused_bezier(trajectory.point_2);

			fill_points_time_horizon(trajectory.point_3, req->point_3[3]);
			fill_points_yaw_bezier(trajectory.point_3, req->point_3[4]);
			fill_points_unused_bezier(trajectory.point_3);

			fill_points_time_horizon(trajectory.point_4, req->point_4[3]);
			fill_points_yaw_bezier(trajectory.point_4, req->point_4[4]);
			fill_points_unused_bezier(trajectory.point_4);

			fill_points_time_horizon(trajectory.point_5, req->point_5[3]);
			fill_points_yaw_bezier(trajectory.point_5, req->point_5[4]);
			fill_points_unused_bezier(trajectory.point_5);

			// [[[end]]] (checksum: 65161665011194b0e5bcdd2ebef39b34)
		}

		std::copy(req->point_valid.begin(), req->point_valid.end(), trajectory.point_valid.begin());

		UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
	}


	/**
	 * @brief Send corrected path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#TRAJECTORY
	 * @param req	received nav_msgs Path msg
	 */
	void path_cb(const nav_msgs::Path::ConstPtr &req)
	{
		mavlink::common::msg::TRAJECTORY trajectory {};
		trajectory.time_usec = req->header.stamp.toNSec() / 1000;	//!< [milisecs]
		trajectory.type = utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS);
		Eigen::Quaterniond q_enu;
		Eigen::Vector3d position_wp, orientation_wp;
		int conter = 0;

		auto fill_points_position = [] (std::array<float, LEN> &point, geometry_msgs::Point position) {
			// [[[cog:
			//     cog.outl("auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));")
			//     for index, axis in zip ("012", "xyz"):
			//         cog.outl("point[{index}] = position_ned.{axis}();".format(**locals()))
			// ]]]
			auto position_ned = ftf::transform_frame_enu_ned(ftf::to_eigen(position));
			point[0] = position_ned.x();
			point[1] = position_ned.y();
			point[2] = position_ned.z();
			// [[[end]]] (checksum: 6b564ac3664cbc831bc9d5d912c976c0)
		};

		auto fill_points_yaw = [this](std::array<float, LEN> &point, geometry_msgs::Quaternion orientation) {
			// cog...
			Eigen::Quaterniond q_enu;
			tf::quaternionMsgToEigen(orientation, q_enu);
			auto orientation_wp = ftf::quaternion_to_rpy(ftf::transform_orientation_enu_ned(ftf::transform_orientation_baselink_aircraft(q_enu)));
			point[9] = wrap_pi(-orientation_wp(2) + (M_PI / 2.0f));
		};

		auto fill_points_unused_fields = [] (std::array<float, LEN> &point) {
			// [[[cog:
			// for p in range(3, 9):
			//     cog.outl("point[{p}] = NAN;".format(**locals()))
			// cog.outl("point[10] = NAN;".format(**locals()))
			// ]]]
			point[3] = NAN;
			point[4] = NAN;
			point[5] = NAN;
			point[6] = NAN;
			point[7] = NAN;
			point[8] = NAN;
			point[10] = NAN;
			// [[[end]]] (checksum: 78bad8df6dced32241db3b734ffd1361)
		};

		auto fill_points_unused = [] (std::array<float, LEN> &point) {
			// [[[cog:
			// for p in range(0, 11):
			//     cog.outl("point[{p}] = NAN;".format(**locals()))
			// ]]]
			point[0] = NAN;
			point[1] = NAN;
			point[2] = NAN;
			point[3] = NAN;
			point[4] = NAN;
			point[5] = NAN;
			point[6] = NAN;
			point[7] = NAN;
			point[8] = NAN;
			point[9] = NAN;
			point[10] = NAN;
			// [[[end]]] (checksum: 2adcbc0e6a99f22697649e405e22c86a)
		};

		// [[[cog:
		// for p in "12345":
		//    cog.outl("if (req->poses.size() >= {p}) {{ \n".format(**locals()))
		//    cog.outl("fill_points_position(trajectory.point_{p}, req->poses[{p} - 1].pose.position);".format(**locals()))
		//    cog.outl("fill_points_yaw(trajectory.point_{p}, req->poses[{p} - 1].pose.orientation);".format(**locals()))
		//    cog.outl("fill_points_unused_fields(trajectory.point_{p});".format(**locals()))
		//    cog.outl("} else {")
		//    cog.outl("fill_points_unused(trajectory.point_{p});".format(**locals()))
		//    cog.outl("} \n")
		// ]]]
		if (req->poses.size() >= 1) { 

		fill_points_position(trajectory.point_1, req->poses[1 - 1].pose.position);
		fill_points_yaw(trajectory.point_1, req->poses[1 - 1].pose.orientation);
		fill_points_unused_fields(trajectory.point_1);
		} else {
		fill_points_unused(trajectory.point_1);
		} 

		if (req->poses.size() >= 2) { 

		fill_points_position(trajectory.point_2, req->poses[2 - 1].pose.position);
		fill_points_yaw(trajectory.point_2, req->poses[2 - 1].pose.orientation);
		fill_points_unused_fields(trajectory.point_2);
		} else {
		fill_points_unused(trajectory.point_2);
		} 

		if (req->poses.size() >= 3) { 

		fill_points_position(trajectory.point_3, req->poses[3 - 1].pose.position);
		fill_points_yaw(trajectory.point_3, req->poses[3 - 1].pose.orientation);
		fill_points_unused_fields(trajectory.point_3);
		} else {
		fill_points_unused(trajectory.point_3);
		} 

		if (req->poses.size() >= 4) { 

		fill_points_position(trajectory.point_4, req->poses[4 - 1].pose.position);
		fill_points_yaw(trajectory.point_4, req->poses[4 - 1].pose.orientation);
		fill_points_unused_fields(trajectory.point_4);
		} else {
		fill_points_unused(trajectory.point_4);
		} 

		if (req->poses.size() >= 5) { 

		fill_points_position(trajectory.point_5, req->poses[5 - 1].pose.position);
		fill_points_yaw(trajectory.point_5, req->poses[5 - 1].pose.orientation);
		fill_points_unused_fields(trajectory.point_5);
		} else {
		fill_points_unused(trajectory.point_5);
		} 

		// [[[end]]] (checksum: 2f3c59967b0cd4b9167720a16adc1471) 


		// check that either x and y are finite or z to set the position waypoint as valid
		if ((std::isfinite(trajectory.point_1[0]) && std::isfinite(trajectory.point_1[1])) || std::isfinite(trajectory.point_1[2])) {
			trajectory.point_valid[0] = true;
		} else {
			trajectory.point_valid[0] = false;
		}

		if ((std::isfinite(trajectory.point_2[0]) && std::isfinite(trajectory.point_2[1])) || std::isfinite(trajectory.point_2[2])) {
			trajectory.point_valid[1] = true;
		} else {
			trajectory.point_valid[1] = false;
		}

		if ((std::isfinite(trajectory.point_3[0]) && std::isfinite(trajectory.point_3[1])) || std::isfinite(trajectory.point_3[2])) {
			trajectory.point_valid[2] = true;
		} else {
			trajectory.point_valid[2] = false;
		}

		if ((std::isfinite(trajectory.point_4[0]) && std::isfinite(trajectory.point_4[1])) || std::isfinite(trajectory.point_4[2])) {
			trajectory.point_valid[3] = true;
		} else {
			trajectory.point_valid[3] = false;
		}

		if ((std::isfinite(trajectory.point_5[0]) && std::isfinite(trajectory.point_5[1])) || std::isfinite(trajectory.point_5[2])) {
			trajectory.point_valid[4] = true;
		} else {
			trajectory.point_valid[4] = false;
		}

		UAS_FCU(m_uas)->send_message_ignore_drop(trajectory);
	}

	void handle_trajectory(const mavlink::mavlink_message_t *msg, mavlink::common::msg::TRAJECTORY &trajectory)
	{
		auto trajectory_desired = boost::make_shared<mavros_msgs::Trajectory>();
		trajectory_desired->header = m_uas->synchronized_header("local_origin", trajectory.time_usec);
		trajectory_desired->type = trajectory.type;						//!< trajectory type (waypoints, bezier)

		auto fill_points_position = [] (float &x, float &y, float &z, std::array<float, LEN> &point_input) {
			// [[[cog:
			//     cog.outl("auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(point_input[0], point_input[1], point_input[2]));".format(**locals()))
			//     for axis in "xyz":
			//         cog.outl("{axis} = enu_position.{axis}();".format(**locals()))
			// ]]]
			auto enu_position = ftf::transform_frame_ned_enu(Eigen::Vector3d(point_input[0], point_input[1], point_input[2]));
			x = enu_position.x();
			y = enu_position.y();
			z = enu_position.z();
			// [[[end]]] (checksum: af1b5403b030d9197230a5114ce658f4) (checksum: ) 
		};

		auto fill_points_velocity = [] (float &x, float &y, float &z, std::array<float, LEN> &point_input) {
			// [[[cog:
			//     cog.outl("auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(point_input[3], point_input[4], point_input[5]));".format(**locals()))
			//     for axis in "xyz":
			//         cog.outl("{axis} = enu_velocity.{axis}();".format(**locals()))
			// ]]]
			auto enu_velocity = ftf::transform_frame_ned_enu(Eigen::Vector3d(point_input[3], point_input[4], point_input[5]));
			x = enu_velocity.x();
			y = enu_velocity.y();
			z = enu_velocity.z();
			// [[[end]]] (checksum: 077fab27fcb68425d69f06b3aa58d075)
		};

		auto fill_points_acceleration = [] (float &x, float &y, float &z, std::array<float, LEN> &point_input) {
			// [[[cog:
			//     cog.outl("auto enu_acceleration = ftf::transform_frame_ned_enu(Eigen::Vector3d(point_input[6], point_input[7], point_input[8]));".format(**locals()))
			//     for axis in "xyz":
			//         cog.outl("{axis} = enu_acceleration.{axis}();".format(**locals()))
			// ]]]
			auto enu_acceleration = ftf::transform_frame_ned_enu(Eigen::Vector3d(point_input[6], point_input[7], point_input[8]));
			x = enu_acceleration.x();
			y = enu_acceleration.y();
			z = enu_acceleration.z();
			// [[[end]]] (checksum: 44441e64183333a9572f2d5b98eae090)
		};

		auto fill_points_yaw = [this](float &yaw_output, float yaw_input) {
			yaw_output = wrap_pi((M_PI / 2.0f) - yaw_input);
		};

		auto fill_points_yaw_speed = [] (float &yaw_speed_output, float yaw_speed_input) {
			yaw_speed_output = yaw_speed_input;
		};

		auto fill_points_time_horizon = [] (float &time_horizon_output, float time_horizon_input){
			time_horizon_output = time_horizon_input;
		};

		auto fill_points_unused_bezier = [] (float *point_output){
			// [[[cog:
			// for index in range(5, 11):
			//     cog.outl("point_output[{index}] = NAN;".format(**locals()))
			// ]]]
			point_output[5] = NAN;
			point_output[6] = NAN;
			point_output[7] = NAN;
			point_output[8] = NAN;
			point_output[9] = NAN;
			point_output[10] = NAN;
			// [[[end]]] (checksum: 8d7950818c8cdfc5fbeb32f3df49de35)
		};

		// [[[cog:
		// for p in "12345":
		//     cog.outl("fill_points_position(trajectory_desired->point_{p}[0], trajectory_desired->point_{p}[1], trajectory_desired->point_{p}[2], trajectory.point_{p});".format(**locals()))
		// ]]]
		fill_points_position(trajectory_desired->point_1[0], trajectory_desired->point_1[1], trajectory_desired->point_1[2], trajectory.point_1);
		fill_points_position(trajectory_desired->point_2[0], trajectory_desired->point_2[1], trajectory_desired->point_2[2], trajectory.point_2);
		fill_points_position(trajectory_desired->point_3[0], trajectory_desired->point_3[1], trajectory_desired->point_3[2], trajectory.point_3);
		fill_points_position(trajectory_desired->point_4[0], trajectory_desired->point_4[1], trajectory_desired->point_4[2], trajectory.point_4);
		fill_points_position(trajectory_desired->point_5[0], trajectory_desired->point_5[1], trajectory_desired->point_5[2], trajectory.point_5);
		// [[[end]]] (checksum: 02f241798785acc3dfce53e51c2ed1c4)


		if (trajectory.type == utils::enum_value(MAV_TRAJECTORY_REPRESENTATION::WAYPOINTS)) {
			// [[[cog:
			// for p in "12345":
			//    cog.outl("fill_points_velocity(trajectory_desired->point_{p}[3], trajectory_desired->point_{p}[4], trajectory_desired->point_{p}[5], trajectory.point_{p});".format(**locals()))
			//    cog.outl("fill_points_acceleration(trajectory_desired->point_{p}[6], trajectory_desired->point_{p}[7], trajectory_desired->point_{p}[8], trajectory.point_{p});".format(**locals()))
			//    cog.outl("fill_points_yaw(trajectory_desired->point_{p}[9], trajectory.point_{p}[9]);".format(**locals()))
			//    cog.outl("fill_points_yaw_speed(trajectory_desired->point_{p}[10], trajectory.point_{p}[10]);\n".format(**locals()))
			// ]]]
			fill_points_velocity(trajectory_desired->point_1[3], trajectory_desired->point_1[4], trajectory_desired->point_1[5], trajectory.point_1);
			fill_points_acceleration(trajectory_desired->point_1[6], trajectory_desired->point_1[7], trajectory_desired->point_1[8], trajectory.point_1);
			fill_points_yaw(trajectory_desired->point_1[9], trajectory.point_1[9]);
			fill_points_yaw_speed(trajectory_desired->point_1[10], trajectory.point_1[10]);

			fill_points_velocity(trajectory_desired->point_2[3], trajectory_desired->point_2[4], trajectory_desired->point_2[5], trajectory.point_2);
			fill_points_acceleration(trajectory_desired->point_2[6], trajectory_desired->point_2[7], trajectory_desired->point_2[8], trajectory.point_2);
			fill_points_yaw(trajectory_desired->point_2[9], trajectory.point_2[9]);
			fill_points_yaw_speed(trajectory_desired->point_2[10], trajectory.point_2[10]);

			fill_points_velocity(trajectory_desired->point_3[3], trajectory_desired->point_3[4], trajectory_desired->point_3[5], trajectory.point_3);
			fill_points_acceleration(trajectory_desired->point_3[6], trajectory_desired->point_3[7], trajectory_desired->point_3[8], trajectory.point_3);
			fill_points_yaw(trajectory_desired->point_3[9], trajectory.point_3[9]);
			fill_points_yaw_speed(trajectory_desired->point_3[10], trajectory.point_3[10]);

			fill_points_velocity(trajectory_desired->point_4[3], trajectory_desired->point_4[4], trajectory_desired->point_4[5], trajectory.point_4);
			fill_points_acceleration(trajectory_desired->point_4[6], trajectory_desired->point_4[7], trajectory_desired->point_4[8], trajectory.point_4);
			fill_points_yaw(trajectory_desired->point_4[9], trajectory.point_4[9]);
			fill_points_yaw_speed(trajectory_desired->point_4[10], trajectory.point_4[10]);

			fill_points_velocity(trajectory_desired->point_5[3], trajectory_desired->point_5[4], trajectory_desired->point_5[5], trajectory.point_5);
			fill_points_acceleration(trajectory_desired->point_5[6], trajectory_desired->point_5[7], trajectory_desired->point_5[8], trajectory.point_5);
			fill_points_yaw(trajectory_desired->point_5[9], trajectory.point_5[9]);
			fill_points_yaw_speed(trajectory_desired->point_5[10], trajectory.point_5[10]);

			// [[[end]]] (checksum: 52e1ca28b3fd6608f9cd5303123aaae6)
		} else {
			// [[[cog:
			// for p in "12345":
			//     cog.outl("fill_points_time_horizon(trajectory_desired->point_{p}[3], trajectory.point_{p}[3]);".format(**locals()))
			//     cog.outl("fill_points_yaw(trajectory_desired->point_{p}[4], trajectory.point_{p}[4]);".format(**locals()))
			//     cog.outl("fill_points_unused_bezier(trajectory_desired->point_{p});\n".format(**locals()))
			// ]]]
			fill_points_time_horizon(trajectory_desired->point_1[3], trajectory.point_1[3]);
			fill_points_yaw(trajectory_desired->point_1[4], trajectory.point_1[4]);
			fill_points_unused_bezier(&trajectory_desired->point_1[0]);

			fill_points_time_horizon(trajectory_desired->point_2[3], trajectory.point_2[3]);
			fill_points_yaw(trajectory_desired->point_2[4], trajectory.point_2[4]);
			fill_points_unused_bezier(&trajectory_desired->point_2[0]);

			fill_points_time_horizon(trajectory_desired->point_3[3], trajectory.point_3[3]);
			fill_points_yaw(trajectory_desired->point_3[4], trajectory.point_3[4]);
			fill_points_unused_bezier(&trajectory_desired->point_3[0]);

			fill_points_time_horizon(trajectory_desired->point_4[3], trajectory.point_4[3]);
			fill_points_yaw(trajectory_desired->point_4[4], trajectory.point_4[4]);
			fill_points_unused_bezier(&trajectory_desired->point_4[0]);

			fill_points_time_horizon(trajectory_desired->point_5[3], trajectory.point_5[3]);
			fill_points_yaw(trajectory_desired->point_5[4], trajectory.point_5[4]);
			fill_points_unused_bezier(&trajectory_desired->point_5[0]);

			// [[[end]]] (checksum: dc08c77bafbfab80cef7f413eaf5b18f)
		}

		std::copy(trajectory.point_valid.begin(), trajectory.point_valid.end(), trajectory_desired->point_valid.begin());

		trajectory_desired_pub.publish(trajectory_desired);
	}

	float wrap_pi(float a)
	{
		if (!std::isfinite(a)) {
			return a;
		}

		return fmod(a + M_PI, 2.0f * M_PI) - M_PI;
	}
};
}					// namespace extra_plugins
}				// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::TrajectoryPlugin, mavros::plugin::PluginBase)
