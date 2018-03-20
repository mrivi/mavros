/**
 * @brief Obstacle avoidance plugin
 * @file obstacle_avoidance.cpp
 * @author Nuno Marques <martina@rivizzigno.com>
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
#include <mavros_msgs/ObstacleAvoidance.h>
#include <mavros_msgs/AnchorPoint.h>


// TODO: create ros message to send obstacle distanc

namespace mavros {
namespace extra_plugins {

//! Mavlink MAV_TRAJECTORY_REPRESENTATION enumeration
using mavlink::common::MAV_TRAJECTORY_REPRESENTATION;

/**
 * @brief Obstacle avoidance plugin to send collision
 * free path to the FCU
 * 
 * @see obstacle_cb()
 */
class ObstacleAvoidancePlugin : public plugin::PluginBase {
public:
	ObstacleAvoidancePlugin() : PluginBase(),
		obstacle_nh("~obstacle")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		obstacle_sub = obstacle_nh.subscribe("anchor_point", 10, &ObstacleAvoidancePlugin::obstacle_cb, this);
	}

	Subscriptions get_subscriptions()
	{
		return { /* Rx disabled */ };
	}

private:
	ros::NodeHandle obstacle_nh;
	ros::Subscriber obstacle_sub;

	/**
	 * @brief Send collision free path to the FCU.
	 *
	 * Message specification: http://mavlink.org/messages/common#OBSTACLE_AVOIDANCE
	 * @param req	received ObstacleAvoidance msg
	 */
	void obstacle_cb(const mavros_msgs::ObstacleAvoidance::ConstPtr &req)
	{
                mavlink::common::msg::OBSTACLE_AVOIDANCE obstacle_avoidance {};
                obstacle_avoidance.time_usec = req->header.stamp.toNSec() / 1000;//!< [milisecs]
                obstacle_avoidance.type = req->type;    	//!< trajectory type (waypoints, bezier)

                /* conversion between ENU and NED */
                obstacle_avoidance.point_1[0] = req->point_1.position_setpoint.y;
                obstacle_avoidance.point_1[1] = req->point_1.position_setpoint.x;
                obstacle_avoidance.point_1[2] = -req->point_1.position_setpoint.z;
                obstacle_avoidance.point_1[3] = req->point_1.velocity_setpoint.y;
                obstacle_avoidance.point_1[4] = req->point_1.velocity_setpoint.x;
                obstacle_avoidance.point_1[5] = -req->point_1.velocity_setpoint.z;
                obstacle_avoidance.point_1[6] = req->point_1.acceleration_setpoint.y;
                obstacle_avoidance.point_1[7] = req->point_1.acceleration_setpoint.x;
                obstacle_avoidance.point_1[8] = -req->point_1.acceleration_setpoint.z;
                obstacle_avoidance.point_1[9] = wrap_pi(req->point_1.yaw + (M_PI / 2.0f));
                obstacle_avoidance.point_1[10] = req->point_1.yaw_speed;

                obstacle_avoidance.point_2[0] = req->point_2.position_setpoint.y;
                obstacle_avoidance.point_2[1] = req->point_2.position_setpoint.x;
                obstacle_avoidance.point_2[2] = -req->point_2.position_setpoint.z;
                obstacle_avoidance.point_2[3] = req->point_2.velocity_setpoint.y;
                obstacle_avoidance.point_2[4] = req->point_2.velocity_setpoint.x;
                obstacle_avoidance.point_2[5] = -req->point_2.velocity_setpoint.z;
                obstacle_avoidance.point_2[6] = req->point_2.acceleration_setpoint.y;
                obstacle_avoidance.point_2[7] = req->point_2.acceleration_setpoint.x;
                obstacle_avoidance.point_2[8] = -req->point_2.acceleration_setpoint.z;
                obstacle_avoidance.point_2[9] = wrap_pi(req->point_2.yaw + (M_PI / 2.0f));
                obstacle_avoidance.point_2[10] = req->point_2.yaw_speed;

                obstacle_avoidance.point_3[0] = req->point_3.position_setpoint.y;
                obstacle_avoidance.point_3[1] = req->point_3.position_setpoint.x;
                obstacle_avoidance.point_3[2] = -req->point_3.position_setpoint.z;
                obstacle_avoidance.point_3[3] = req->point_3.velocity_setpoint.y;
                obstacle_avoidance.point_3[4] = req->point_3.velocity_setpoint.x;
                obstacle_avoidance.point_3[5] = -req->point_3.velocity_setpoint.z;
                obstacle_avoidance.point_3[6] = req->point_3.acceleration_setpoint.y;
                obstacle_avoidance.point_3[7] = req->point_3.acceleration_setpoint.x;
                obstacle_avoidance.point_3[8] = -req->point_3.acceleration_setpoint.z;
                obstacle_avoidance.point_3[9] = wrap_pi(req->point_3.yaw + (M_PI / 2.0f));
                obstacle_avoidance.point_3[10] = req->point_3.yaw_speed;

                obstacle_avoidance.point_4[0] = req->point_4.position_setpoint.y;
                obstacle_avoidance.point_4[1] = req->point_4.position_setpoint.x;
                obstacle_avoidance.point_4[2] = -req->point_4.position_setpoint.z;
                obstacle_avoidance.point_4[3] = req->point_4.velocity_setpoint.y;
                obstacle_avoidance.point_4[4] = req->point_4.velocity_setpoint.x;
                obstacle_avoidance.point_4[5] = -req->point_4.velocity_setpoint.z;
                obstacle_avoidance.point_4[6] = req->point_4.acceleration_setpoint.y;
                obstacle_avoidance.point_4[7] = req->point_4.acceleration_setpoint.x;
                obstacle_avoidance.point_4[8] = -req->point_4.acceleration_setpoint.z;
                obstacle_avoidance.point_4[9] = wrap_pi(req->point_4.yaw + (M_PI / 2.0f));
                obstacle_avoidance.point_4[10] = req->point_4.yaw_speed;

                obstacle_avoidance.point_5[0] = req->point_5.position_setpoint.y;
                obstacle_avoidance.point_5[1] = req->point_5.position_setpoint.x;
                obstacle_avoidance.point_5[2] = -req->point_5.position_setpoint.z;
                obstacle_avoidance.point_5[3] = req->point_5.velocity_setpoint.y;
                obstacle_avoidance.point_5[4] = req->point_5.velocity_setpoint.x;
                obstacle_avoidance.point_5[5] = -req->point_5.velocity_setpoint.z;
                obstacle_avoidance.point_5[6] = req->point_5.acceleration_setpoint.y;
                obstacle_avoidance.point_5[7] = req->point_5.acceleration_setpoint.x;
                obstacle_avoidance.point_5[8] = -req->point_5.acceleration_setpoint.z;
                obstacle_avoidance.point_5[9] = wrap_pi(req->point_5.yaw + (M_PI / 2.0f));
                obstacle_avoidance.point_5[10] = req->point_5.yaw_speed;

        	std::copy(req->point_valid.begin(), req->point_valid.end(), obstacle_avoidance.point_valid.begin());
        	std::copy(req->field_of_view.begin(), req->field_of_view.end(), obstacle_avoidance.field_of_view.begin());

        	UAS_FCU(m_uas)->send_message_ignore_drop(obstacle_avoidance);
	}

        float wrap_pi(float a)
        {

                if (!std::isfinite(a)) {
                        return a;
                }

                while (a >= M_PI) {
                        a -= (M_PI * 2.0f);
                }

                while (a < -M_PI) {
                        a += (M_PI * 2.0f);
                }

                return a;
        }
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::ObstacleAvoidancePlugin, mavros::plugin::PluginBase)
