/*
    robot_joint_vel_control node

    Copyright 2023 Università degli studi della Campania "Luigi Vanvitelli"
    Author: Marco Costanzo <marco.costanzo@unicampania.it>
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <ros/ros.h>
#include "std_srvs/SetBool.h"
#include "sensor_msgs/JointState.h"

bool vel_mode = false;
sensor_msgs::JointState cmd_joint_position, cmd_joint_vel;

void initIntegrator()
{
    cmd_joint_position = *ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");
    cmd_joint_position.name.pop_back();
    cmd_joint_position.name.pop_back();
    cmd_joint_position.position.pop_back();
    cmd_joint_position.position.pop_back();

    cmd_joint_vel = cmd_joint_position;
    cmd_joint_vel.velocity.resize(7);
    for (int i = 0; i < cmd_joint_vel.velocity.size(); i++)
    {
        cmd_joint_vel.velocity[i] = 0.0;
    }
    ROS_INFO_STREAM("[ROBOT CONTROL]: robot state is\n"
                    << cmd_joint_position);
}

bool set_mode_vel_cmd_srv_cb(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{

    if (req.data)
    {
        // vel mode
        initIntegrator();
        ROS_INFO_STREAM("[ROBOT CONTROL]: joint_velocity");
    }
    else
    {
        // position mode
        ROS_INFO_STREAM("[ROBOT CONTROL]: joint_position");
    }

    vel_mode = req.data;

    res.success = true;

    return true;
}

void cmd_joint_vel_cb(const sensor_msgs::JointState::ConstPtr &msg)
{
    if (!vel_mode)
    {
        ROS_ERROR_STREAM_THROTTLE(1, "[ROBOT CONTROL]: the robot is not in velocity mode!");
        return;
    }
    if (msg->velocity.size() == cmd_joint_vel.velocity.size())
    {
        cmd_joint_vel.velocity = msg->velocity;
    }
    else
    {
        ROS_ERROR_STREAM_THROTTLE(1, "[ROBOT CONTROL]: invalid command velocity vector size!");
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_joint_vel_control");

    ros::NodeHandle nh;

    ros::ServiceServer srv_set_mode_vel = nh.advertiseService("/velocity_mode", set_mode_vel_cmd_srv_cb);

    ros::Publisher pub_cmd_joint_position = nh.advertise<sensor_msgs::JointState>("/cmd/joint_position", 1);

    ros::Subscriber sub_cmd_joint_vel = nh.subscribe("/cmd/joint_velocity", 1, cmd_joint_vel_cb);

    ROS_INFO_STREAM("[ROBOT CONTROL] Initialized");

    double DT = 0.01;
    ros::Rate loop_rate(1.0 / DT);
    while (ros::ok())
    {
        if (!vel_mode)
        {
            loop_rate.sleep();
            ros::spinOnce();
            continue;
        }

        for (int i = 0; i < cmd_joint_position.position.size(); i++)
        {
            cmd_joint_position.position[i] += DT * cmd_joint_vel.velocity[i];
        }

        pub_cmd_joint_position.publish(cmd_joint_position);
        ros::spinOnce();
    }

    return 0;
}
