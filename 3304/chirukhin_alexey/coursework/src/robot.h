#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <thread>


class Robot
{
private:
	enum robotStatus
	{
		FLYING,
		SET,
		DEAD
	}
    ros::NodeHandle& m_node_handle;
    ros::Rate& m_rate;
    ros::Publisher m_gazebo_publisher;

    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;

    std::string m_name;

    gazebo_msgs::ModelState m_state_msg;

    double m_x;
    double m_y;
    double m_z;

	robotStatus status;

    std::thread* m_move_thread;
    bool canceled;

public:
    Robot(ros::NodeHandle& hnd, ros::Rate& rate, std::string name)
        : m_node_handle(hnd), m_rate(rate), m_name(name)
    {
        m_gazebo_publisher = m_node_handle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
        m_state_msg.model_name = m_name;
        canceled = false;
    }

    ~Robot()
    {
        canceled = true;
        if (m_move_thread != nullptr)
        {
            m_move_thread->join();
        }
    }

    void spawnModel(std::string sdf_model_file_path, double x, double y, double z)
    {
        m_x = x;
        m_y = y;
        m_z = z;

        ros::service::waitForService("gazebo/spawn_sdf_model");
        ros::ServiceClient srv = m_node_handle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

        std::ifstream fin(sdf_model_file_path.c_str());

        std::string model, buf;
        while (!fin.eof() && fin) {
            getline(fin, buf);
            model += buf + "\n";
        }

        gazebo_msgs::SpawnModel spawn_msg;
        spawn_msg.request.model_xml = model;
        spawn_msg.request.model_name = m_name;

        geometry_msgs::Pose pose;
        pose.position.x = m_x;
        pose.position.y = m_y;
        pose.position.z = m_z;

        spawn_msg.request.initial_pose = pose;
        srv.call(spawn_msg);
    }

    void deleteModel()
    {
        canceled = true;
        if (m_move_thread != nullptr)
        {
            m_move_thread->join();
        }

        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient srv = m_node_handle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        gazebo_msgs::DeleteModel delete_msg;
        delete_msg.request.model_name = m_name;

        srv.call(delete_msg);
    }

    void move(double x, double y, double z)
    {
		status = robotStatus::FLYING;
        m_move_thread = new std::thread( [=] { this->_move(x, y, z); } );
    }

private:
    void _move(double x, double y, double z)
    {
        while (!canceled)
        {
            transform.setOrigin(tf::Vector3(m_x, m_y, m_z));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", m_name));

            if (m_x != x && m_y != y && m_z != z)
            {
                float dx = fabs(x - m_x) / 1000;
                dx *= (x - m_x) > 0 ? 1 : -1;
                float dy = fabs(y - m_y) / 1000;
                dy *= (y - m_y) > 0 ? 1 : -1;
                float dz = fabs(z - m_z) / 1000;
                dy *= (z - m_z) > 0 ? 1 : -1;

                m_x += dx;
                m_y += dy;
                m_z += dz;
            }

            m_state_msg.pose.position.x = m_x;
            m_state_msg.pose.position.y = m_y;
            m_state_msg.pose.position.z = m_z;
            m_gazebo_publisher.publish(m_state_msg);

            m_rate.sleep();
        }
    }
};

#endif // ROBOT_H
