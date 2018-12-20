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
public:
    enum status
    {
        CREATED,
        SPAWNED,
        MOVING,
        STOPPED,
        SET
    };

private:
    ros::NodeHandle& m_node_handle;
    ros::Rate m_rate;
    ros::Publisher m_gazebo_publisher;

    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;

    std::string m_name;

    gazebo_msgs::ModelState m_state_msg;

    double m_x;
    double m_y;
    double m_z;

    status m_status;

    std::thread* m_move_thread;
    bool m_move_canceled;

public:
    Robot(ros::NodeHandle& hnd, int rate, std::string name) : m_node_handle(hnd),  m_rate(ros::Rate(rate)), m_name(name)
    {
        m_gazebo_publisher = m_node_handle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);
        m_state_msg.model_name = m_name;
        m_move_canceled = false;
        m_move_thread = nullptr;
        m_status = Robot::status::CREATED;
    }

    ~Robot()
    {
        if (m_status != Robot::status::CREATED)
        {
            deleteModel();
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

        m_status = Robot::status::SPAWNED;
    }

    void deleteModel()
    {
        stop();

        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient srv = m_node_handle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        gazebo_msgs::DeleteModel delete_msg;
        delete_msg.request.model_name = m_name;

        srv.call(delete_msg);
    }

    Robot::status getStatus()
    {
        return m_status;
    }

    void stop()
    {
        m_move_canceled = true;
        if (m_move_thread != nullptr)
        {
            m_move_thread->join();
            delete m_move_thread;
            m_move_thread = nullptr;
        }
        m_status = Robot::status::STOPPED;
    }

    void move(double x, double y, double z)
    {
        m_move_canceled = false;
        m_move_thread = new std::thread( [=] { this->_move(x, y, z); } );
        m_status = Robot::status::MOVING;
    }

private:
    void _move(double x, double y, double z)
    {
        float dx = fabs(x - m_x) / 1000;
        dx *= (x - m_x) > 0 ? 1 : -1;
        float dy = fabs(y - m_y) / 1000;
        dy *= (y - m_y) > 0 ? 1 : -1;
        float dz = fabs(z - m_z) / 1000;
        dy *= (z - m_z) > 0 ? 1 : -1;

        while (!m_move_canceled)
        {
            transform.setOrigin(tf::Vector3(m_x, m_y, m_z));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", m_name));

            if (m_x != x && m_y != y && m_z != z)
            {
                m_x += dx;
                m_y += dy;
                m_z += dz;
            }
            else
            {
                if (m_status != Robot::status::SET)
                    m_status = Robot::status::SET;
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
