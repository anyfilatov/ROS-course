#ifndef ROBOT_H
#define ROBOT_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <string>
#include <thread>
#include <iostream>

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

protected:
    ros::NodeHandle& m_node_handle;

private:
    ros::Rate m_rate;
    ros::Publisher m_gazebo_publisher;
    ros::Publisher m_appear_publisher;
    ros::Subscriber m_shot_subscriber;
    gazebo_msgs::ModelState m_state_msg;

    tf::TransformBroadcaster tf_broadcaster;
    tf::Transform transform;

    std::string m_name;

    int m_initial_x;
    int m_initial_y;

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
        m_appear_publisher = m_node_handle.advertise<std_msgs::String>("appear", 10);
        m_shot_subscriber = m_node_handle.subscribe("shoot/" + m_name, 10, &Robot::_shotCallback, this);
        m_state_msg.model_name = m_name;
        m_move_canceled = false;
        m_move_thread = nullptr;
        m_status = Robot::status::CREATED;
    }

    virtual ~Robot()
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
        while (!fin.eof() && fin)
        {
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

        std_msgs::String msg;
        msg.data = m_name;
        m_appear_publisher.publish(msg);

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

    void move(double x, double y, double z, double speed)
    {
        m_initial_x = x;
        m_initial_y = y;
        m_move_canceled = false;
        m_move_thread = new std::thread( [=] { this->_move(x, y, z, speed); } );
        m_status = Robot::status::MOVING;
    }

    std::string getName()
    {
        return m_name;
    }

    Robot::status getStatus()
    {
        return m_status;
    }

    int getInitialX()
    {
        return m_initial_x;
    }

    int getInitialY()
    {
        return m_initial_y;
    }

private:
    void _move(double x, double y, double z, double speed)
    {
        double precision = 0.001;

        float dx = fabs(x - m_x) / speed;
        dx *= (x - m_x) > 0 ? 1 : -1;
        float dy = fabs(y - m_y) / speed;
        dy *= (y - m_y) > 0 ? 1 : -1;
        float dz = fabs(z - m_z) / speed;
        dz *= (z - m_z) > 0 ? 1 : -1;

        while (!m_move_canceled)
        {
            transform.setOrigin(tf::Vector3(m_x, m_y, m_z));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", m_name));

            if (std::abs(m_x - x) > precision)
                m_x += dx;
            if (std::abs(m_y - y) > precision)
                m_y += dy;
            if (std::abs(m_z - z) > precision)
                m_z += dz;

            if (std::abs(m_x - x) < precision &&
                std::abs(m_y - y) < precision &&
                std::abs(m_z - z) < precision)
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

    void _shotCallback(const std_msgs::String& msg)
    {
        if (m_status != Robot::status::SET)
            stop();
    }
};

#endif // ROBOT_H
