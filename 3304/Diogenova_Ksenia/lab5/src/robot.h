#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>

enum direction { FORWARD, BACK, RIGHT, LEFT };

class Robot {
private:
    ros::NodeHandle& m_node_handle;
    ros::Publisher m_gazebo_publisher;

    ros::Rate* m_full_rate;
    ros::Rate* m_half_rate;

    std::string m_name;

    gazebo_msgs::ModelState m_state_msg;

    double m_x;
    double m_y;
    double m_angle;
    direction m_direction;

    int m_fps;

public:
    Robot(ros::NodeHandle& hnd, int rate, std::string name, std::string sdf_model_file_path, double x, double y)
        : m_node_handle(hnd), m_name(name)
    {
        m_gazebo_publisher = m_node_handle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);

        m_full_rate = new ros::Rate(rate/2);
        m_half_rate = new ros::Rate(rate);
        m_fps = rate/2;

        m_x = x;
        m_y = y;
        m_angle = 0;
        m_direction = FORWARD;

        m_state_msg.model_name = m_name;
        m_state_msg.pose.position.x = m_x;
        m_state_msg.pose.position.y = m_y;

        spawnModel(sdf_model_file_path);
    }

    ~Robot()
    {
        delete m_full_rate;
        delete m_half_rate;
    }

    void spawnModel(std::string sdf_model_file_path) {
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

        spawn_msg.request.initial_pose = pose;
        srv.call(spawn_msg);
    }

    void deleteModel()
    {
        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient srv = m_node_handle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        gazebo_msgs::DeleteModel delete_msg;
        delete_msg.request.model_name = m_name;

        srv.call(delete_msg);
    }

    void move(double x, double y){
        if (x > m_x) {
            bool turned = _turn(FORWARD);

            double dx = (x - m_x) / m_fps;
            for (int i = 0; i < m_fps; i++) {
                m_x += dx;
                m_state_msg.pose.position.x = m_x;
                m_state_msg.pose.position.y = m_y;
                m_gazebo_publisher.publish(m_state_msg);

                if (turned)
                    m_half_rate->sleep();
                else
                    m_full_rate->sleep();
            }
            m_x = x;
        } else if (x < m_x) {
            bool turned = _turn(BACK);

            double dx = (m_x - x) / m_fps;
            for (int i = 0; i < m_fps; i++) {
                m_x -= dx;
                m_state_msg.pose.position.x = m_x;
                m_state_msg.pose.position.y = m_y;
                m_gazebo_publisher.publish(m_state_msg);

                if (turned)
                    m_half_rate->sleep();
                else
                    m_full_rate->sleep();
            }
            m_x = x;
        }

        if (y > m_y)
        {
            bool turned = _turn(LEFT);

            double dy = (y - m_y) / m_fps;
            for (int i = 0; i < m_fps; i++)
            {
                m_y += dy;
                m_state_msg.pose.position.x = m_x;
                m_state_msg.pose.position.y = m_y;
                m_gazebo_publisher.publish(m_state_msg);

                if (turned)
                    m_half_rate->sleep();
                else
                    m_full_rate->sleep();
            }
            m_y = y;
        }
        else if (y < m_y)
        {
            bool turned = _turn(RIGHT);

            double dy = (m_y - y) / m_fps;
            for (int i = 0; i < m_fps; i++)
            {
                m_y -= dy;
                m_state_msg.pose.position.x = m_x;
                m_state_msg.pose.position.y = m_y;
                m_gazebo_publisher.publish(m_state_msg);

                if (turned)
                    m_half_rate->sleep();
                else
                    m_full_rate->sleep();
            }
            m_y = y;
        }
    }

private:
    bool _turn(direction to) {
        if (to == m_direction) {
            return false;
        }

        switch(to)
        {
        case FORWARD:
            switch(m_direction)
            {
            case BACK:
                _turnAround();
                break;
            case RIGHT:
                _turnConterclockwise();
                break;
            case LEFT:
                _turnClockwise();
                break;
            }
            break;
        case BACK:
            switch(m_direction)
            {
            case FORWARD:
                _turnAround();
                break;
            case RIGHT:
                _turnClockwise();
                break;
            case LEFT:
                _turnConterclockwise();
                break;
            }
            break;
        case RIGHT:
            switch(m_direction)
            {
            case FORWARD:
                _turnClockwise();
                break;
            case BACK:
                _turnConterclockwise();
                break;
            case LEFT:
                _turnAround();
                break;
            }
            break;
        case LEFT:
            switch(m_direction)
            {
            case FORWARD:
                _turnConterclockwise();
                break;
            case BACK:
                _turnClockwise();
                break;
            case RIGHT:
                _turnAround();
                break;
            }
            break;
        }

        m_direction = to;
        return true;
    }

    void _turnClockwise()
    {
        _turn(3.14159/2, true);
    }

    void _turnConterclockwise()
    {
        _turn(3.14159/2, false);
    }

    void _turnAround()
    {
        _turn(3.14159, false);
    }

    void _turn(double angle, bool clockwise)
    {
        tf::Quaternion tfq;
        geometry_msgs::Quaternion q;

        double dangle = angle / m_fps;

        for (int i = 0; i < m_fps; i++)
        {
            if (clockwise)
                m_angle -= dangle;
            else
                m_angle += dangle;

            tfq.setRPY(0, 0, m_angle);
            tf::quaternionTFToMsg(tfq, q);

            m_state_msg.pose.orientation = q;
            m_gazebo_publisher.publish(m_state_msg);
            m_half_rate->sleep();
        }
    }
};

#endif // ROBOT_H

