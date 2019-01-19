#ifndef RobotCar_H
#define RobotCar_H

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>

enum direction { FORWARD, BACK, RIGHT, LEFT };

class RobotCar
{
private:
    ros::NodeHandle& gazebo_node;
    ros::Publisher gazebo_publisher;

    ros::Rate* gazebo_full_rate;
    ros::Rate* gazebo_half_rate;

    std::string gazebo_name;

    gazebo_msgs::ModelState gazebo_state_msg;

    // position and direction of RobotCar
    double gazebo_x;
    double gazebo_y;
    double gazebo_angle;
    direction gazebo_direction;

    int gazebo_fps;

public:
    RobotCar(ros::NodeHandle& hnd, int rate, std::string name, std::string sdf_model_file_path, double x, double y)
        : gazebo_node(hnd), gazebo_name(name)
    {
        gazebo_publisher = gazebo_node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);

        gazebo_full_rate = new ros::Rate(rate/2);
        gazebo_half_rate = new ros::Rate(rate);
        gazebo_fps = rate/2;

        gazebo_x = x;
        gazebo_y = y;
        gazebo_angle = 0;
        gazebo_direction = FORWARD;

        gazebo_state_msg.model_name = gazebo_name;
        gazebo_state_msg.pose.position.x = gazebo_x;
        gazebo_state_msg.pose.position.y = gazebo_y;

        spawnModel(sdf_model_file_path);
    }

    ~RobotCar()
    {
        delete gazebo_full_rate;
        delete gazebo_half_rate;
    }

    void spawnModel(std::string sdf_model_file_path)
    {
        ros::service::waitForService("gazebo/spawn_sdf_model");
        ros::ServiceClient srv = gazebo_node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

        std::ifstream fin(sdf_model_file_path.c_str());

        std::string model, buf;
        while (!fin.eof() && fin) {
            getline(fin, buf);
            model += buf + "\n";
        }

        gazebo_msgs::SpawnModel spawn_msg;
        spawn_msg.request.model_xml = model;
        spawn_msg.request.model_name = gazebo_name;

        geometry_msgs::Pose pose;
        pose.position.x = gazebo_x;
        pose.position.y = gazebo_y;

        spawn_msg.request.initial_pose = pose;
        srv.call(spawn_msg);
    }

    void deleteModel()
    {
        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient srv = gazebo_node.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        gazebo_msgs::DeleteModel delete_msg;
        delete_msg.request.model_name = gazebo_name;

        srv.call(delete_msg);
    }

    void move(double x, double y)
    {
        if (x > gazebo_x)
        {
            bool turned = _turn(FORWARD);

            double dx = (x - gazebo_x) / gazebo_fps;
            for (int i = 0; i < gazebo_fps; i++)
            {
                gazebo_x += dx;
                gazebo_state_msg.pose.position.x = gazebo_x;
                gazebo_state_msg.pose.position.y = gazebo_y;
                gazebo_publisher.publish(gazebo_state_msg);

                if (turned)
                    gazebo_half_rate->sleep();
                else
                    gazebo_full_rate->sleep();
            }
            gazebo_x = x;
        }
        else if (x < gazebo_x)
        {
            bool turned = _turn(BACK);

            double dx = (gazebo_x - x) / gazebo_fps;
            for (int i = 0; i < gazebo_fps; i++)
            {
                gazebo_x -= dx;
                gazebo_state_msg.pose.position.x = gazebo_x;
                gazebo_state_msg.pose.position.y = gazebo_y;
                gazebo_publisher.publish(gazebo_state_msg);

                if (turned)
                    gazebo_half_rate->sleep();
                else
                    gazebo_full_rate->sleep();
            }
            gazebo_x = x;
        }

        if (y > gazebo_y)
        {
            bool turned = _turn(LEFT);

            double dy = (y - gazebo_y) / gazebo_fps;
            for (int i = 0; i < gazebo_fps; i++)
            {
                gazebo_y += dy;
                gazebo_state_msg.pose.position.x = gazebo_x;
                gazebo_state_msg.pose.position.y = gazebo_y;
                gazebo_publisher.publish(gazebo_state_msg);

                if (turned)
                    gazebo_half_rate->sleep();
                else
                    gazebo_full_rate->sleep();
            }
            gazebo_y = y;
        }
        else if (y < gazebo_y)
        {
            bool turned = _turn(RIGHT);

            double dy = (gazebo_y - y) / gazebo_fps;
            for (int i = 0; i < gazebo_fps; i++)
            {
                gazebo_y -= dy;
                gazebo_state_msg.pose.position.x = gazebo_x;
                gazebo_state_msg.pose.position.y = gazebo_y;
                gazebo_publisher.publish(gazebo_state_msg);

                if (turned)
                    gazebo_half_rate->sleep();
                else
                    gazebo_full_rate->sleep();
            }
            gazebo_y = y;
        }
    }

private:
    bool _turn(direction to)
    {
        if (to == gazebo_direction)
        {
            return false;
        }

        switch(to)
        {
        case FORWARD:
            switch(gazebo_direction)
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
            switch(gazebo_direction)
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
            switch(gazebo_direction)
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
            switch(gazebo_direction)
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

        gazebo_direction = to;
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

        double dangle = angle / gazebo_fps;

        for (int i = 0; i < gazebo_fps; i++)
        {
            if (clockwise)
                gazebo_angle -= dangle;
            else
                gazebo_angle += dangle;

            tfq.setRPY(0, 0, gazebo_angle);
            tf::quaternionTFToMsg(tfq, q);

            gazebo_state_msg.pose.orientation = q;
            gazebo_publisher.publish(gazebo_state_msg);
            gazebo_half_rate->sleep();
        }
    }
};

#endif // RobotCar_H