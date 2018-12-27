#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <math.h>
#include <iostream>

class Robot
{
private:
    ros::NodeHandle& _node_handle;
    ros::Publisher _gazebo_publisher;

    ros::Rate* _full_rate;
    ros::Rate* _half_rate;

    std::string _name;

    gazebo_msgs::ModelState _state_msg;

    double _x;
    double _y;
    double _angle;

    int _fps;

public:
    Robot(ros::NodeHandle& hnd, int rate, std::string name, std::string sdf_model_file_path, double x, double y)
        : _node_handle(hnd), _name(name)
    {
        _gazebo_publisher = _node_handle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);

        _full_rate = new ros::Rate(rate/2);
        _half_rate = new ros::Rate(rate);
        _fps = rate/5;

        _x = x;
        _y = y;
        _angle = 0.0;

        _state_msg.model_name = _name;
        _state_msg.pose.position.x = _x;
        _state_msg.pose.position.y = _y;

		tf::Quaternion tfq;
        geometry_msgs::Quaternion q;
		tfq.setRPY(0, 0, _angle);
        tf::quaternionTFToMsg(tfq, q);
        _state_msg.pose.orientation = q;

        spawnModel(sdf_model_file_path);
    }

    ~Robot()
    {
        delete _full_rate;
        delete _half_rate;
    }

    void spawnModel(std::string sdf_model_file_path)
    {
        ros::service::waitForService("gazebo/spawn_sdf_model");
        ros::ServiceClient srv = _node_handle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

        std::ifstream fin(sdf_model_file_path.c_str());

        std::string model, buf;
        while (!fin.eof() && fin)
		{
            getline(fin, buf);
            model += buf + "\n";
        }

        gazebo_msgs::SpawnModel spawn_msg;
        spawn_msg.request.model_xml = model;
        spawn_msg.request.model_name = _name;

        geometry_msgs::Pose pose;
        pose.position.x = _x;
        pose.position.y = _y;

        spawn_msg.request.initial_pose = pose;
        srv.call(spawn_msg);
    }

    void deleteModel()
    {
        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient srv = _node_handle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        gazebo_msgs::DeleteModel delete_msg;
        delete_msg.request.model_name = _name;

        srv.call(delete_msg);
    }

    void move(double x, double y)
    {
		double dx = (x - _x) / _fps;
		double dy = (y - _y) / _fps;

		double angle = atan2(dy, dx);
		_turn(angle, _angle > angle ? true : false);

        for (int i = 0; i < _fps; i++)
        {
			_x += dx;
			_y += dy;
            _state_msg.pose.position.x = _x;
            _state_msg.pose.position.y = _y;
			
			tf::Quaternion tfq;
        	geometry_msgs::Quaternion q;
			tfq.setRPY(0, 0, _angle);
            tf::quaternionTFToMsg(tfq, q);
            _state_msg.pose.orientation = q;

            _gazebo_publisher.publish(_state_msg);
            _full_rate->sleep();
        }
    }

private:
    void _turn(double angle, bool clockwise)
    {
        tf::Quaternion tfq;
        geometry_msgs::Quaternion q;

        double dangle = std::abs(_angle - angle) / _fps;

        for (int i = 0; i < _fps; i++)
        {
            if (clockwise)
                _angle -= dangle;
            else
                _angle += dangle;

            tfq.setRPY(0, 0, _angle);
            tf::quaternionTFToMsg(tfq, q);

            _state_msg.pose.orientation = q;
            _gazebo_publisher.publish(_state_msg);
            _half_rate->sleep();
        }
    }
};

#endif // ROBOT_H

