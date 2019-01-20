#ifndef ROBOT_H
#define ROBOT_H

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>

class Robot
{
private:
    ros::NodeHandle& nodeHandle;
    ros::Publisher modelStatePublisher;

    ros::Rate* rate;
    ros::Rate* m_half_rate;

    std::string modelName;

    gazebo_msgs::ModelState modelState;
    geometry_msgs::Pose pose;

    // position and direction of robot
    double angle;

    int fps;

public:
    Robot(ros::NodeHandle& hnd, int rate, std::string name, std::string sdf_model_file_path, double x, double y)
        : nodeHandle(hnd), modelName(name)
    {
        modelStatePublisher = nodeHandle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 1000);

        this->rate = new ros::Rate(rate/2);
        m_half_rate = new ros::Rate(rate);
        fps = rate/2;

        pose.position.x=x;
        pose.position.y=y;
        angle = 0;

        modelState.model_name = modelName;

        spawnModel(sdf_model_file_path);
    }

    ~Robot()
    {
        delete rate;
        delete m_half_rate;
    }

    void spawnModel(std::string sdf_model_file_path)
    {
        ros::service::waitForService("gazebo/spawn_sdf_model");
        ros::ServiceClient srv = nodeHandle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

        std::ifstream fin(sdf_model_file_path.c_str());

        std::string model, buf;
        while (!fin.eof() && fin) {
            getline(fin, buf);
            model += buf + "\n";
        }

        gazebo_msgs::SpawnModel spawn_msg;
        spawn_msg.request.model_xml = model;
        spawn_msg.request.model_name = modelName;

        geometry_msgs::Pose pose;
        pose.position.x = this->pose.position.x;
        pose.position.y = this->pose.position.y;

        spawn_msg.request.initial_pose = pose;
        
        srv.call(spawn_msg);
    }

    void deleteModel()
    {
        ros::service::waitForService("gazebo/delete_model");
        ros::ServiceClient srv = nodeHandle.serviceClient<gazebo_msgs::DeleteModel>("gazebo/delete_model");

        gazebo_msgs::DeleteModel delete_msg;
        delete_msg.request.model_name = modelName;

        srv.call(delete_msg);
    }

    void move(double x, double y)
    {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        double dx, dy;
        int signX, signY;   
        if(x > pose.position.x) {
            dx = (x - pose.position.x);
            signX = 1;
        }
        else {
            dx = (pose.position.x - x);
            signX = -1;
        }
        if(y > pose.position.y) {
            dy = (y - pose.position.y);
            signY = 1;
        }
        else {
            dy = (pose.position.y - y);
            signY = -1;
        }
        dx /= fps;
        dy /= fps;
        std::cout<<dx<<" dy "<<dy<<std::endl;
        
        turnRobot(x, y);
        for (int i = 0; i < fps; i++){
            pose.position.x += signX * dx;
            pose.position.y += signY * dy;
            modelState.pose =pose;
            modelStatePublisher.publish(modelState);
            rate->sleep();
        }
        
    }


private:
void turnRobot(double moveToX, double moveToY) {
        double width = moveToX - pose.position.x;
        double height = moveToY - pose.position.y;
        double lenght = sqrt(pow(width, 2) + pow(height, 2));
        double angle;

        // static double lastAngle = 0;

        if (lenght == 0) 
            return;

        std::cout<<"w "<<width<<" h "<<height<<" l "<<lenght<<std::endl;
        angle = asin(height/lenght);

        std::cout<<"angle id"<<angle<<std::endl;
        // if(moveToX < pose.position.x && moveToY > pose.position.y){
        //     angle += M_PI / 2;
        // } else if (moveToX < pose.position.x && moveToY < pose.position.y){
        //     angle += M_PI;
        // } else if (moveToX > pose.position.x && moveToY < pose.position.y) {
        //     angle += (3*M_PI) / 2;
        // }

        // if(abs(width) < 1e-5) angle = asin(height/lenght);
        // else angle = acos(width/lenght);
        std::cout<<moveToX<<";"<<moveToY<<std::endl;
        std::cout<<pose.position.x<<";"<<pose.position.y;
        // std::cout<<"[1] angle "<<angle <<" lastAngle "<<lastAngle<<std::endl;
        angle -= this->angle;
        // lastAngle += angle;
        // while (lastAngle > 2*M_PI)
        //     lastAngle -= 2*M_PI;
        // while (lastAngle < 0)
        //     lastAngle += 2*M_PI;
        // std::cout<<"[2] angle "<<angle <<" lastAngle "<<lastAngle<<std::endl;
        _turn(angle, false);
    }

    void _turn(double angle, bool clockwise)
    {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        tf::Quaternion tfq;
        geometry_msgs::Quaternion q;
        
        ros::Rate turnRate(60);

        double dangle = angle / fps;

        for (int i = 0; i < fps; i++)
        {
            if (clockwise)
                this->angle -= dangle;
            else
                this->angle += dangle;
            tfq.setRPY(0, 0, this->angle);
            tf::quaternionTFToMsg(tfq, q);
            pose.orientation = q;
            modelState.pose =pose;
            modelStatePublisher.publish(modelState);
            rate->sleep();
        }
    }
};

#endif // ROBOT_H
