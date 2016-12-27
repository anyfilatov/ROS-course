#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "robot_msg/robotMesg.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>

using namespace std;

        //base info.
        string name;
                int id;
        double robotX=0;
        double robotY=0;
        double angle=0;
        double goalX;
        double goalY;
        double curDeltX=0;
        double curDeltY=0;
        double curDelt=0.5;
        double curDeltAngle;
        string target="pioneer2d_1";
        //for visualization.
        ros::Publisher pub;
        gazebo_msgs::ModelState gazeboMsg;

        void broadcastPosition() {
                //for broadcast.
                tf::Quaternion quaternion;
                quaternion.setRPY(0, 0, 0);
                static tf::TransformBroadcaster broadCast;
                tf::Transform transform;
                // cout << "Searcher (" << name <<") went to x:" << x << " y:" << y << endl;
                transform.setOrigin(tf::Vector3(robotX, robotY, 0.0));
                transform.setRotation(quaternion);
                broadCast.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
        }

        void repaint() {
            gazeboMsg.pose.position.x = robotX;
            gazeboMsg.pose.position.y = robotY;
            gazeboMsg.pose.orientation.z = sin(angle / 2);
            gazeboMsg.pose.orientation.w = cos(angle / 2);
            pub.publish(gazeboMsg);
        }

        void driveToTarget(ros::NodeHandle node) {

                tf::TransformListener listener;
                tf::StampedTransform transform;

                int iter = 0;
                ros::Rate rate(120);
                while (node.ok()) {
                    //ROS_INFO("sTARTASS");
                        try {
                                        listener.lookupTransform(name, target, ros::Time(0), transform);
                            }
                            catch (tf::TransformException &e) {
                                    ROS_ERROR("%s", e.what());
                                    broadcastPosition();
                                    ros::Duration(1.0).sleep();
                                    ros::spinOnce();
                                    continue;
                            }
                     //ROS_INFO("CLOSEASS");

                        if (iter == 0) {
                                iter = 1000;

                                goalX += transform.getOrigin().x();
                                goalY += transform.getOrigin().y();

                                double dx = goalX - robotX;
                                double dy = goalY - robotY;
                                curDeltX = dx / iter;
                                curDeltY = dy / iter;
                                curDeltAngle = (-angle + atan2(dy, dx)) / iter;



                        }
                        if (transform.getOrigin().length() < curDelt){
                             ROS_INFO("yattutttt!!!!");
                            if (strcmp(target.c_str(), "world")==0) return;
                            target="world";
                            robot_msg::robotMesg msg;
                            msg.a=target;
                            ros::Publisher pub = node.advertise<robot_msg::robotMesg>("/chatter", 100);

                            ros::Rate loop_rate(2);
                            for (int i = 0; i < 2; i++)//////jjjjjjjjjj
                            {
                                    pub.publish(msg);
                                    loop_rate.sleep();

                            }
                            sleep(2);
                                return;
                        }
                        if (transform.getOrigin().length() > curDelt) {
                                robotX += curDeltX;
                                robotY += curDeltY;
                                angle += curDeltAngle;




                        } //else stopping, because not to collide!
                        iter--;

                        broadcastPosition();
                        repaint(); //repaint state.

                        ros::spinOnce();
                        rate.sleep();
                }
        }


        void spawnRobot(ros::NodeHandle node) {

                ros::service::waitForService("gazebo/spawn_sdf_model");
            ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
            gazebo_msgs::SpawnModel srv;

            // ifstream fin("/home/user/.gazebo/models/pioneer3at/model.sdf");
            // ifstream fin("/home/user/.gazebo/models/pioneer2dx/model.sdf");
            ifstream fin("/home/upc/.gazebo/models/pioneer2dx/model.sdf");

                string model;
            string buf;
            while(!fin.eof()){
                getline(fin, buf);
                model += buf + "\n";
            }
            srv.request.model_xml = model;
            srv.request.model_name = name;
            geometry_msgs::Pose pose;
            srv.request.initial_pose = pose;
            add_robot.call(srv);

            //state...
            gazeboMsg.model_name = name;
          gazeboMsg.pose.position.x = 0.0;
                gazeboMsg.pose.position.y = 0.0;
                gazeboMsg.pose.position.z = 0.0;
                gazeboMsg.pose.orientation.z = 0.0;
                gazeboMsg.pose.orientation.w = 0.0;

                broadcastPosition();
                repaint();
                sleep(3);
        }


int main(int argc, char **argv) {
        ros::init(argc, argv, "searcher");
        sleep(5);



        name= "pioneer2dx"; //iiii
        ros::NodeHandle node;
        pub=node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        spawnRobot(node);
        driveToTarget(node);
        curDelt = 0.5;
        driveToTarget(node);


        return 0;
}
