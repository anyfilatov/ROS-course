#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"

#include <robot_msg/robotMesg.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>
#include <cstdlib>
#include <ctime>

using namespace std;



string name;
bool isFollow=false;
int id;
double x, y, z;
double angle = 0;
double robotX=0;
double robotY=0;
double goalX, goalY;
double curDeltX=0;
double curDeltY=0;
double curDelt=0.5;
double curDeltAngle;
string target="null";
string nameSearcher;


ros::Publisher pub;
gazebo_msgs::ModelState robotState;


        void redrawRobot() {
            robotState.pose.position.x = x;
            robotState.pose.position.y = y;
            robotState.pose.orientation.z = sin(angle / 2);
            robotState.pose.orientation.w = cos(angle / 2);
            pub.publish(robotState);
        }

        void broadcastPosition() {

                static tf::TransformBroadcaster br;
                tf::Transform transform;
                transform.setOrigin(tf::Vector3(x, y, 0.0));
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform.setRotation(q);
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
        }


        void chatterCallback(const robot_msg::robotMesg& msg) {
                target = msg.a;
                if (target != "null") {
                        isFollow = true;
                } else isFollow = false;
                ROS_INFO("Message is " );
        }

        double getCurrentDistance() {
                double diffX = goalX - x;
                double diffY = goalY - y;
                return sqrt(diffX * diffX + diffY * diffY);
        }

        void walk() {
                ros::Rate loop_rate(100);
                int iter = 0;
                while(ros::ok() && !isFollow) {
                        if (iter == 0) {
                                iter = 1000;
                                goalX += (double) (rand() % 10 - 5);
                                goalY += (double) (rand() % 10 - 5);

                                double dx = goalX - x;
                                double dy = goalY - y;
                                curDeltX = dx / iter;
                                curDeltY = dy / iter;
                                curDeltAngle = (-angle + atan2(dy, dx)) / iter;
                        }

                        x += curDeltX;
                        y += curDeltY;
                        angle += curDeltAngle;
                        iter--;

                        broadcastPosition();
                        redrawRobot();

                        ros::spinOnce();
                        loop_rate.sleep();
                }
        }

        void goTotarget(ros::NodeHandle node) {
                std::cout << "Begin following" << std::endl;

                tf::TransformListener listener;
                double dx, dy, da;
                int iter = 0, step = 10;
                ros::Rate rate(100);
                while (node.ok() && isFollow) {
                        tf::StampedTransform transform;
                        try {
                                listener.lookupTransform(name, target, ros::Time(0), transform);
                        }
                        catch (tf::TransformException &e) {
                                ROS_ERROR("%s", e.what());
                                broadcastPosition();
                                ros::Duration(1.0).sleep();
                                continue;
                        }

                        if (iter == 0) {
                                iter = 1000;
                                dx = transform.getOrigin().x() / iter;
                                dy = transform.getOrigin().y() / iter;
                                da = atan2(dy, dx) / iter;
                        }


                        if (transform.getOrigin().length() > curDelt) {
                                x += dx;
                                y += dy;
                                angle += da;
                        }

                        iter--;

                        broadcastPosition();
                        redrawRobot();

                        ros::spinOnce();
                        rate.sleep();
                }
        }


        void spawnRobot(ros::NodeHandle node) {

            ros::service::waitForService("gazebo/spawn_sdf_model");
            ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
            gazebo_msgs::SpawnModel srv;

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


           robotState.model_name = name;
           robotState.pose.position.x = 0.0;
           robotState.pose.position.y = 0.0;
           robotState.pose.position.z = 0.0;
           robotState.pose.orientation.z = 0.0;
           robotState.pose.orientation.w = 0.0;
        }





int main(int argc, char **argv) {
     ROS_INFO("INFO0");
        ros::init(argc, argv, "walker");

        ros::NodeHandle node;
        ROS_INFO("INFO1");
        pub = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
        name="pioneer2d_1";
        sleep(1.0);
        spawnRobot(node);
        ros::Subscriber sub =node.subscribe("/chatter", 100, &chatterCallback);;
        walk();
        goTotarget(node);

	return 0;
}
