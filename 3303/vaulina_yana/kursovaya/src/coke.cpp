#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <string>
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include <cmath> 
#include <tf/transform_listener.h>
 
using namespace std;
ros::Publisher cmd_vel_topic;
ros::Publisher pub;
//tf::TransformListener listener;

float cokes[6][2] = {{-4.7,4.8},{-1.53,-0.79},{0.86,1.68},{-2.25,5.4},{-4.2,-4.2},{-6.7,-5.3}};
float treasure_house[6][2] = {{12.9,-2.4},{13.9,-2.6},{13.7,-3.5},{13.2,-3.6},{12.4,-3},{12.4,-3.6}};
string names[6]={"coke_1","coke_2","coke_3","coke_4","coke_5","coke_6"};
bool coke_picked[6]={false,false,false,false,false,false};
int cokes_in_house = 6;

tf::StampedTransform getCoord(const tf::TransformListener &listener){ 
    tf::StampedTransform transform;
    while(true){
        try{
            listener.waitForTransform("odom","base_footprint",ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("odom","base_footprint",ros::Time(0), transform);
        } 
        catch (tf::TransformException &ex) {
            ROS_ERROR("???%s",ex.what());
            ros::Duration(1.0).sleep();
            sleep(1);
            continue;
         }
        ROS_INFO("HELPER get helper from TF: point x=%f, y=%f, z=%f ",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        ROS_INFO("HELPER get helper from TF: point x=%f, y=%f, z=%f, w=%f ",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
        return transform;
    }
}

float getDistance(tf::Transform q1,float x,float y){
    float x2 = (q1.getOrigin().x() - x) * (q1.getOrigin().x() -x);
    float y2 = (q1.getOrigin().y() -y) * (q1.getOrigin().y() - y);
    float k = (sqrt(x2 + y2));
    ROS_INFO("x=%f, y=%f,  DIST %f",x,y,k);
    return k;
}

void checkCoka(){
    tf::TransformListener listener;
    tf::StampedTransform coord = getCoord(listener);
   for (int i=0;i<6;i++){
        float dist = getDistance(coord,cokes[i][0], cokes[i][1]);
        if(!coke_picked[i] && dist<0.5){
            geometry_msgs::Twist pos;
            pos.linear.x = 0;
            pos.angular.z = 0;
            cmd_vel_topic.publish(pos);
            gazebo_msgs::ModelState msg;
            msg.model_name = names[i];
            msg.pose.position.x = treasure_house[i][0];
            msg.pose.position.y = treasure_house[i][1];
            pub.publish(msg);
            coke_picked[i]=true;
            cokes_in_house--;
            break;
        }

   }
}

void callback(const sensor_msgs::LaserScan & msg){
    if (cokes_in_house==0){
        geometry_msgs::Twist pos;
        pos.linear.x = 0.0;
        pos.angular.z =0.0;
        cmd_vel_topic.publish(pos);
        return;
    }
    checkCoka();
    bool wall_is_near = false;
    for(int i=0;i<30;i++){
        if(msg.ranges[i]<0.5){
            wall_is_near=true;
            break;
        }
        if(msg.ranges[359 - i]<0.5){
            wall_is_near=true;
            break;
        }
    }
    if(!wall_is_near){
        geometry_msgs::Twist pos;
        if(msg.ranges[0]>2){
             pos.linear.x = 0.5;
        }else{
            pos.linear.x = 0.25;
        }
        
        pos.angular.z = 0;
        cmd_vel_topic.publish(pos);
        //sleep(0.1);
    }else{
        geometry_msgs::Twist pos;
        pos.linear.x = 0.0;
        pos.angular.z =- 0.7;
        cmd_vel_topic.publish(pos);
        sleep(0.1);
        
        
    }
  

}
int main(int argc, char** argv) {
    ros::init(argc, argv, "coka");
    ros::NodeHandle node;
    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = node.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
 
    
    gazebo_msgs::SpawnModel srv;
 
    ifstream fin("/home/yana/.gazebo/models/coke_can/model.sdf");
   

    string model;
    string buf;
    while(!fin.eof()){
        getline(fin, buf);
        model += buf + "\n";
    }
    srv.request.model_xml = model;
    geometry_msgs::Pose pose;
    srv.request.initial_pose = pose;
    for(int i=0;i<6;i++){

        
        srv.request.model_name =names[i];
        cout<<srv.request.model_name<<"\n";
        add_robot.call(srv);
    }
    

   
    
 
    pub = node.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    cmd_vel_topic = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    sleep(1.0);
//    ros::Rate loop_rate(1);
    for(int i=0;i<6;i++){
       
        gazebo_msgs::ModelState msg;
        msg.model_name = names[i];
        msg.pose.position.x = cokes[i][0];
        msg.pose.position.y = cokes[i][1];
        pub.publish(msg);
        cout<<msg.model_name<<"\n";
        cout<<msg.pose.position.x<<"\n";
        cout<<msg.pose.position.y<<"\n";
       // loop_rate.sleep();
    }


    ros::Subscriber sub = node.subscribe("/scan",30,callback);
   
   // sleep(1.0);
    ros::spin();
    return 0;
}