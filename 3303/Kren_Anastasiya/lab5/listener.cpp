#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/Empty.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <cmath> 

ros::Publisher cmd_vel_topic;
tf::Transform home;
float prev_x;
float prev_y;

void setHomeCoord(float x,float y){
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(x, y, 0) );
    tf::Quaternion q;
    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    home = transform;
}

void setPrev(tf::StampedTransform q1){
	prev_x = q1.getOrigin().x();
	prev_y = q1.getOrigin().y();
}



float calculateRotation(tf::Transform q_to,tf::Transform q_from){
    float x_cur = q_to.getOrigin().x() - q_from.getOrigin().x();
    float y_cur = q_to.getOrigin().y() - q_from.getOrigin().y();
    float a = sqrt(x_cur*x_cur + y_cur*y_cur);
    float alfa = acos(x_cur / a);
    if(y_cur <0){alfa = -alfa;}
    ROS_INFO("LOST x_cur= %f  y_cur= %f",x_cur, y_cur);
    ROS_INFO("LOST calculateRotation: %f",alfa);
    return alfa;
}


float toEulerAngle(tf::Transform q)
{
   double siny_cosp = +2.0 * (q.getRotation().w() * q.getRotation().z() + q.getRotation().x() * q.getRotation().y());
   double cosy_cosp = +1.0 - 2.0 * (q.getRotation().y() * q.getRotation().y() + q.getRotation().z() * q.getRotation().z());  
   float yaw = atan2(siny_cosp, cosy_cosp);
   ROS_INFO("LOST toEulerAngle: %f",yaw);
   return yaw;
}

tf::StampedTransform getLostCoord(const tf::TransformListener &listener){ 
    tf::StampedTransform transform;
    while(true){
        try{
            listener.waitForTransform("odom","robot2",ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("odom","robot2",ros::Time(0), transform);
        } 
        catch (tf::TransformException &ex) {
            ROS_ERROR("???%s",ex.what());
            ros::Duration(1.0).sleep();
            sleep(1);
            continue;
        }
        //ROS_INFO("LOST get lost from TF: point x=%f, y=%f, z=%f ",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
       // ROS_INFO("LOST get lost from TF: point x=%f, y=%f, z=%f, w=%f ",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
        return transform;
    }
}
tf::StampedTransform getHelperCoord(const tf::TransformListener &listener){ 
    tf::StampedTransform transform;
    while(true){
        try{
            listener.waitForTransform("odom","robot1",ros::Time(0), ros::Duration(1.0));
            listener.lookupTransform("odom","robot1",ros::Time(0), transform);
        } 
        catch (tf::TransformException &ex) {
            ROS_ERROR("???%s",ex.what());
            ros::Duration(1.0).sleep();
            sleep(1);
            continue;
         }
        //ROS_INFO("LOST get helper from TF: point x=%f, y=%f, z=%f ",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
        //ROS_INFO("LOST get helper from TF: point x=%f, y=%f, z=%f, w=%f ",transform.getRotation().x(),transform.getRotation().y(),transform.getRotation().z(),transform.getRotation().w());
        return transform;
    }
}

float getDistance(tf::Transform q1,tf::Transform q2){
    float x2 = (q1.getOrigin().x() - q2.getOrigin().x()) * (q1.getOrigin().x() - q2.getOrigin().x());
    float y2 = (q1.getOrigin().y() - q2.getOrigin().y()) * (q1.getOrigin().y() - q2.getOrigin().y());
    float k = (sqrt(x2 + y2));
    return k;
}

float getAlfa(tf::Transform q_to,tf::Transform q_from){
    float a = calculateRotation(q_to,q_from) - toEulerAngle(q_from);
    if(a>M_PI){a -= 2*M_PI;}
    if(a< - M_PI){a += 2* M_PI;}
    return a;
}



void goHome(const std_msgs::Empty & empty){
	ROS_INFO("I WAS FOUND");
  	 
	geometry_msgs::Twist stop;
	geometry_msgs::Twist pos;
    pos.linear.x = 1;
    tf::TransformListener listener;
    setPrev(getHelperCoord(listener));
    sleep(2.0);
    ros::Rate loop_rate(1);
   	while(true){
        tf::StampedTransform lost = getLostCoord(listener);
        tf::StampedTransform helper = getHelperCoord(listener);
        if (getDistance(lost,home) < 1){
            break;
        }
        /*
        if (getDistance(helper,lost) < 1){
            continue;
        }
        */
        setPrev(helper);
 		pos.angular.z = getAlfa(helper,lost);
		ROS_INFO("Move to position:\n1) pos.linear: x=%f y=%f z=%f\n2) pos.angular: x=%f y=%f z=%f\n",
                    pos.linear.x,pos.linear.y,pos.linear.z,pos.angular.x,pos.angular.y,pos.angular.z);
		cmd_vel_topic.publish(pos);
        loop_rate.sleep();
    }
    cmd_vel_topic.publish(stop);
    ROS_INFO("!!!I'm home!!!\n");
    sleep(1);
    ros::shutdown();
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    cmd_vel_topic =n.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1000);
    setHomeCoord(0,0);
    ros::Subscriber sub = n.subscribe("isFound",30,goHome);
	ros::spin();
    return 0;
}