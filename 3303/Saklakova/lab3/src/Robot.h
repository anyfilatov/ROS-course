#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h> 
#include <tf/transform_listener.h> 

class Robot {
public:
	Robot(const char* name, const int id, float r, float g, float b){
		robot = name;
		lab3_pub = nh.advertise<visualization_msgs::Marker>("pt_topic", 1);
		
		mrk.header.frame_id = "/my_frame";
		mrk.header.stamp = ros::Time::now(); 
		mrk.ns = "basic_shapes";
		mrk.id = id; 
		mrk.type = visualization_msgs::Marker::SPHERE;
		mrk.action = visualization_msgs::Marker::ADD;
		mrk.pose.position.x = 0;
		mrk.pose.position.y = 0; 
		mrk.pose.position.z = 0;
		mrk.pose.orientation.x = 0.0;
		mrk.pose.orientation.y = 0.0;
		mrk.pose.orientation.z = 0.0;
		mrk.pose.orientation.w = 1.0;
		mrk.scale.x = 1.0; 
		mrk.scale.y = 1.0;
		mrk.scale.z = 1.0;
		
		set_color(r, g, b);
		mrk.color.a = 1.0;

		mrk.lifetime = ros::Duration(); 
		lab3_pub.publish(mrk);
		offset_x = offset_y = 0.0;
	}
	void move(double x, double y){	
		static tf::TransformBroadcaster br; 
		tf::Transform transform;
		ros::Rate loop(30);

		float dx = fabs(getX() - x) / 10.0;
		float dy = fabs(getY() - y) / 10.0;
		dx *= getX() < x ? 1 : -1;
		dy *= getY() < y ? 1 : -1;

		for (float i = 0; i < 10 ; i++){
			mrk.pose.position.x += dx;
			mrk.pose.position.x += dy;
			lab3_pub.publish(mrk);		

			transform.setOrigin(tf::Vector3(getX(), getY(), 0.0)); 
			tf::Quaternion q; 
			q.setRPY(0, 0, 0); 
			transform.setRotation(q);
			br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", robot)); 

			ros::spinOnce();
			loop.sleep();
		}
	}
	bool take_pos(const char* name){
		static tf::TransformListener listener;
		tf::StampedTransform transform; 
		try{
			listener.lookupTransform("/world", name, ros::Time(0), transform);
			
			offset_x = transform.getOrigin().x(); 
			offset_y = transform.getOrigin().y();
			if (fabs(getX() - offset_x) < 1.0 && fabs(getY() - offset_y) < 1.0){
				return true;
			}
		}
		catch (tf::TransformException &ex){
			ROS_ERROR("%s", ex.what());
			ros::Duration(1.0).sleep();
		}
		return false;
	}
	void broadcast_pos(){
		move(offset_x, offset_y);
	}
	float getX(){
		return mrk.pose.position.x;
	}
	float getY(){
		return mrk.pose.position.y;
	}
	void set_color(float r, float g, float b){	
	mrk.color.r = r;
	mrk.color.g = g;
	mrk.color.b = b;
	}
protected:
	const char* robot;
	float offset_x;
	float offset_y;
	ros::NodeHandle nh;
	ros::Publisher lab3_pub; 
	visualization_msgs::Marker mrk; 
};
