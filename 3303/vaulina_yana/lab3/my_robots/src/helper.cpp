
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher rviz_pub;

float x_cur;
float y_cur;
float z_cur;

float home_x = 10.0;
float home_y = 10.0;
float home_z = 10.0;

float step_size = 0.3;

void setPoint(float x,float y,float z){
	x_cur = x;
 	y_cur = y;
 	z_cur = z;
}

visualization_msgs::Marker createPoint(){
  visualization_msgs::Marker point;
  point.header.frame_id = "/point_on_map";
  point.header.stamp = ros::Time::now();
  point.ns = "there_is_point";
  point.action = visualization_msgs::Marker::ADD;
  point.pose.orientation.w = 1;
  point.id = 1;
  point.type = visualization_msgs::Marker::POINTS;
  point.scale.x = 0.5;
  point.scale.y = 0.5;
  point.color.r = 0.0;
  point.color.g = 1.0;
  point.color.b = 0.0;
  point.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = x_cur;
  p.y = y_cur;
  p.z = z_cur;
  point.points.push_back(p);
  return point;

}
visualization_msgs::Marker createHome(){
  visualization_msgs::Marker point;
  point.header.frame_id = "/point_on_map";
  point.header.stamp = ros::Time::now();
  point.ns = "there_is_point";
  point.action = visualization_msgs::Marker::ADD;
  point.pose.orientation.w = 1;
  point.id = 2;
  point.type = visualization_msgs::Marker::POINTS;
  point.scale.x = 0.7;
  point.scale.y = 0.7;
  point.color.r = 1.0;
  point.color.g = 1.0;
  point.color.b = 1.0;
  point.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = home_x;
  p.y = home_y;
  p.z = home_z;
  point.points.push_back(p);
  return point;

}

void sendPositionToTf(){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(x_cur, y_cur, z_cur) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "helper"));
  ROS_INFO("HELPER sent to TF: point x=%f, y=%f, z=%f ",x_cur,y_cur,z_cur);
}
tf::StampedTransform getLostCoord(const tf::TransformListener &listener){ 
	tf::StampedTransform transform;
  	while(true){
	    try{
	        listener.waitForTransform("world","lost",ros::Time(0), ros::Duration(1.0));
	        listener.lookupTransform("world", "lost",ros::Time(0), transform);
		} 
		catch (tf::TransformException &ex) {
		    ROS_ERROR("???%s",ex.what());
		    ros::Duration(1.0).sleep();
		    sleep(1);
		    continue;
		 }
	    
	    ROS_INFO("HELPER get from TF: point x=%f, y=%f, z=%f ",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
		return transform;
	}
}
float getDistance(float x,float y,float z){
    float x2 = (x - x_cur) * (x - x_cur);
    float y2 = (y - y_cur) * (y - y_cur);
    float z2 = (z - z_cur) * (z - z_cur);
    float k = (sqrt(x2 + y2 + z2));
    return k;
}


void makeStep(float x,float y,float z,float d){
    
    x_cur += step_size / d * (x - x_cur);
    y_cur += step_size / d * (y - y_cur);
    z_cur += step_size / d * (z - z_cur);
    ROS_INFO("HELPER made a step: point x=%f, y=%f, z=%f",x_cur,x_cur,x_cur);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "helper");
  ros::NodeHandle node;
  rviz_pub = node.advertise<visualization_msgs::Marker>("pt_topic",10,true);
  tf::TransformListener listener;
  setPoint(home_x,home_y,home_z);
 
  rviz_pub.publish(createPoint());
  ROS_INFO("HELPER sent to RVIZ: point x=%f, y=%f, z=%f ",x_cur,x_cur,x_cur);
   rviz_pub.publish(createHome());
  ROS_INFO("HOME sent to RVIZ: point x=%f, y=%f, z=%f ",home_x,home_y,home_z);
  sleep(1);
 
 
  sendPositionToTf();
  
  
  ros::Rate rate(10);
	while(true){
		tf::StampedTransform lost = getLostCoord(listener);
		float distance = getDistance(lost.getOrigin().x(),lost.getOrigin().y(),lost.getOrigin().z());
		if(distance < step_size){
			sendPositionToTf();
			break;
		}
		makeStep(lost.getOrigin().x(),lost.getOrigin().y(),lost.getOrigin().z(),distance);
		rviz_pub.publish(createPoint());
		ROS_INFO("HELPER sent to RVIZ: point x=%f, y=%f, z=%f ",x_cur,x_cur,x_cur);
		sendPositionToTf();
		rate.sleep();
	}
	ROS_INFO("HELPER hurah!!!");
	sendPositionToTf();
  	while(true){
		sendPositionToTf();
		float distance = getDistance(home_x,home_y,home_z);
		if(distance < step_size){
			break;
		}
		makeStep(home_x,home_y,home_z,distance);
		rviz_pub.publish(createPoint());
		ROS_INFO("HELPER sent to RVIZ: point x=%f, y=%f, z=%f ",x_cur,x_cur,x_cur);
		rate.sleep();
	}
	ROS_INFO("HELPER IS HOME"); 

 
  ros::spinOnce();
  return 0;
}