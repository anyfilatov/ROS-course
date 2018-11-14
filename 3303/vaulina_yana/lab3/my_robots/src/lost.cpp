
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher rviz_pub;
float step_size = 0.3;

float x_cur;
float y_cur;
float z_cur;

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
  point.id = 0;
  point.type = visualization_msgs::Marker::POINTS;
  point.scale.x = 0.5;
  point.scale.y = 0.5;
  point.color.r = 1.0;
  point.color.g = 0.0;
  point.color.b = 0.0;
  point.color.a = 1.0;
  geometry_msgs::Point p;
  p.x = x_cur;
  p.y = y_cur;
  p.z = z_cur;
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
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "lost"));
  ROS_INFO("LOST sent to TF: point x=%f, y=%f, z=%f ",x_cur,y_cur,z_cur);
}
tf::StampedTransform getHelperCoord(const tf::TransformListener &listener){ 
	tf::StampedTransform transform;
  	while(true){
	    try{
	        listener.waitForTransform("world","helper",ros::Time(0), ros::Duration(1.0));
	        listener.lookupTransform("world", "helper",ros::Time(0), transform);
		} 
		catch (tf::TransformException &ex) {
		    ROS_ERROR("!!!%s",ex.what());
		    ros::Duration(1.0).sleep();
		    sleep(1);
		    continue;
		 }
	    
	    ROS_INFO("LOST get from TF: point x=%f, y=%f, z=%f ",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
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
    ROS_INFO("LOST made a step: point x=%f, y=%f, z=%f",x_cur,x_cur,x_cur);
}
int main(int argc, char** argv){
	ros::init(argc, argv, "lost");
	ros::NodeHandle node;
	rviz_pub = node.advertise<visualization_msgs::Marker>("pt_topic",10,true);
	tf::TransformListener listener;
	
	setPoint(1.0,1.0,1.0);
	rviz_pub.publish(createPoint());
	ROS_INFO("LOST sent to RVIZ: point x=%f, y=%f, z=%f ",x_cur,x_cur,x_cur);
	sendPositionToTf();
	sleep(1);
	ros::Rate rate(10);
	while(true){
		sendPositionToTf();
		tf::StampedTransform helper = getHelperCoord(listener);
		float distance = getDistance(helper.getOrigin().x(),helper.getOrigin().y(),helper.getOrigin().z());
		if(distance < step_size){
			break;
		}
		
		rate.sleep();
	}
	ROS_INFO("LOST hurah!!!");
	sendPositionToTf();
	sleep(1);
	while(true){
		tf::StampedTransform helper = getHelperCoord(listener);
		float distance = getDistance(helper.getOrigin().x(),helper.getOrigin().y(),helper.getOrigin().z());
		if(distance < step_size){
			break;
		}
		makeStep(helper.getOrigin().x(),helper.getOrigin().y(),helper.getOrigin().z(),distance);
		rviz_pub.publish(createPoint());
		ROS_INFO("LOST sent to RVIZ: point x=%f, y=%f, z=%f ",x_cur,x_cur,x_cur);
		sendPositionToTf();
		rate.sleep();
	}
  
	ROS_INFO("LOST IS HOME");  
  

 
  ros::spinOnce();
  return 0;
}