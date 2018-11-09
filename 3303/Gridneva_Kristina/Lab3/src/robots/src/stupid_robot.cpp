//Нода для потерявшегося робота. Робот стоит на месте и отправляет свои координаты в tf. После того, как его обнаружит другой робот, начинает движение вслед за ним.
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>


visualization_msgs::Marker setMarker(float x, float y, float z){ // публикация маркера в rviz
	visualization_msgs::Marker marker;
	// Название фрейма для маркера.
	marker.header.frame_id = "/my_map";
	marker.header.stamp = ros::Time::now();
	// namespace и id маркера.
	marker.ns = "robots";
	marker.id = 0;
	// Тип маркера
	marker.type = visualization_msgs::Marker::POINTS;
 	// Действие для маркера
	marker.action = visualization_msgs::Marker::ADD;
	// Глобальные координаты для сообщения
	marker.pose.position.x = 0;
 	marker.pose.position.y = 0;
       	marker.pose.position.z = 0;
       	marker.pose.orientation.x = 0.0;
       	marker.pose.orientation.y = 0.0;
       	marker.pose.orientation.z = 0.0;
       	marker.pose.orientation.w = 1.0;
	// Размер маркера
	marker.scale.x = 1.0;
       	marker.scale.y = 1.0;
       	marker.scale.z = 1.0;
	// Цвет маркера
	marker.color.r = 0.0;
       	marker.color.g = 1.0;
       	marker.color.b = 0.0;
       	marker.color.a = 1.0;
	// Определение точки
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	p.z = z;
	marker.points.push_back(p);
	return marker;
}

tf::StampedTransform transformPoint(const tf::TransformListener &listener){ // прослушивание tf для определения координат движения
	tf::StampedTransform transform;
	bool success = false;
	do
	{
		try{	
		listener.lookupTransform("world2", "smart_robot", ros::Time(0), transform);
		ROS_INFO("origin: %.2f, %.2f, %.2f", transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
		success = true;
		}
		catch(tf::TransformException &ex){
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
		}
		sleep(1);
	}
	while(!success);
	
	return transform;
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "stupid_rob");
	ros::NodeHandle n;
	ros::Rate r(100);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("st_topic", 10, true);
	tf::TransformBroadcaster broadcaster;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	float start_x = -16;
	float start_y = -16;
	float start_z = 1;
	bool flag = false;
	float cur_x = start_x;
	float cur_y = start_y;
	float cur_z = start_z;
	marker_pub.publish(setMarker(start_x, start_y, start_z));
	broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world", "stupid_robot"));
	ROS_INFO("-----------START----------");
	sleep(1);
	do
	{
		marker_pub.publish(setMarker(start_x, start_y, start_z));
		// вещание в tf своих координат
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world", "stupid_robot"));
		ROS_INFO("HELP! My coordinates: %.2f, %.2f, %.2f", cur_x, cur_y, cur_z);
		transform = transformPoint(listener);
		
		r.sleep();
	}
	while((transform.getOrigin().x() != start_x) && (transform.getOrigin().y() != start_y));
	ROS_INFO("He finds me!");
	do
	{
		
		transform = transformPoint(listener);
		float cur_x = transform.getOrigin().x();
		float cur_y = transform.getOrigin().y();
		float cur_z = transform.getOrigin().z();
		marker_pub.publish(setMarker(cur_x, cur_y, cur_z));
		// вещание в tf своих координат движения	
		broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0,0,0,1), tf::Vector3(start_x, start_y, start_z)), ros::Time::now(), "world", "stupid_robot"));
		ROS_INFO("My coordinates: %.2f, %.2f, %.2f", cur_x, cur_y, cur_z);
		
		
		r.sleep();
	}
	while(n.ok());
	ros::spinOnce();
	return 0;
}
