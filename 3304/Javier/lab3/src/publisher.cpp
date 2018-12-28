#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <ros/console.h>
#include <string> 
#include <cmath>  
 using namespace std;
static string LOBOT = "LOST";
 class FBot {  
    public:
    string name;
	ros::NodeHandle &nh;
	bool isFollow;
	int id;
	double x, y, z;
 	string nameSearcher;
    ros::Publisher point_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    visualization_msgs::Marker point;
	FBot(ros::NodeHandle &nh, string name) : nh(nh), name(name) {
		isFollow = false;
		id = 1;
		x = y = z = 0;
	}
 	string getName() {
		return name;
	}
 	void handlerVoice(const std_msgs::String& str) {
		if (str.data != "stop") {
			nameSearcher = str.data;
			isFollow = true;
		} else isFollow = false;
		cout << "Lost robot voice (" << str.data << ")" << endl;
	}
 	ros::Subscriber listen() {
		return nh.subscribe("/voice", 100, &FBot::handlerVoice, this);
	}
 	void walk() {
		ros::Rate loop_rate(1);
		while(ros::ok() && !isFollow) {
			x += (rand() % 5 - 2);
			y += (rand() % 5 - 2);
			z = 0.0;
 			broadcastPosition();
			repaint();
 			ros::spinOnce();
			loop_rate.sleep();
		}
 		follow();
	}
 	void initVisualisation() {
		point.header.frame_id = name;
		point.header.stamp = ros::Time::now();
		point.ns = name;
		point.action = visualization_msgs::Marker::ADD;
		point.id = id;
		point.type = visualization_msgs::Marker::CYLINDER;
		point.pose.orientation.w = 0.0;
        point.pose.position.x = 0.0;
		point.pose.position.y = 0.0;
		point.pose.position.z = 0.0;
 		point.color.r = 0.4f;
        point.color.g = 0.0f;
        point.color.b = 0.6f;
		point.color.a = 1.0;
 		point.scale.x = 1.0;
		point.scale.y = 1.0;
		point.scale.z = 1.0;
		
	}
 	void repaint() {
		point.pose.position.x = x;
		point.pose.position.y = y;
		point.pose.position.z = z;
		point_pub.publish(point);
	}
    
 	void follow() {
		std::cout << "The LOBOT Move" << std::endl;
 		tf::TransformListener listener;
		float thresholdX = 0.5f, thresholdY = 0.5f;
 		ros::Rate rate(1);
		while (nh.ok() && isFollow) {
			tf::StampedTransform transform;
			try {
				listener.lookupTransform(name, nameSearcher, ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				ROS_ERROR("%s", e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
				continue;
			}
 			x += transform.getOrigin().x();
			y += transform.getOrigin().y();
 			broadcastPosition();
			repaint();
			
			ros::spinOnce();
			rate.sleep();
		}
	}
 	void broadcastPosition() {
		static tf::TransformBroadcaster br;
		tf::Transform transform;
 		cout << "LoBot (" << name <<") move out x:" << x << " y:" << y << endl;
 		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, z);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}

};
 int main(int argc, char **argv) {
	ros::init(argc, argv, "lobot");
 	ros::NodeHandle nh;
	FBot lost(nh, LOBOT);
	lost.initVisualisation();
	ros::Subscriber sub = lost.listen();
	lost.walk();
 	return 0;
}