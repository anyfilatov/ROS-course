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
static string REBOT = "RESCUE";
class SBot {
    public:
    string name;
	ros::NodeHandle &nh;
	int id;
	double x, y, z;
 	string nameWalker;
	ros::Publisher point_pub;
	visualization_msgs::Marker point;
	SBot(ros::NodeHandle &nh, string name, string nameWalker) : nh(nh), name(name), nameWalker(nameWalker) {
		id = 2;
		x = y = z = 0;
	}
 	void search() {
		move(name, nameWalker);
		cry(name);	
	}
 	void comeback() {
		move(name, "world");
		cry("stop");
	}
 	void initVisualisation() {
		cout << "init visualization" << endl;
 		point_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		point.header.frame_id = name;
		point.header.stamp = ros::Time::now();
		point.ns = name;
		point.action = visualization_msgs::Marker::ADD;
		point.id = id;
		point.type = visualization_msgs::Marker::CYLINDER;
        point.pose.position.x = 0.0;
		point.pose.position.y = 0.0;
		point.pose.position.z = 0.0;
 		point.color.g = 1.0f;
        point.color.r = 1.0f;
		point.color.a = 1.0;
 		point.scale.x = 1.0;
		point.scale.y = 1.0;
		point.scale.z = 1.0;
		
		
 		broadcastPosition();
		repaint();
		sleep(3);
	}
 	void move(const std::string& targetFrame, const std::string& sourceFrame) {
		std::cout << "Rescuer (" << name << ") start to rescue (" << sourceFrame << ")" << std::endl;
 		tf::TransformListener listener;
		float thresholdX = 0.5f, thresholdY = 0.5f;
 		ros::Rate rate(1);
		while (nh.ok()) {
			tf::StampedTransform transform;
			try {
				listener.lookupTransform(targetFrame, sourceFrame, ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				ROS_ERROR("%s", e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
				continue;
			}
 			x += sqrt(pow(transform.getOrigin().x(), 2) + pow(transform.getOrigin().x(), 2)) / 2.0 * (transform.getOrigin().x() < 0) ? -1.0 : 1.0;
			y += sqrt(pow(transform.getOrigin().y(), 2) + pow(transform.getOrigin().y(), 2)) / 2.0 * (transform.getOrigin().y() < 0) ? -1.0 : 1.0;
 			cout << "Absolute robot position(" << fabs(transform.getOrigin().x()) << ", " << fabs(transform.getOrigin().y()) << ")" << endl;		
			if (fabs(transform.getOrigin().x()) < thresholdX 
				&& fabs(transform.getOrigin().y()) < thresholdY)
				break;
 			broadcastPosition();
			repaint();
			
			rate.sleep();
		}
	}
 	void cry(const std::string& msg) {
		std_msgs::String s;
		s.data = msg;
	  	ros::Publisher pub = nh.advertise<std_msgs::String>("/voice", 100);
	 
	 	cout << "Cry message: (" << msg << ")" << endl;
	  	ros::Rate loop_rate(1);
	  	for (int i = 0; i < 3; i++) {
	        pub.publish(s);
	        loop_rate.sleep();
	    }
	}
 	void broadcastPosition() {
		static tf::TransformBroadcaster br;
		tf::Transform transform;
 		cout << "Rescue robot (" << name <<") move out x:" << x << " y:" << y << endl;
 		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, z);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}
 	void repaint() {
		point.pose.position.x = x;
		point.pose.position.y = y;
		point.pose.position.z = z;
		point_pub.publish(point);
	}
 
 };
 int main(int argc, char **argv) {
	ros::init(argc, argv, "rebot");
 	ros::NodeHandle nh;
	SBot rescue(nh, REBOT, LOBOT);
	rescue.initVisualisation();
	rescue.search();
	rescue.comeback();
 	return 0;
}

