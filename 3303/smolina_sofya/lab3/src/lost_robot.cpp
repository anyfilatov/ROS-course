#include <ros/ros.h>
 #include <visualization_msgs/Marker.h>  
 #include <turtlesim/Pose.h>
 #include <time.h>
 #include <math.h>
 #include <std_msgs/String.h>
 #include <tf/transform_broadcaster.h>
 #include <tf/transform_listener.h>

 const int minX = -5;
 const int maxX = 5;
 const int minY = -5;
 const int maxY = 5;
 bool isFound = false;
 bool isFinish = false;
const float maxStep = 0.03;
const int startX = 0;
const int startY = 0;
 
 void handler(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("message: %s", msg->data);
	isFound = msg->data == "follow me";
	isFinish = msg->data == "finish";
 }

void printPoint(visualization_msgs::Marker msg){
	ROS_INFO("%s x=%f y=%f z=%f", msg.ns, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

visualization_msgs::Marker initPoint() {
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/robot_frame";
    msg.header.stamp = ros::Time::now();
    msg.ns = "lost_robot";
    msg.action = visualization_msgs::Marker::ADD;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.scale.x = 0.2;
    msg.scale.y = 0.3;
    msg.scale.z = 0.2;
    msg.pose.position.x = startX;
    msg.pose.position.y = startY;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
  
    printPoint(msg);
    return msg;
}

float nextRandStep(float curr, float max, float min, float maxStep) {
	curr += pow(-1, rand() % 30) * maxStep;
	if(curr > max){
		curr = max;
	} else if(curr < min){
		curr = min;
	}
	return curr;
}

float nextStepForHelper(float curr, float goal, float maxStep) {
	if(fabs(curr - goal) < maxStep){
		curr = goal;
	} else if(goal < 0 && goal - curr > 0){
		curr +=maxStep;
	} else curr += copysignf(maxStep, goal - curr);
	return curr;
}


 int main(int argc, char **argv) {
    ros::init(argc,argv,"lost_robot");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<visualization_msgs::Marker>("lost_robot_marker", 10, true);
    ros::Subscriber sub = n.subscribe("robot_assistent_topic", 10, handler);
    ros::Rate r(30);
	
    visualization_msgs::Marker msg = initPoint();
    pub.publish(msg);
        tf::TransformBroadcaster transformBroadcaster;
	tf::Transform transform;
	tf::TransformListener transformListener;
	tf::StampedTransform stamTransform;    
	
	while(ros::ok() && !isFinish){
		
	    if (!isFound) {
                msg.pose.position.x = nextRandStep(msg.pose.position.x, maxX, minX, maxStep);
                msg.pose.position.y = nextRandStep(msg.pose.position.y, maxY, minY, maxStep);
                        
	     } else {
		    try {
		            transformListener.lookupTransform("/world", "/robot_assistent/pose", ros::Time(0), stamTransform);
			}
			catch (tf::TransformException &exeption) {
				ROS_INFO("TransformException");
				ros::Duration(1.0).sleep();
				continue;
			}
                     msg.pose.position.x = nextStepForHelper(msg.pose.position.x, stamTransform.getOrigin().x(), maxStep);
		     msg.pose.position.y = nextStepForHelper(msg.pose.position.y, stamTransform.getOrigin().y(), maxStep);
		}
	    printPoint(msg);
	    pub.publish(msg);
	    
	    transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
	    tf::Quaternion quaternion;
	    quaternion.setRPY(0, 0, 0);
	    transform.setRotation(quaternion);
	    transformBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/lost_robot/pose"));
	    
	    ros::spinOnce();
	    r.sleep();
  	}
}
