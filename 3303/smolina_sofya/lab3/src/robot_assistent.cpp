#include <ros/ros.h>
 #include <visualization_msgs/Marker.h>  
 #include <turtlesim/Pose.h>
 #include <time.h>
 #include <math.h>
 #include <std_msgs/String.h>
 #include <tf/transform_broadcaster.h>
 #include <tf/transform_listener.h>
 
bool isFound = false;
bool isFinish = false;
const int startX = -5;
const int startY = -5;
const float max_step = 0.03;
const float minDiff = 0.1;

void printPoint(visualization_msgs::Marker msg){
	ROS_INFO("%s x=%f y=%f z=%f", msg.ns, msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
}

visualization_msgs::Marker initPoint() {
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/robot_frame";
    msg.header.stamp = ros::Time::now();
    msg.ns = "robot_assistent";
    msg.action = visualization_msgs::Marker::ADD;
    msg.id = 0;
    msg.type = visualization_msgs::Marker::SPHERE;
    msg.scale.x = 0.2;
    msg.scale.y = 0.3;
    msg.scale.z = 0.2;
    msg.pose.position.x = startX;
    msg.pose.position.y = startY;
   
    msg.color.r = 0.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
  
    printPoint(msg);
    return msg;
}

float nextStepForGoal(float curr, float goal, float maxStep) {
	if(fabs(curr - goal) < maxStep){
		curr = goal;
	} else if(goal < 0 && goal - curr > 0){
		curr +=maxStep;
	} else curr += copysignf(max_step, goal - curr);
	return curr;
}

 int main(int argc, char **argv) {
 	ros::init(argc,argv,"robot_assistent");
 	ros::NodeHandle nh;
 	ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("robot_assistent_marker",10,true);
 	ros::Publisher pubLostRobot = nh.advertise<std_msgs::String>("robot_assistent_topic",10);
	ros::Rate r(30);
	
	visualization_msgs::Marker msg = initPoint();
	    
	pub.publish(msg);
	
	tf::TransformBroadcaster transformBroadcast;
	tf::Transform transform;
	tf::TransformListener transformListener;
	tf::StampedTransform stamTransform;
	
	while(ros::ok() && !isFinish){
		
		if (isFound) {
			if(startX - minDiff < msg.pose.position.x && startX + minDiff > msg.pose.position.x
			&& startY - minDiff < msg.pose.position.y && startY + minDiff > msg.pose.position.y){
				isFinish = true;
				std_msgs::String message;
				message.data = "finish";
				pubLostRobot.publish(message);
				ros::spinOnce();
			} else {
				msg.pose.position.x = nextStepForGoal(msg.pose.position.x, startX, max_step);
 				msg.pose.position.y = nextStepForGoal(msg.pose.position.y, startY, max_step);
			}
		} else {
			try {
				transformListener.lookupTransform("/world", "/lost_robot/pose", ros::Time(0), stamTransform);
			}
			catch (tf::TransformException &exeption) {
				ROS_INFO("TransformException");
				ros::Duration(1.0).sleep();
				continue;
			}
			float lostRobotX  = stamTransform.getOrigin().x();
			float lostRobotY = stamTransform.getOrigin().y();
			if(lostRobotX - minDiff < msg.pose.position.x && lostRobotX + minDiff > msg.pose.position.x
			&& lostRobotY - minDiff < msg.pose.position.y && lostRobotY + minDiff > msg.pose.position.y){
				isFound = true;
				std_msgs::String message;
				message.data = "follow me";
				pubLostRobot.publish(message);
				ros::spinOnce();
			} else {

				msg.pose.position.x = nextStepForGoal(msg.pose.position.x, stamTransform.getOrigin().x(), max_step);
 				msg.pose.position.y = nextStepForGoal(msg.pose.position.y, stamTransform.getOrigin().y(), max_step);
			}
			
		}
	    pub.publish(msg);
	    
	    transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion quaternion;
		quaternion.setRPY(0, 0, 0);
		transform.setRotation(quaternion);
		transformBroadcast.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/robot_assistent/pose"));
	    
	    ros::spinOnce();
	    r.sleep();
  	}
}
