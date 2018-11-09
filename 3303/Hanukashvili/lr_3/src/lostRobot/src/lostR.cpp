#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>


const int lGr = -4.5;
const int rGr = 4.5;
const int changePos = 10;
const float minDX = 0.5; 
const float minDY = 0.5;
bool isFound = false;
bool isFinish = false;

void getStatus(const std_msgs::String::ConstPtr& msg) {

	if (msg->data == "302")	isFound = true;
	else isFound = false;

	if (msg->data == "200")	
		{
			isFinish = true;
			isFound = true;
		}
	else {
		isFinish = false;

	}
}

int main(int argc, char ** argv)
{

 	ros::init(argc,argv,"lostR");
 	ros::NodeHandle n;
 	ros::Publisher pub = n.advertise<visualization_msgs::Marker>("lostR_t", 10 ,true);
 	ros::Subscriber getStatusInfo = n.subscribe("getStatusInfo_t", 10, getStatus);
	ros::Rate r(30);
	//
	visualization_msgs::Marker msg;
	msg.header.frame_id = "/robots_frame";
	msg.header.stamp = ros::Time::now();

   	msg.ns = "lost_robot";
    	msg.action = visualization_msgs::Marker::ADD;
        // Generation of random robot position
    	msg.pose.position.x = -5 + rand() % 10;
    	msg.pose.position.y = -5 + rand() % 10;
    	msg.pose.position.z = 0.5;    
    	msg.id = 0;
	//Draw robot
   	msg.type = visualization_msgs::Marker::CUBE;
    
    	msg.scale.x = 1.0;
    	msg.scale.y = 1.0;
    	msg.scale.z = 1.0;
    
    	msg.color.r = 0.0f;
    	msg.color.g = 1.0f;
    	msg.color.b = 0.50f;
    	msg.color.a = 1.0f;
	    
	pub.publish(msg);


	tf::TransformBroadcaster tBr;
	tf::Transform transform;
	tf::TransformListener tLst;
	tf::StampedTransform stamTransform;

	while(ros::ok()){

		if(isFound)
		{

			try {
				tLst.lookupTransform("/world", "/searchR/pose", ros::Time(0), stamTransform);
			}
			catch (tf::TransformException &exeption) {
				ros::Duration(1.0).sleep();
				continue;
			}
			float dX =  msg.pose.position.x - stamTransform.getOrigin().x();
			float dY = msg.pose.position.y - stamTransform.getOrigin().y();
			
			if(isFinish) {
				
				    
			    	msg.color.r = 1.0f;
			    	msg.color.g = 0.0f;
			    	msg.color.b = 0.0f;
			    	msg.color.a = 1.0f;

			}

			if(!isFinish) {

				if(dX >= minDX  ) {

					msg.pose.position.x = msg.pose.position.x - 0.1;
				}

				if( dX <= minDX && dX<=0  ) {
					
					msg.pose.position.x = msg.pose.position.x + 0.1;
				}

				if (dY <= minDY && dY <=0 ) {

	    				msg.pose.position.y = msg.pose.position.y + 0.1;
		    		}

				if (dY >= minDY  ) {
					
	    				msg.pose.position.y = msg.pose.position.y -0.1;
			    	}

			}
		}
			


		if(!isFound){
			if(msg.pose.position.x <= lGr || msg.pose.position.y <= lGr ) {
				if(msg.pose.position.x <= lGr ) {
					msg.pose.position.x = msg.pose.position.x + rand() % changePos;
	    				msg.pose.position.y = msg.pose.position.y + 0.1;

				}
				if(msg.pose.position.y <= lGr ) {
					msg.pose.position.x = msg.pose.position.x - 0.1;
	    				msg.pose.position.y = msg.pose.position.y + rand() % changePos;
				}

			}
			if(msg.pose.position.x >= rGr || msg.pose.position.y >= rGr ) {
				if(msg.pose.position.x >= rGr ) {
					msg.pose.position.x = msg.pose.position.x - rand() % changePos;
	    				msg.pose.position.y = msg.pose.position.y + 0.1;
				}
				if(msg.pose.position.y >= rGr ) {
					msg.pose.position.x = msg.pose.position.x - 0.1;
	    				msg.pose.position.y = msg.pose.position.y - rand() % changePos;

				}

			}
			else {
			msg.pose.position.x = msg.pose.position.x + 0.01;
    			msg.pose.position.y = msg.pose.position.y - 0.01;
    			msg.pose.position.z = 0.5;    
			}
		}
 		pub.publish(msg);
	    
	    	transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion quaternion;
		quaternion.setRPY(0, 0, 0);
		transform.setRotation(quaternion);
		tBr.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/lostR/pose"));
	    
	   	ros::spinOnce();
		r.sleep();		

  	}
}
