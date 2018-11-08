 #include <ros/ros.h>
 #include <visualization_msgs/Marker.h>  
 #include <turtlesim/Pose.h>
 #include <time.h>
 #include <math.h>
 #include <std_msgs/String.h>
 #include <tf/transform_broadcaster.h>
 #include <tf/transform_listener.h>
 


bool isFound = false;
const float minDX = 0.7; 
const float minDY = 0.7;
const float minFinDX = 0.1; 
const float minFinDY = 0.1;
const float lGr = -4.5;
const float rGr = 4.5;
const int changePos = 10;


 int main(int argc, char **argv) {
	
 	ros::init(argc,argv,"searchR");
 	ros::NodeHandle n;
 	ros::Publisher pub = n.advertise<visualization_msgs::Marker>("searchR_t",10,true);
	ros::Publisher pubFin = n.advertise<visualization_msgs::Marker>("finishS_t",10,true);

 	ros::Publisher informLostRobot = n.advertise<std_msgs::String>("getStatusInfo_t",10);
	ros::Rate r(30);
	
	visualization_msgs::Marker msg , finishLine;
    	msg.header.frame_id = finishLine.header.frame_id ="/robots_frame";
    	msg.header.stamp = finishLine.header.stamp = ros::Time::now();
    	msg.ns = finishLine.ns = "looking_robot";
    	msg.action = finishLine.action = visualization_msgs::Marker::ADD;
    
       
	// Generation of random robot position
    	msg.pose.position.x = -5 + rand() % 10;
    	msg.pose.position.y = -5 + rand() % 10;
    	msg.pose.position.z = 0.5;    
    	msg.id = 1;
	finishLine.id = 2;
	//Draw robot
   	msg.type = visualization_msgs::Marker::CUBE;
	//Draw finish line
    	finishLine.type = visualization_msgs::Marker::LINE_LIST;

    	msg.scale.x = 1.0;
    	msg.scale.y = 1.0;
    	msg.scale.z = 1.0;
    
	finishLine.scale.x = 0.1;

    	msg.color.r = 0.0f;
    	msg.color.g = 1.0f;
    	msg.color.b = 0.50f;
    	msg.color.a = 1.0f;
	    
        geometry_msgs::Point p;
        p.x = 1;
        p.y = 5;
        p.z = 0.5;  
        // The line list needs two points for each line
        finishLine.points.push_back(p);
        p.x += 2.0;
        finishLine.points.push_back(p);

    	finishLine.color.r = 1.0f;
    	finishLine.color.g = 0.0f;
    	finishLine.color.b = 0.50f;
    	finishLine.color.a = 1.0f;

	pub.publish(msg);
	pubFin.publish(finishLine);
	
	tf::TransformBroadcaster tBr;
	tf::Transform transform;
	tf::TransformListener tLst;
	tf::StampedTransform stamTransform;
	
	while(ros::ok()){
		
		if (isFound)
		{
			    	msg.color.r = 0.0f;
			    	msg.color.g = 0.0f;
			    	msg.color.b = 1.0f;
			    	msg.color.a = 1.0f;
				pub.publish(msg);
				float findX = msg.pose.position.x - p.x;
				float findY = msg.pose.position.y - p.y;
			
				if(msg.pose.position.x >= (p.x-2) && msg.pose.position.x <= (p.x-0.1) ) {
					if(findY <= minDY && findY >=0) {
						
					    	finishLine.color.r = 0.0f;
					    	finishLine.color.g = 0.0f;
					    	finishLine.color.b = 1.0f;
					    	finishLine.color.a = 1.0f;
						pubFin.publish(finishLine);

						std_msgs::String status;
						status.data = "200";
						informLostRobot.publish(status);
						//ros::spinOnce();

					}
					if (findY <= minFinDY && findY <=0 ) {

		    				msg.pose.position.y = msg.pose.position.y + 0.01;
		    				
					}
					if (findY >= minFinDY  ) {
						
		    				msg.pose.position.y = msg.pose.position.y -0.01;
		    				
					}

				}
			
				if(msg.pose.position.x <= (p.x-2)   ) {

					msg.pose.position.x = msg.pose.position.x + 0.01;
					
				}
				if( msg.pose.position.x >= p.x ) {
					
					msg.pose.position.x = msg.pose.position.x - 0.01;
				}

				if (findY <= minFinDY && findY <=0 ) {

		    				msg.pose.position.y = msg.pose.position.y + 0.01;
		    				
					}
				if (findY >= minFinDY  ) {
						
		    				msg.pose.position.y = msg.pose.position.y -0.01;
		    				
					}

		}
		if (!isFound) {

			try {
				tLst.lookupTransform("/world", "/lostR/pose", ros::Time(0), stamTransform);
			}
			catch (tf::TransformException &exeption) {
				ros::Duration(1.0).sleep();
				continue;
			}
			float dX = fabs(msg.pose.position.x - stamTransform.getOrigin().x());
			float dY = fabs(msg.pose.position.y - stamTransform.getOrigin().y());
			

			if (dX <= minDX && dY <= minDY) {
			    

				isFound = true;
				std_msgs::String status;
				status.data = "302";
				informLostRobot.publish(status);
				ros::spinOnce();
			} 
			else {
	
				if(msg.pose.position.x <= lGr || msg.pose.position.y <= lGr ) {
					if(msg.pose.position.x <= lGr ) {
						msg.pose.position.x = msg.pose.position.x + rand() % changePos;
		    				msg.pose.position.y = msg.pose.position.y + 0.1;
		    				msg.pose.position.z = 0.5; 

					}
					if(msg.pose.position.y <= lGr ) {
						msg.pose.position.x = msg.pose.position.x - 0.1;
		    				msg.pose.position.y = msg.pose.position.y + rand() % changePos;
		    				msg.pose.position.z = 0.5; 

					}

				}
				if(msg.pose.position.x >= rGr || msg.pose.position.y >= rGr ) {
					if(msg.pose.position.x >= lGr ) {
						msg.pose.position.x = msg.pose.position.x - rand() % changePos;
		    				msg.pose.position.y = msg.pose.position.y + 0.1;
		    				msg.pose.position.z = 0.5; 

					}
					if(msg.pose.position.y >= lGr ) {
						msg.pose.position.x = msg.pose.position.x - 0.1;
		    				msg.pose.position.y = msg.pose.position.y - rand() % changePos;
		    				msg.pose.position.z = 0.5; 

					}
						
				}

			
			else {
			msg.pose.position.x = msg.pose.position.x - (rand() % 4)*0.01;
    			msg.pose.position.y = msg.pose.position.y - (rand() % 4)*0.01;
    			msg.pose.position.z = 0.5;    
			}
		}
	}
	   	pub.publish(msg);
	    
	    	transform.setOrigin(tf::Vector3(msg.pose.position.x, msg.pose.position.y, 0.0) );
		tf::Quaternion quaternion;
		quaternion.setRPY(0, 0, 0);
		transform.setRotation(quaternion);
		tBr.sendTransform(tf::StampedTransform(transform, ros::Time(0), "/world", "/searchR/pose"));
	    
	    ros::spinOnce();
	    r.sleep();
  	}
}
