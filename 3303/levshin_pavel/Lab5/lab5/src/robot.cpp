#include "robot.h"

Robot::Robot(string name, string fn, int id, float x, float y, float speedLinear, float speedAngular, float min_dist){
	this->name = name;
	this->friendName = fn;
    this->speedLinear = speedLinear;
	this->speedAngular = speedAngular;
	this->min_dist = min_dist;
	ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
 	gazebo_msgs::SetModelState setmodelstate;
  	gazebo_msgs::ModelState modelstate;
  	modelstate.model_name = name + "_model";
  	setmodelstate.request.model_state = modelstate;
	setmodelstate.request.model_state.pose.position.x = x;
	setmodelstate.request.model_state.pose.position.y = y;
	
	if (!client.call(setmodelstate))
  	{
        ROS_ERROR("Failed to call service ");
  	}
	pub = nh.advertise<geometry_msgs::Twist>(name + "_robot/cmd_vel", 1000);	
}

void Robot::move(float linear_x, float angular_z){
    	geometry_msgs::Twist pos;
		pos.linear.x = linear_x;
		pos.angular.z = angular_z;
		pub.publish(pos);
}

bool Robot::isNearThePoint(float x, float y,  float eps){
    float squareDistance = pow((x - myPosition.getOrigin().x()), 2.0f) + pow((y - myPosition.getOrigin().y()), 2.0f);
	return (squareDistance < eps*eps);
}

bool Robot::isFriendReached(){
    return isNearThePoint(myFriend.getOrigin().x(), myFriend.getOrigin().y(), min_dist);
}

tf::Transform Robot::getTransform(string name){

	tf::StampedTransform tr;
    try
	{
		listener.waitForTransform("odom", name + "_robot", ros::Time(0), ros::Duration(1.0));
		listener.lookupTransform("odom", name + "_robot", ros::Time(0), tr);

	}	
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("???%s",ex.what());
	}
	tf::Transform transform;
	transform.setOrigin(tr.getOrigin());
	transform.setRotation(tr.getRotation());
	return transform;
}

float Robot::calculateRotation(tf::Transform q_to,tf::Transform q_from){
    float x_cur = q_to.getOrigin().x() - q_from.getOrigin().x();
    float y_cur = q_to.getOrigin().y() - q_from.getOrigin().y();
    float a = sqrt(x_cur*x_cur + y_cur*y_cur);
    float alfa = acos(x_cur / a);
    if(y_cur <0){alfa = -alfa;}
    //ROS_INFO("HELPER x_cur= %f  y_cur= %f",x_cur, y_cur);
    //ROS_INFO("HELPER calculateRotation: %f",alfa);
    return alfa;
}
float Robot::toEulerAngle(tf::Transform q)
{
   double siny_cosp = +2.0 * (q.getRotation().w() * q.getRotation().z() + q.getRotation().x() * q.getRotation().y());
   double cosy_cosp = +1.0 - 2.0 * (q.getRotation().y() * q.getRotation().y() + q.getRotation().z() * q.getRotation().z());  
   float yaw = atan2(siny_cosp, cosy_cosp);
   //ROS_INFO("HELPER toEulerAngle: %f",yaw);
   return yaw;
}

float Robot::getAlfa(tf::Transform q_to,tf::Transform q_from){
    float a = calculateRotation(q_to,q_from) - toEulerAngle(q_from);
    if(a>M_PI){a -= 2*M_PI;}
    if(a< - M_PI){a += 2* M_PI;}
    return a;
}