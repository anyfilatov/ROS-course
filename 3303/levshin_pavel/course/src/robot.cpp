#include "robot.h"

Robot::Robot(string name, string topic, int id, int32_t markerType, tf::Vector3 position, tf::tfVector4 rotation, tf::Vector3 scale, tf::tfVector4 color, float speed, float min_dist){
	this->name = name;
    this->speed = speed;
	this->min_dist = min_dist;
	pub = nh.advertise<visualization_msgs::Marker>(topic, 100, true);
	pubGameOver = nh.advertise<std_msgs::String>("/GameOver", 100, true);
	subGameOver = nh.subscribe("/GameOver", 100, &Robot::handlerGameOver, this);
	marker.header.frame_id = "/world";
    marker.header.stamp = Time::now();
    marker.ns = "";
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.x = rotation.x();
    marker.pose.orientation.y = rotation.y();
    marker.pose.orientation.z = rotation.z();
    marker.pose.orientation.w = rotation.w();
    marker.id = id;
    marker.type = markerType;
    marker.scale.x = scale.x();
    marker.scale.y = scale.y();
    marker.scale.z = scale.z();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.color.r = color.x();
    marker.color.g = color.y();
    marker.color.b = color.z();
    marker.color.a = color.w();
	geometry_msgs::Point p;
	p.x = 0;
	p.y = 0;
	p.z = 0;
	marker.points.push_back(p);
	marker.lifetime = Duration();
	pub.publish(marker);
	saveTransform();
	
}

void Robot::moveTo(float x, float y){
    float distance = sqrt(pow((x - marker.pose.position.x), 2.0f) + pow((y - marker.pose.position.y), 2.0f));
    marker.pose.position.x += (x - marker.pose.position.x)/distance * speed / ((double)FPS);
    marker.pose.position.y += (y - marker.pose.position.y)/distance * speed / ((double)FPS);	
}

bool Robot::isNearThePoint(float x, float y){
    float squareDistance = pow((x - marker.pose.position.x), 2.0f) + pow((y - marker.pose.position.y), 2.0f);
	return (squareDistance < min_dist*min_dist);
}

bool Robot::isFriendReached(){
    return isNearThePoint(myFriend.getOrigin().x(), myFriend.getOrigin().y());
}

tf::Transform Robot::getTransform(string name)
{
    static tf::TransformListener listener;
    static tf::StampedTransform tr;
	tf::Transform transform;
	try
	{
		//listener.waitForTransform(BASE_WORLD, name, ros::Time(0), ros::Duration());
		listener.lookupTransform(BASE_WORLD, name, ros::Time(0), tr);
	}	
	catch (tf::TransformException &ex)
	{
		//ROS_INFO("[%s] %s", this->name.c_str(), ex.what());
	}
	transform.setOrigin(tr.getOrigin());
	transform.setRotation(tr.getRotation());
	return transform;
}

void Robot::saveTransform(){
	tf::Transform transform;
	transform.setOrigin(tf::Vector3(marker.pose.position.x, marker.pose.position.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), BASE_WORLD, name));
}

ros::V_string Robot::getNodesNameByPattern(string pattern){
    ros::V_string results;
    ros::V_string nodes;
    ros::master::getNodes(nodes);
    auto it = std::find_if(std::begin(nodes), std::end(nodes), [pattern](const string & elem) {
        return std::strstr(elem.c_str(), pattern.c_str()) != nullptr;
    });
    while (it != std::end(nodes)) {
        results.push_back(*it);
        it = std::find_if(std::next(it), std::end(nodes), [pattern](const string & elem) {
            return std::strstr(elem.c_str(), pattern.c_str()) != nullptr;
        });
    }
    return results;
}

void Robot::handlerGameOver(const std_msgs::String& msg){
	ROS_INFO("%s", msg.data.c_str());
    deleteMarker();
	ros::shutdown();
}

void Robot::deleteMarker(){
	marker.action = visualization_msgs::Marker::DELETE;
	pub.publish(marker);
    ROS_INFO("[%s] I'm death", name.c_str());
}