#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>  
#include <cstdlib>
#include <ctime>

#include <football_msg/ballstate.h>

using namespace std;

static const string NOTHING = "_NOTHING_";
static const string FREEZY_MSG = "_FREEZEY_";
static const string STICK_MSG = "_STICK_";
static const string FLY_MSG = "_FLY_";
static const string LOGTAG = "#BALL# ";

enum State {
	FREEZE = 1,
	STICK,
	FLY
};

class Ball {
private:
	double x, y, angle;
	string name;
	State state;
	string holdPlayer;
	double goalX, goalY;

	bool initBeforeSticking;
	double stickGapX, stickGapY;
	int power;

	ros::NodeHandle &nh;
	tf::TransformListener &listener;
	tf::StampedTransform &transform;
	ros::Publisher pub;
	gazebo_msgs::ModelState robotState;


	void broadcastPosition() {
		//for broadcast.
		static tf::TransformBroadcaster br;
		tf::Transform transform;

		transform.setOrigin(tf::Vector3(x, y, 0.0));
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
	}

	void repaint() {
		robotState.pose.position.x = x;
	    robotState.pose.position.y = y;
	    pub.publish(robotState);
	}

	void initVisualisation() {
		// cout << "init visualization" << endl;
		ROS_INFO("%s init visualization", LOGTAG.c_str());

		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient addRobot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    ifstream fin("/home/user/.gazebo/models/cricket_ball/model.sdf");
 
		string model;
	    string buf;
	    while(!fin.eof()){
	        getline(fin, buf);
	        model += buf + "\n";
	    }

	    srv.request.model_xml = model;
	    srv.request.model_name = name;
	    geometry_msgs::Pose pose;
	    srv.request.initial_pose = pose;
	    addRobot.call(srv);

	    //state...
	    robotState.model_name = name;
	    robotState.pose.position.x = x;
		robotState.pose.position.y = y;
		robotState.pose.position.z = 0.0;
		robotState.pose.orientation.z = 0.0;
		robotState.pose.orientation.w = 0.0;

		broadcastPosition();
		repaint();
		sleep(3);
	}

	//todo change alg as in football impl
	void shiftPositionOLD(double offsetX, double offsetY) {
		int iter = 1000;
		double gapX = offsetX / iter;
		double gapY = offsetY / iter;
		double gapAngle = (atan2(offsetY, offsetX) - angle) / iter;
		ros::Rate rate(100);

		while (nh.ok() && iter != 0) {
			x += gapX;
			y += gapY;
			angle += gapAngle;
			iter--;
		
			broadcastPosition();
			repaint(); //repaint state.

			ros::spinOnce();
			rate.sleep();
		}
	}

	void shiftPositionOLD_(double offsetX, double offsetY, int power) {
		int iter = 80000 / power;
		double gapX = offsetX / iter;
		double gapY = offsetY / iter;
		double gapAngle = (atan2(offsetY, offsetX) - angle) / iter;
		ros::Rate rate(100);

		while (nh.ok() && iter != 0) {
			x += gapX;
			y += gapY;
			angle += gapAngle;
			iter--;
		
			broadcastPosition();
			repaint(); //repaint state.

			ros::spinOnce();
			rate.sleep();
		}
	}

	void shiftPosition(double offsetX, double offsetY) {
		int itX = (int) ((abs(offsetX) * 100.0) * (100.0 / power)) + 1; //because !=0
		int itY = (int) ((abs(offsetY) * 100.0) * (100.0 / power)) + 1; 
		cout << "*** SHIFT POSITION offsetX: " << offsetX << ", offsetY: " << offsetY << ", itX: " << itX << ", itY: " << itY << endl;
		int itMax = std::max(itX, itY);
		double gapX = offsetX / itX;
		double gapY = offsetY / itY;

		ros::Rate rate(100);
		while (nh.ok() && itMax != 0) {
			if (itX) x += gapX;
			if (itY) y += gapY;
			// angle += gapAngle;
			
			itY--;
			itX--;
			itMax--;
		
			broadcastPosition();
			repaint(); //repaint state.

			ros::spinOnce();
			rate.sleep();
		}
	}

	void transformPosition(double _x, double _y, bool isNeg) {

		double offset = 0.3; //apply if x != 0

		if (isNeg) offset *= -1;
		
		x += _x + offset;
		y += _y;
		broadcastPosition();
		repaint();
		ros::spinOnce();

	}


	void handlerHold(const football_msg::ballstate& bs) {
		ROS_INFO("%s Triggered the handlerHold, status is: %s", LOGTAG.c_str(), bs.state.c_str());
		string st = bs.state;
		if (st == STICK_MSG && holdPlayer == NOTHING) {
			holdPlayer = bs.holder;
			state = STICK;
			ROS_INFO("%s The ball holding by (%s)", LOGTAG.c_str(), holdPlayer.c_str());
		} else if (st == FLY_MSG) {
			state = FLY;
			ROS_INFO("%s The ball is fly to goal (%f, %f)", LOGTAG.c_str(), bs.x, bs.y);
			holdPlayer = bs.holder;
			goalX = bs.x;
			goalY = bs.y;
			power = bs.power;
			//todo add afte the speed
			// shiftPosition(bs.x, bs.y); 
		} else {
			//todo impl the tackling
			ROS_INFO("Not triggered the handlerHold, because holdPlayer=%s", holdPlayer.c_str());
		}
	}

	//moving if state != FREEZE
	void movingIf() {
		switch(state) {
			case STICK:
				ROS_INFO("%s select case STICK", LOGTAG.c_str());
				// cout << "select case STICK" << endl;
				stickMove();
				break;

			case FLY:
				ROS_INFO("%s select case FLY", LOGTAG.c_str());
				// cout << "select case FLY" << endl;
				flyMove();
				break;

			case FREEZE:
				ROS_INFO("%s select case FREEZE", LOGTAG.c_str());
				// cout << "select case FREEZE" << endl;
				freezeMove();
				break;

			default:
				ROS_INFO("%s Unchecked case of ball moving", LOGTAG.c_str());
				// cout << "Unchecked case of ball moving" << endl;
		}
	}

	void stickMove() {
		ROS_INFO("%s Triggered the stickMove", LOGTAG.c_str());
		// tf::TransformListener listener;
		// tf::StampedTransform transform;

		broadcastPosition();
		ros::spinOnce();

		double last_x = -1000;
		ros::Rate rate(100);
		while (nh.ok() && state == STICK) {
			try {
					listener.lookupTransform(name, holdPlayer, ros::Time(0), transform);
				}
				catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					// ros::Duration(1.0).sleep();
					continue;
				}

			double _x = transform.getOrigin().x();
			double _y = transform.getOrigin().y();
			bool isNeg = false;
			if (last_x != -1000 /*&& last_y != -1000*/) {
				double dif_x = _x - last_x;
				// double dif_y = _y - last_y;
				isNeg = dif_x <= 0;
			}
			last_x = _x;

			//workaround!
			if (holdPlayer == "navas" || holdPlayer == "pepe" || holdPlayer == "ramos" || holdPlayer == "bale" || holdPlayer == "ronaldo")
				isNeg = false;
			else
				isNeg = true;

			// last_y = _y;

			// ROS_INFO("Stick to x:%f y:%f", _x, _y);
			transformPosition(_x, _y, isNeg);

			ros::spinOnce();
			rate.sleep();
		}
	}

	void flyMove() {
		//continue...
		// shiftPosition(goalX, goalY); 
		shiftPosition(goalX, goalY);
		state = FREEZE;
	}

	void freezeMove() {
		ros::Rate rate(1000);
		while (nh.ok() && state == FREEZE) {

			broadcastPosition();
			repaint();
			ros::spinOnce();
			rate.sleep();
		}
	}

public:
	Ball(ros::NodeHandle &nh, tf::TransformListener &listener, tf::StampedTransform &transform) : nh(nh), listener(listener), transform(transform) {
		x = 0.0;
		y = 0.0;
		angle = 0;
		name = "ball";
		state = FREEZE;
		holdPlayer = NOTHING;

		initBeforeSticking = false;
		stickGapX = stickGapY = 0;

		pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);



		initVisualisation();
	}

	~Ball(){}

	void play() {
		ros::Rate rate(100);
		while (nh.ok()) {
			movingIf();
			ros::spinOnce();
			rate.sleep();
		}
	}

	ros::Subscriber listen() {
		return nh.subscribe("/ballstate", 100, &Ball::handlerHold, this);
	}
	
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "ball");

	ROS_INFO("%s The ball node init!", LOGTAG.c_str());

	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	Ball ball(nh, listener, transform);
	ros::Subscriber sub = ball.listen();
	ball.play();
	
	ros::spin();
	return 0;
}