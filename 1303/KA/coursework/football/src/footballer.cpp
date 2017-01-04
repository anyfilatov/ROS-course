#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include <string> 
#include <cstdlib>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <ros/console.h>
#include <cmath>  

#include <fstream>
#include <vector>
#include <iostream>
#include <algorithm>
#include <vector>

#include <football_msg/ballstate.h>

using namespace std;

static const string FREEZY = "_FREEZEY_";
static const string STICK = "_STICK_";
static const string FLY = "_FLY_";
static const string NOTHING = "_NOTHING_";

static const string LOGTAG = "#FTB# ";

enum Position {
	GK = 1,
	CB,
	MF,
	FW
};

enum State {
	STAY = 1,
	COMEBACK,
	TAKE_BALL,
	ATTACK,
	STRIKE,
	WAIT,
	KEEP, //only gk
	PASS, //only gk
	GRAB
};

class Footballer {
private:
	//number description: 1(GK), 2-3(CB), 4(MF), 5(FW)
	string teamName, number, name, rival, rivalGL;
	Position position;
	State state;
	double x, y, angle;
	double gap;

	//skills.
	int speed, control, power;

	vector<string> allies;
	vector<string> enemies;

	ros::NodeHandle &nh;
	tf::TransformListener &listener;
	tf::StampedTransform &transform;
	ros::Publisher pub, pubMsg;
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
	    robotState.pose.orientation.z = sin(angle / 2);
	    robotState.pose.orientation.w = cos(angle / 2);
	    pub.publish(robotState);
	}

	void initVisualisation() {
		// cout << "init visualization" << endl;
		ROS_INFO("%s init visualization", LOGTAG.c_str());

		ros::service::waitForService("gazebo/spawn_sdf_model");
	    ros::ServiceClient addRobot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
	    gazebo_msgs::SpawnModel srv;
	 
	    // ifstream fin("/home/user/.gazebo/models/pioneer2dx/model.sdf");
	    string real = "/home/user/.gazebo/models/turtlebot/model.sdf";
	    string barca = "/home/user/.gazebo/models/create/model.sdf";
	    string club = real;
	    if (teamName == "barca") club = barca;
	    ifstream fin(club.c_str());
 
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
		robotState.pose.orientation.z = sin(angle / 2);
		robotState.pose.orientation.w = cos(angle / 2);

		broadcastPosition();
		repaint();
		sleep(3);
	}

	// read data contains info about team players.
	// {team number x y}
	void readData(int curNum) {
		int num, posX, posY, ang;
		string team, riv;
		ifstream fileData;
		fileData.open("/home/user/Develop/workspace/src/football/res/data.txt", ios::app); 
		if (fileData.is_open()) {
		    // cout << "File is open."<<endl;
		    // cout << "cur team:" << curTeam << ", cur num:" << curNum << endl;
		    while(fileData >> team >> num >> posX >> posY >> ang >> riv) {
		        // cout << "team:" << team << ", num:" << num << ", x:" << posX << ", y:" << posY << endl;
		        if (team == teamName && num == curNum) {
		        	x = posX;
		        	y = posY;
		        	angle = ang;
		        	rival = riv;
		        	rivalGL = rival + "_goal-line";
		        	cout << "init player " << name << ", x:" << x << ", y:" << y << ", angle:" << angle << ", rival:" << rival << endl;
		        } else if (team == teamName) {
		        	allies.push_back(riv);
		        } else {
		        	enemies.push_back(riv);
		        }
		    }
		    fileData.close();
		} else cout << "Unable to open the file"; 
	}

	// read player's skills
	// {name speed control power}
	void readSkills() {
		ifstream fileSkills;
		fileSkills.open("/home/user/Develop/workspace/src/football/res/skills.txt", ios::app);
		int playerSpeed, playerControl, playerPower;
		string playerName;

		cout << "Current player name:" << name << endl;
		if (fileSkills.is_open()) {
		    while(fileSkills >> playerName >> playerSpeed >> playerControl >> playerPower) {
		    	cout << "Read skill. Name:" << playerName << ", Speed:" << playerSpeed << ", Control:" << playerControl << ", Power:" << playerPower << endl;
		        if (playerName == name) {
		        	speed = playerSpeed;
		        	control = playerControl;
		        	power = playerPower;
		        	cout << "### Init player's skills. Name:" << name << ", Speed:" << speed << ", Control:" << control << ", Power:" << power << endl;
		        }
		    }
		    fileSkills.close();
		} else cout << "Unable to open the file with players skills"; 
	}

	void initRole() {
		int num = atoi(number.c_str());
		// cout << "INIT ROLE ### team int:" << team << ", string:" << teamName << endl;

		switch(num) {
			case 1:
				position = GK;
				cout << "init role as GK" << endl;
				state = STAY;
				break;

			case 2:
			case 3:
				position = CB;
				cout << "init role as CB" << endl;
				state = STAY;
				break;

			case 4:
				position = MF;
				cout << "init role as MF" << endl;
				state = TAKE_BALL;
				break;

			case 5:
				position = FW;
				cout << "init role as FW" << endl;
				state = TAKE_BALL;
				break;
			default:
				cout << "Unrecognized position by number" << endl;
				exit(1);
		}

		readData(num);
		readSkills();
	}

	void shiftPosition(double offsetX, double offsetY) {
		// ROS_INFO("%s Shift position (%f, %f)", LOGTAG.c_str(), offsetX, offsetY);
		// ROS_INFO("%s Current position (%f, %f)", LOGTAG.c_str(), x, y);
		double cst = 50000.0;
		// double cst = 5000.0;
		int itX = (int) ((abs(offsetX) * cst) / (speed * 1.0)) + 1; //because !=0
		int itY = (int) ((abs(offsetY) * cst) / (speed * 1.0)) + 1; 
		// cout << "*** SHIFT POSITION offsetX: " << offsetX << ", offsetY: " << offsetY << ", itX: " << itX << ", itY: " << itY << endl;
		int itMax = std::max(itX, itY);
		double gapX = offsetX / itX;
		double gapY = offsetY / itY;
		// double gapAngle = (atan2(offsetY, offsetX)) / itMax;
		// double gapAngle = (atan2(offsetY, offsetX)) / iter;
		double defObstackleDist = 2.0;
		bool isObstackle = checkObstackle(defObstackleDist);
		if (isObstackle) {
			defObstackleDist /= 2; //feature for around!
			isObstackle = false;
		}
		// ROS_INFO("Before check is obstackle %d", isObstackle);
		ros::Rate rate(100);
		// ros::Rate rate(100); //todo uncomment
		while (nh.ok() && itMax != 0 && !isObstackle) {
			if (itX) x += gapX;
			if (itY) y += gapY;
			// angle += gapAngle;
			
			itY--;
			itX--;
			itMax--;
		
			broadcastPosition();
			repaint(); //repaint state.

			isObstackle = checkObstackle(defObstackleDist);
			// ROS_INFO("Result of seq check is obstackle %d", isObstackle);

			ros::spinOnce();
			rate.sleep();
		}

		if (isObstackle && state != ATTACK) {
			ROS_INFO("%s !!! Is obstackle !!!", LOGTAG.c_str());
			aroundObstackle();
		}
		//todo check if isObstackle then go around
	}

	void aroundObstackle() {
		ROS_INFO("%s !!! To around the obstackle !!!", LOGTAG.c_str());

		double minDist = 1000.0;
		string nameToMin = "";

		for (int i = 0; i < allies.size(); i++) {
			try {
					listener.lookupTransform(name, allies[i], ros::Time(0), transform);
				}	catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

				if (transform.getOrigin().length() <= minDist) {
					minDist = transform.getOrigin().length();
					nameToMin = allies[i];
				}
		}

		for (int i = 0; i < enemies.size(); i++) {
			try {
					listener.lookupTransform(name, enemies[i], ros::Time(0), transform);
				}	catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

				if (transform.getOrigin().length() <= minDist) {
					minDist = transform.getOrigin().length();
					nameToMin = enemies[i];
				}
		}



		try {
				listener.lookupTransform(name, nameToMin, ros::Time(0), transform);
			}	catch (tf::TransformException &e) {
				ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
			}

		double _x = transform.getOrigin().x();
		double _y = transform.getOrigin().y();
		if (_y > 0) _y += 1;
		else _y -= 1;
		shiftPosition(_x, _y);
	}


	bool checkObstackle() {
		return checkObstackle(2.0);
	}

	bool checkObstackle(double lim) {
		for (int i = 0; i < allies.size(); i++) {
			try {
					listener.lookupTransform(name, allies[i], ros::Time(0), transform);
				}	catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

				if (transform.getOrigin().length() <= lim) {
					return true;
				}
		}

		for (int i = 0; i < enemies.size(); i++) {
			try {
					listener.lookupTransform(name, enemies[i], ros::Time(0), transform);
				}	catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

				if (transform.getOrigin().length() <= lim) {
					return true;
				}
		}

		return false;
	}

	void shiftPositionOLD(double offsetX, double offsetY) {
		int iter = 100000 / speed; //depends on speed skill
		double gapX = offsetX / iter;
		double gapY = offsetY / iter;
		double gapAngle = (atan2(offsetY, offsetX) + angle) / iter;
		// double gapAngle = (atan2(offsetY, offsetX)) / iter;
		ros::Rate rate(100);

		//featch 100
		while (nh.ok() && iter != 100) {
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

//number description: 1(GK), 2-3(CB), 4(MF), 5(FW)
	void playAsGk() {
		ROS_INFO("%s Run play as goalkeeper", LOGTAG.c_str());
		state = KEEP;
		//at the moment impl only for FW
		//switch-case...
		ros::Rate rate(100);
		while (nh.ok()) {
			switch(state) {
				case KEEP:
					ROS_INFO("%s play status KEEP", LOGTAG.c_str());
					keepBall();
					state = PASS;
					break;

				case PASS:
					ROS_INFO("%s play status PASS", LOGTAG.c_str());
					passBall();
					state = KEEP;
					break;

				default:
					// cout << "Unchecked case of player's state" << endl;
					ROS_INFO("%s Unchecked case of GK player's state", LOGTAG.c_str());
			}

			ros::spinOnce();
			rate.sleep();
		}
	}

	void playAsCb() {
		ROS_INFO("%s Run play as central back", LOGTAG.c_str());
		state = WAIT;
		//at the moment impl only for FW
		//switch-case...
		ros::Rate rate(100);
		while (nh.ok()) {
			switch(state) {
				case WAIT:
					ROS_INFO("%s play status WAIT", LOGTAG.c_str());
					wait(); 
					state = GRAB;
					break;

				case GRAB:
					ROS_INFO("%s play status GRAB", LOGTAG.c_str());
					moving("ball");
					// grabBall(); //todo impl
					state = PASS;
					break;

				case PASS:
					ROS_INFO("%s play status PASS", LOGTAG.c_str());
					passBall(); //todo impl
					state = WAIT;
					break;

				default:
					// cout << "Unchecked case of player's state" << endl;
					ROS_INFO("%s Unchecked case of CB player's state", LOGTAG.c_str());
			}

			ros::spinOnce();
			rate.sleep();
		}
	}

	void playAsMf() {
		ROS_INFO("%s Run play as MF", LOGTAG.c_str());
		state = WAIT;
		//at the moment impl only for FW
		//switch-case...
		ros::Rate rate(100);
		while (nh.ok()) {
			switch(state) {
				case WAIT:
					ROS_INFO("%s play status WAIT", LOGTAG.c_str());
					wait(); 
					state = GRAB;
					break;

				case GRAB:
					ROS_INFO("%s play status GRAB", LOGTAG.c_str());
					moving("ball");
					// grabBall(); //todo impl
					state = PASS;
					break;

				case PASS:
					ROS_INFO("%s play status PASS", LOGTAG.c_str());
					passBall(); //todo impl
					state = WAIT;
					break;

				default:
					// cout << "Unchecked case of player's state" << endl;
					ROS_INFO("%s Unchecked case of CB player's state", LOGTAG.c_str());
			}

			ros::spinOnce();
			rate.sleep();
		}
	}

	void playAsFw() {
		// cout << "Run play as forward" << endl;
		ROS_INFO("%s Run play as forward", LOGTAG.c_str());
		//at the moment impl only for FW
		//switch-case...
		ros::Rate rate(100);
		while (nh.ok()) {
			switch(state) {
				case TAKE_BALL:
					// cout << "play status TAKE_BALL" << endl;
					ROS_INFO("%s play status TAKE_BALL", LOGTAG.c_str());
					takeBall();
					state = ATTACK;
					break;

				case ATTACK:
					// cout << "play status ATTACK" << endl;
					ROS_INFO("%s play status ATTACK", LOGTAG.c_str());
					attack();
					state = STRIKE;
					break;

				case STRIKE:
					// cout << "play status STRIKE" << endl;
					ROS_INFO("%s play status STRIKE", LOGTAG.c_str());
					strike();
					// state = COMEBACK;
					state = WAIT;
					break;

				case WAIT:
					ROS_INFO("%s play status WAIT", LOGTAG.c_str());
					wait();

				default:
					// cout << "Unchecked case of player's state" << endl;
					ROS_INFO("%s Unchecked case of player's state", LOGTAG.c_str());
			}

			ros::spinOnce();
			rate.sleep();
		}
		
	}


	void moving(string target) {
		moving(target, gap);
	}

	void moving(string target, double limit) {
		broadcastPosition();
		ros::Rate rate(100);
		while (nh.ok()) {
			try {
					listener.lookupTransform(name, target, ros::Time(0), transform);
				}
				catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

			if (transform.getOrigin().length() > limit) {
				double _x = transform.getOrigin().x();
				double _y = transform.getOrigin().y();
				shiftPosition(_x, _y);
			} else return;

			ros::spinOnce();
			rate.sleep();
		}
	}

	void wait() {
		ros::Rate rate(1000);
		while (nh.ok() && state == WAIT) {

			broadcastPosition();
			repaint();
			ros::spinOnce();
			rate.sleep();
		}
	}



	void toHoldBall() {
		// std_msgs::String s;
		// s.data = name;
		football_msg::ballstate bs;
		bs.state = STICK;
		bs.holder = name;
	  	ROS_INFO("%s pub the message to ball", LOGTAG.c_str());
	  	broadcastPosition(); //feature (ball will player)
	    pubMsg.publish(bs);
	    ros::spinOnce();
	}


	void takeBall() {
		moving("ball");
		toHoldBall();
	}


	void attack() {
		// cout << "ATTACK TO: " << rival << endl;
		ROS_INFO("%s ATTACK TO: %s", LOGTAG.c_str(), rival.c_str());
		moving(rival);
	}

	void strike() {
		ROS_INFO("%s STRIKE TO: %s", LOGTAG.c_str(), rival.c_str());

		try {
				listener.lookupTransform(name, rivalGL, ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
				strike();
			}

		
		double _x = transform.getOrigin().x();
		double _y = transform.getOrigin().y();

		// double shift = 1.0;
		// if (_x < 0) shift *= -1;
		// _x += shift;


		football_msg::ballstate bs;
		bs.state = FLY;
		bs.holder = NOTHING;
		bs.power = power;
		bs.x = _x;
		bs.y = _y;
	  	ROS_INFO("%s strike the ball", LOGTAG.c_str());
	  	// broadcastPosition(); //feature (ball will player)
	    pubMsg.publish(bs);
	    ros::spinOnce();		
	}


	void keepBall() {
		ROS_INFO("%s KEEP BALL", LOGTAG.c_str());

		ros::Rate rate(100);
		while (nh.ok()) {
			try {
				listener.lookupTransform(name, "ball", ros::Time(0), transform);
			}
			catch (tf::TransformException &e) {
				ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
				broadcastPosition();
				ros::Duration(1.0).sleep();
				//strike();
			}

			if (transform.getOrigin().length() < 2.0) {
				double _x = transform.getOrigin().x();
				double _y = transform.getOrigin().y();
				shiftPosition(0, _y);
			}
			//todo add logic of keep ball...

			ros::spinOnce();
			rate.sleep();
		}
		
	}

	void passBall() {
		ROS_INFO("%s CALL UNIMPLEMENT METHOD PASS BALL", LOGTAG.c_str());


		double minDist = 1000;
		string nameToMin = name; //!!!
		for (int i = 0; i < allies.size(); i++) {
			try {
					listener.lookupTransform(name, allies[i], ros::Time(0), transform);
				}	catch (tf::TransformException &e) {
					ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
					broadcastPosition();
					ros::Duration(1.0).sleep();
					continue;
				}

				if (transform.getOrigin().length() <= minDist) {
					minDist = transform.getOrigin().length();
					nameToMin = allies[i];
				}
		}

		try {
			listener.lookupTransform(name, nameToMin, ros::Time(0), transform);
		}
		catch (tf::TransformException &e) {
			ROS_ERROR("%s %s", LOGTAG.c_str(), e.what());
			broadcastPosition();
			ros::Duration(1.0).sleep();
			// strike();
		}

		double _x = transform.getOrigin().x();
		double _y = transform.getOrigin().y();

		football_msg::ballstate bs;
		bs.state = FLY;
		bs.holder = NOTHING;
		bs.power = power;
		bs.x = _x;
		bs.y = _y;
	  	ROS_INFO("%s pass the ball", LOGTAG.c_str());
	  	// broadcastPosition(); //feature (ball will player)
	    pubMsg.publish(bs);
	    ros::spinOnce();
	}

public:
	Footballer(ros::NodeHandle &nh, tf::TransformListener &listener, tf::StampedTransform &transform, string teamName, string number, string name) : nh(nh), listener(listener), transform(transform), teamName(teamName), number(number), name(name) {
		x = y = angle = 0;
		gap = 1.0;
		initRole();

		pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
		// pubMsg = nh.advertise<std_msgs::String>("/holdball", 100);
		pubMsg = nh.advertise<football_msg::ballstate>("/ballstate", 100);

		initVisualisation();
	}

	~Footballer(){}
	

	void play() {
		switch(position) {
			case GK:
				playAsGk();
				break;

			case CB:
				playAsCb();
				break;

			case MF:
				playAsMf();
				break;

			case FW:
				playAsFw();
				break;

			default:
				// cout << "Undefined position" << endl;
				ROS_INFO("%s Undefined position", LOGTAG.c_str());
		}
	}
};


int main(int argc, char **argv) {
	ros::init(argc, argv, "footballer");
	if (argc < 3) {
		ROS_ERROR("Not specified robot footballer's team and number as an arguments!");
		return -1;
	}
	sleep(5);

	ros::NodeHandle nh;
	tf::TransformListener listener;
	tf::StampedTransform transform;
	Footballer footballer(nh, listener, transform, argv[1], argv[2], argv[3]);
	
	football_msg::ballstate bs;

	footballer.play();

	ros::spin();
	return 0;
}