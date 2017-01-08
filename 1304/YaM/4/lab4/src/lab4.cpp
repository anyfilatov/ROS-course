#include <ros/ros.h>
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "math.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"

ros::Publisher velocity_publisher;

double pi=3.14159265;
float xpose=0;
float ypose=0;
float orient=0;
float anglePose=0;
bool firstStart=true;
float nxpose=0;
float nypose=0;
float norient=0;
float targetx=10;
float targety=5;
bool goal=false;
bool goToSpin=false;
float distTogoal=100;
int maxSteps=20;
int step=0;;

struct direction
{
	static const int LEFT=1;
	static const int RIGHT=2;
};
struct command
{
    static const int goFoward=1;
    static const int goRight=2;
    static const int goLeft=3;
    static const int goBack=4;

};
int currentCommand=1;
bool isLeft=true;
bool isRight=true;
bool isForward=true;
void  directionFinding(const sensor_msgs::LaserScan& msg)
{
	 isForward = true;
	 isRight = true;
	 isLeft = true;
    if (msg.ranges[msg.ranges.size()/2] < 1) {
      isForward = false;
    }

    if (msg.ranges[msg.ranges.size()/6] < 1.5) {
        isRight = false;
    }
    if (msg.ranges[msg.ranges.size()*5/6] < 1.5) {
        isLeft = false;
    }
   
}

float getAngleToTarget() {
    float deltax = (float) (xpose- targetx);
    float deltay = (float) (ypose- targety);
    return (float) (atan2(deltay, deltax) - anglePose);
}

void getRobotPose(nav_msgs::Odometry msg) {
  xpose = msg.pose.pose.position.x;
  ypose = msg.pose.pose.position.y;
  distTogoal=sqrt(pow(xpose - targetx, 2.0) +pow(ypose - targety, 2.0));
  anglePose=tf::getYaw(msg.pose.pose.orientation);
  if(firstStart){
     nxpose = msg.pose.pose.position.x;
     nypose = msg.pose.pose.position.y;
     norient = tf::getYaw(msg.pose.pose.orientation);
     firstStart=false;
  };
}
void lookAtGoal()
{
    geometry_msgs::Twist vel_msg;
    float angle;
    if(xpose==targetx) angle=pi/2-orient;
    else if (ypose==targety) angle=0-orient;
    else angle=atan2(fabs(xpose-targetx), fabs(ypose-targety));

    float gorient=anglePose;
    if (fabs(angle-2*gorient)> 0.1) {
        ROS_ERROR_STREAM("g(: "<<"|"<<angle<<"]"<<gorient <<"abs"<<fabs(angle-gorient));
        goToSpin=true;
        vel_msg.angular.z = 0.3;
        velocity_publisher.publish(vel_msg);
    }
    else {
        ROS_ERROR_STREAM("s(: "<<"|"<<angle<<"]"<<gorient<<"abs"<<fabs(angle-gorient) );
        goToSpin=false;
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
    }


}

void drive(int val)
{
	geometry_msgs::Twist vel_msg;
	vel_msg.linear.x = val;
	velocity_publisher.publish(vel_msg);
}


void turn(int direct)
{
	geometry_msgs::Twist vel_msg;
	if(direct==direction::LEFT)
	{
		vel_msg.angular.z = 0.2;
		velocity_publisher.publish(vel_msg);
	}
	else if(direct==direction::RIGHT)
	{

		vel_msg.angular.z = -0.2;
		velocity_publisher.publish(vel_msg);
	}
}
float clPosex=0;
float clPosey=0;
float clPoseAngle=0;
float clount=0;
int cldir=1;
bool crashListener(float angleToTr)
{   if (clount==0) {
        clPosex=xpose;
        clPosey=ypose;
        clPoseAngle=angleToTr;
    }
    if(fabs(xpose-clPosex)<0.1&&fabs(ypose-clPosey)<0.1&&fabs(angleToTr=-clPoseAngle)<0.1){
        clount++;
        ROS_ERROR_STREAM("true");}
    else {
        clount=0; return false;
    }

    if(clount>=100) {ROS_ERROR_STREAM("sasasa"<<clount); clount=0; return true;};
}

struct mode
{
    static const int goToTarget=1;
    static const int topBack=2;
    static const int rightLeft=3;
    static const int startDrive=4;


};
int currMode=1;
bool turnTopBackProc=false;
int currLeftRgit=command::goLeft;
int currTopBack=command::goFoward;
bool turnTop()
{

    if((fabs(anglePose-(anglePose/fabs(anglePose))*pi/2)>0.05)&&(currMode==mode::topBack))
    {

         turn(direction::LEFT);
         return false;
    } else return true;

}
bool turnBack()
{

    if((fabs(anglePose-pi/2)>0.05)&&(currMode==mode::topBack))
    {

         turn(direction::LEFT);
         return false;
    } else return true;

}
bool turnleftRightProc=false;
bool turnLeft()
{

    if((fabs(anglePose-(anglePose/fabs(anglePose))*pi) > 0.05)&&(currMode==mode::rightLeft))
    {

         turn(direction::RIGHT); 
         false;

    }else {currMode=mode::startDrive; return true;};
}
bool turnRight()
{

    if((fabs(anglePose) > 0.05)&&(currMode==mode::rightLeft))
    {

         turn(direction::RIGHT);
         false;

    }else {currMode=mode::startDrive; return true;};
}

bool robotAction()
{

    float angleToTarget=(getAngleToTarget()+pi);
    while (angleToTarget>pi) angleToTarget-=2*pi;
    while (angleToTarget<-pi) angleToTarget+=2*pi;


  if (distTogoal < 0.1) {
      return true;
  }
  if(currMode==mode::goToTarget)
  {

      if(fabs(angleToTarget) > 0.05)
      {

           turn(direction::LEFT);

      }else {
         if(isForward)
         {
             drive(1.0f);
         }
         else {turnTopBackProc=false; currMode=mode::topBack;};

      }
      return false;

  }
  if((currMode==mode::topBack)||(currMode==mode::rightLeft)||(currMode==mode::startDrive))
  {
      if(!turnTopBackProc)
      {
          if(currTopBack==command::goFoward)
          {
              turnTopBackProc=turnTop();
          }else
          {
              turnTopBackProc=turnBack();
          }

      }else
      {
          if(isForward && !( currMode==mode::rightLeft))
          {
              drive(1.0f);
              if(step==19) {currMode=mode::goToTarget; step=0; return false;}
              step++;
          } else
          {
                  if(currMode==mode::topBack && step!=0){
                  if (currTopBack==command::goFoward) {
                      currTopBack=command::goBack;
                      if(currLeftRgit==command::goLeft) currLeftRgit=command::goRight;
                      else currLeftRgit=command::goLeft;
                  }
                  else {
                      currTopBack=command::goFoward;
                      if(currLeftRgit==command::goLeft) currLeftRgit=command::goRight;
                      else currLeftRgit=command::goLeft;
                  }
                  ROS_ERROR_STREAM("chageDirTopBack(: "<<currTopBack);
              }else{
                  if(turnleftRightProc && currMode==mode::startDrive)  {
                      if(currLeftRgit==command::goLeft) currLeftRgit=command::goRight;
                      else currLeftRgit=command::goLeft;
                      ROS_ERROR_STREAM("chageRightlef(: "<<currLeftRgit);
                  }

              };
              turnleftRightProc=false;
              currMode=mode::rightLeft;
              step=0;

              if(currLeftRgit==command::goLeft)
              {
                 turnleftRightProc=turnLeft();
              }else
              {
                 turnleftRightProc=turnRight();
              }

          }
      }
   return false;
  }
}

void  searchingGoal(ros::NodeHandle node)
{
    ros::Rate rate(30);
	while(node.ok() && !goal){
		goal=robotAction();
		ros::spinOnce();
		rate.sleep();
	}

}


int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop");

  ros::NodeHandle node;
  velocity_publisher = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ros::Subscriber laser_subscriber = node.subscribe("/base_scan", 1000, directionFinding);
  ros::Subscriber robot_subscriber = node.subscribe("/odom", 1000, getRobotPose);

  searchingGoal(node);
  return 0;
}
