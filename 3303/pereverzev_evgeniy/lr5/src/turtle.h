#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <string>
#include <cmath>

using namespace std;
using namespace ros;

 class Turtle {
    private:
        tf::Transform getTransform(string name);
		float calculateRotation(tf::Transform q_to);
	    string name;
	    NodeHandle nh;
	    Publisher pub;
        
    public:
	    Turtle(string name, int id, float r, float g, float b, float x, float y, float speed);
	    void move(float linear, float angle);
		bool isNearly(float x, float y, float dist);
	    bool isDestenation();
		void getPartnerTransform(string name);
		void getHomeTransform();
    	float getAngle(tf::Transform q_to);
		tf::Transform partner;
		tf::Transform home;
        float speed;
  
};
