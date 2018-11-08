#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <string>
#include <cmath>

using namespace std;
using namespace ros;

 class Turtle {
    private:
	    void sendTransform(float x, float y);
	    string name;
	    visualization_msgs::Marker marker;
	    NodeHandle nh;
	    Publisher pub;
        
    public:
	    Turtle(string name, int id, float r, float g, float b, float x, float y, float speed);
	    void move(float x, float y);
		bool isNearly(float x, float y);
	    bool isDestenation();
        void getTransform(string name);
	    float partner_X;
	    float partner_Y;
        float speed;
  
};
