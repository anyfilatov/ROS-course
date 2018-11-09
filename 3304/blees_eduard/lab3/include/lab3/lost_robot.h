#include "lab3/robot.h"
#include <string>
#include <cstdlib>
#include <ctime>

using namespace std;

class LostBot: public Bot
{
private:
	float 	speed;

	void runAround();
	void followSaver();

public:
	LostBot();
	LostBot(string saver_name, string name, string rviz_marker_topic, float x, float y);
	void start();
};
