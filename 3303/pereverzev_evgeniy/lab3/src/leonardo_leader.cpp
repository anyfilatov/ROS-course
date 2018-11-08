#include "turtle.h"
#include <ctime>

using namespace std;
using namespace ros;

 int main(int argc, char **argv) {
	srand(time(0));
    init(argc, argv, "leonardo");
	bool flag = false;
	Turtle leonardo("leonardo", 2, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 2.0f);
    Rate loop(30);
	while (ok()) {
        leonardo.getTransform("michelangelo");
		if (leonardo.isDestenation() || flag)
		{	
			if (leonardo.isNearly(0, 0))
			{
				break;
			}
			flag = true;
			leonardo.move(0.0f, 0.0f);
			ROS_INFO ("Leonardo move to home (%3.2f, %3.2f)", 0.0f, 0.0f);
		}
		else
		{
			leonardo.move(leonardo.partner_X, leonardo.partner_Y);
			ROS_INFO ("Leonardo move to Michelangelo (%3.2f, %3.2f)", leonardo.partner_X, leonardo.partner_Y);
		}
		//sleep(2);
        loop.sleep();
	}
	return 0;
 }