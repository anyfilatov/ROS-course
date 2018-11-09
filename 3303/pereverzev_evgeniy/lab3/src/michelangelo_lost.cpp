#include "turtle.h"
#include <ctime>

using namespace std;
using namespace ros;

 int main(int argc, char **argv) {
	srand(time(0));
    init(argc, argv, "michelangelo");
	bool flag = false;
    float curr_x, curr_y;
    int count_frame = 0;
	Turtle michelangelo("michelangelo", 1, 1.0f, 0.5f, 0.0f, 3.0f, 3.0f, 1.0f);
    Rate loop(30);
	while (ok()) {
		michelangelo.getTransform("leonardo");
		if (michelangelo.isDestenation() || flag)
		{	
			if (michelangelo.isNearly(0, 0))
			{
				break;
			}
			flag = true;
			michelangelo.move(michelangelo.partner_X, michelangelo.partner_Y);
			ROS_INFO ("Michelangelo follow Leonardo (%3.2f, %3.2f)", michelangelo.partner_X, michelangelo.partner_Y);
		}
		else
		{ 
            if (count_frame == 0)
            {
                curr_x = rand()% 10;
                curr_y = rand()% 10;
            }
			michelangelo.move(curr_x, curr_y);
            count_frame++;
            count_frame%=31;
			ROS_INFO ("Michelangelo move to next position (%3.2f, %3.2f)", curr_x, curr_y);
		}
		//sleep(2);
        loop.sleep();
	}
	return 0;
 }