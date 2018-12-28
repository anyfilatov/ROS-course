#include "turtle.h"
#include <ctime>

using namespace std;
using namespace ros;

enum Stage {
	Lost,
	Rotate_to_partner,
	Go_to_partner
};

 int main(int argc, char **argv) {
	srand(time(0));
    init(argc, argv, "michelangelo");
	bool flag = false;
    float curr_linear, curr_angle;
    int count_frame = 0;
	Turtle michelangelo("michelangelo", 1, 1.0f, 0.5f, 0.0f, 5.0f, 5.0f, 1.0f);
    Rate loop(30);
	Stage stage = Stage::Lost;
	bool isDone = false;
	while (!isDone) {
		michelangelo.getPartnerTransform("leonardo");
		michelangelo.getHomeTransform();

		switch(stage){
			case Stage::Lost:{
				if (count_frame == 0)
        		{
        		    curr_linear = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(michelangelo.speed)));
        		    curr_angle = -1.0 + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2)));
        		}
        		count_frame++;
        		count_frame%=31;
				michelangelo.move(curr_linear, curr_angle);
				if (michelangelo.isDestenation()){
                    michelangelo.move(0, 0);
                    stage = Stage::Rotate_to_partner;
                }
				break;
			}
			case Stage::Rotate_to_partner:{
				float angle = michelangelo.getAngle(michelangelo.partner);
				if (fabs(angle) < 0.1){
					michelangelo.move(0,0);
					stage = Stage::Go_to_partner;
					break;
				}
				michelangelo.move(0, -angle);
				break;
			}
			case Stage::Go_to_partner:{
				float angle = michelangelo.getAngle(michelangelo.home);
                if (!michelangelo.isDestenation()){
                     michelangelo.move(michelangelo.speed, -angle);
                    if (fabs(angle) > 0.3)
                    {
                       michelangelo.move(0, 0); 
                       stage = Stage::Rotate_to_partner;
                    } 
                }
                else michelangelo.move(0,0);
                if (michelangelo.isNearly(michelangelo.home.getOrigin().x(), michelangelo.home.getOrigin().y(), 0.75)){
                    michelangelo.move(0, 0);
                     isDone = true;
                 }
				break;
			}
			default: break;
		}
        loop.sleep();
	}
	return 0;
 }