#include "turtle.h"
#include <ctime>

using namespace std;
using namespace ros;

enum Stage {
	Rotate_to_partner,
	Go_to_partner,
	Rotate_to_home,
	Go_to_home
};

 int main(int argc, char **argv) {
	srand(time(0));
    init(argc, argv, "leonardo");
	bool flag = false;
	Turtle leonardo("leonardo", 2, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.3f);
    Rate loop(30);
	Stage stage = Stage::Rotate_to_partner;
	bool isDone = false;
	while (!isDone) {

		ROS_INFO("%d", stage);
        leonardo.getPartnerTransform("michelangelo");
		leonardo.getHomeTransform();
		switch(stage){
			case Stage::Rotate_to_partner:{

				float angle = leonardo.getAngle(leonardo.partner);
				if (fabs(angle) < 0.1){
					leonardo.move(0,0);
					stage = Stage::Go_to_partner;
					break;
				}
				leonardo.move(0, -angle);
				break;
			}
			case Stage::Go_to_partner:{
				float angle = leonardo.getAngle(leonardo.partner);
                leonardo.move(leonardo.speed, -angle);

                if (fabs(angle) > 0.3) {
                    leonardo.move(0, 0);
                    stage = Stage::Rotate_to_partner;
                }

				ROS_INFO("angle: %3.2f", angle);
                if (leonardo.isDestenation()){
                    leonardo.move(0, 0);
                    stage = Stage::Rotate_to_home;
                }
				break;
			}
			case Stage::Rotate_to_home:{
				float angle = leonardo.getAngle(leonardo.home);
				if (fabs(angle) < 0.1){
					leonardo.move(0,0);
					stage = Stage::Go_to_home;
					break;
				}
				leonardo.move(0, -angle);
				break;
			}
			case Stage::Go_to_home:{
				float angle = leonardo.getAngle(leonardo.home);
                leonardo.move(leonardo.speed, -angle);
                if ((fabs(angle) > 0.3)){
                    leonardo.move(0, 0);
                    stage = Stage::Rotate_to_home;
                } 
                if (leonardo.isNearly(leonardo.home.getOrigin().x(), leonardo.home.getOrigin().x(), 0.25)) {
                    leonardo.move(0,0);
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