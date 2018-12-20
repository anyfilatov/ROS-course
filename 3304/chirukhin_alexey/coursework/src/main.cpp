#include "base.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "base");

	Base base;
	base.defendPlanet();    

    ros::spin();
    return 0;
}
