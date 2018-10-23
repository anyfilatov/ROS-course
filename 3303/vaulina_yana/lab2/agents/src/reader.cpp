#include <ros/ros.h>
#include <my_message/Message1.h>
#include <ctime>


void reader(const my_message::Message1 & message)
{
	if(message.x == -1 && message.y == -1){
		ROS_INFO("attack is over");
		ros::shutdown();
	} else {	
		srand( time( 0 ) );
		int guess = 0;
		for(int i=0;i<3;i++){
			int my_x = rand() % 5;
			int my_y = rand() % 5;
			ROS_INFO("%d )intended point(%d;%d).",i, my_x, my_y);
			if (my_x == message.x && my_y == message.y){
				ROS_INFO("point is guessed");
				guess = 1;
				break;
			} else {
				ROS_INFO("point is NOT guessed");
			}
		}
		int p = rand() % 100;
		ROS_INFO("probability value %d",p);
		if ((guess==0 && p<5)||(guess==1 && p<80)){
			ROS_INFO("HURRAH!!!rocket is shot down");
		}else{
			ROS_INFO("OH NO!!!we're hit =(");
		}
	
		ROS_INFO("the point was (%d;%d).\n", message.x, message.y);

	}
}

int main(int argc, char **argv){
	ros::init(argc,argv,"reader");
	ROS_INFO("Reader is ready to defend\n");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("Name",30,reader);
	ros::spin();
	return 0;
}
