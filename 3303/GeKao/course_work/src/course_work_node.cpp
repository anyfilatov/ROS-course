#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "std_msgs/Empty.h"
#include <cstdlib>
#include <vector>

using namespace std;

int shoot = 0;

struct TargetInit {
    double speed;
    double y0;
    double alpha;
    long time0;

    void init(long _time0) {
        time0 = _time0;
        generateInitValues();
    }

    void generateInitValues() {
        double f = (double)rand() / RAND_MAX;
        alpha = f * M_PI;

        y0 = rand() % 20 + 10;

        f = (double)rand() / RAND_MAX;
        speed = 0.01 + f * 0.05;
    }

    void print() {
        std::cout << "Bullet init: speed = " << speed << " ; y0 = " << y0 << " ; alpha = " << alpha << " ; time0 = " << time0 << ";"  << std::endl;
    }
};

struct BulletInit {
    double speed;
    double theta;
    long time0;

    void init(TargetInit &target_init, double _speed, long _time0) {
        speed = _speed;
        time0 = _time0;

        double beta = atan(target_init.speed * (time0 - target_init.time0) / target_init.y0);
        theta = asin(target_init.speed / speed * cos(beta)) + beta;
    }

    void print() {
        std::cout << "Bullet init: speed = " << speed << " ; theta = " << theta << " ; time0 = " << time0 << ";" << std::endl;
    }
};

struct Coordinates {
    double x;
    double y;
    double z;

    void init(double _x, double _y, double _z) {
        x = _x;
        y = _y;
        z = _z;
    }

    void print(std::string beginning) {
        std::cout << beginning << " : ( " << x << " ; " << y << " ; " << z << " )" << std::endl;
    }
};

Coordinates calculateTarget(TargetInit target, long time) {
    Coordinates result;
    result.x = target.speed * (time - target.time0) * cos(target.alpha);
    result.y = target.y0;
    result.z = target.speed * (time - target.time0) * sin(target.alpha);
    return result;
}

Coordinates calculateBullet(BulletInit bullet, TargetInit target, long time) {
    Coordinates result;
    result.x = bullet.speed * ((double)time - bullet.time0) * sin(bullet.theta) * cos(target.alpha);
    result.y = bullet.speed * ((double)time - bullet.time0) * cos(bullet.theta);
    result.z = bullet.speed * ((double)time - bullet.time0) * sin(bullet.theta) * sin(target.alpha);
    return result;
}

void updatePose(geometry_msgs::Pose &pose, float x, float y, float z) {
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;

    pose.orientation.x = 1 * sin(M_PI / 2.0 / 2);
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = cos(M_PI / 2.0 / 2);
}

gazebo_msgs::SpawnModel createModel(string model, string model_name) {
    gazebo_msgs::SpawnModel srv_target;
    ifstream fin("/home/andrey/.gazebo/models/" + model + "/model.sdf");

    string model_target;
    string buf;
    while (!fin.eof()) {
        getline(fin, buf);
        model_target += buf + "\n";
    }
    srv_target.request.model_xml = model_target;
    srv_target.request.model_name = model_name;
    return srv_target;
}

gazebo_msgs::ModelState createModelState(string model_name, Coordinates coord) {
    gazebo_msgs::ModelState msg;
    msg.model_name = model_name;
    updatePose(msg.pose, coord.x, coord.y, coord.z);
    return msg;
}

void chatterCallback(const std_msgs::Empty::ConstPtr& msg)
{
    ROS_INFO("Shooting signal recieved");
    shoot++;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "course_work");

    srand(time(0));

    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("shooting_signal", 1000, chatterCallback);
    ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);

    ros::service::waitForService("gazebo/spawn_sdf_model");
    ros::ServiceClient add_robot = nh.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");

    gazebo_msgs::SpawnModel srv;

    int n = 10;

    for(int i = 0; i < n; i++) {
        srv = createModel("cricket_ball", "bullet" + to_string(i));
        add_robot.call(srv);

        srv = createModel("bowl", "target" + to_string(i));
        add_robot.call(srv);
    }

    ros::Rate rate(25);

    double bullet_speed = 0.4;
    long time = 0;

    std::vector<TargetInit> target_inits;
    std::vector<BulletInit> bullet_inits;
    std::vector<Coordinates> targets;
    std::vector<Coordinates> bullets;


    for(int i = 0; i < n; i++) {
        TargetInit target_init;
        BulletInit bullet_init;
        Coordinates target;
        Coordinates bullet;

        target_init.init(time);
        bullet_init.init(target_init, bullet_speed, time);

        target.init(0, target_init.y0, 0);
        bullet.init(0, 0, 0);

        target_inits.push_back(target_init);
        bullet_inits.push_back(bullet_init);
        targets.push_back(target);
        bullets.push_back(bullet);
    }

    while (ros::ok()) {
      int count = 0;
        for(int i = 0; i < n; i++) {
            TargetInit target_init = target_inits[i];
            BulletInit bullet_init = bullet_inits[i];
            Coordinates target = targets[i];
            Coordinates bullet = bullets[i];

            if(shoot <= i) {
                bullet_init.init(target_init, bullet_speed, time);
                bullet_inits[i] = bullet_init;
            }

            if(bullet.y < target.y) {
                target = calculateTarget(target_init, time);
                targets[i] = target;
                pub.publish(createModelState("target" + to_string(i), target));

                if(shoot > i) {
                    bullet = calculateBullet(bullet_init, target_init, time);
                    bullets[i] = bullet;
                    pub.publish(createModelState("bullet" + to_string(i), bullet));
                }
            } else {
                count++;
            }
        }

        if(count == n) {
            ROS_INFO("Well done!");
            break;
        }

        time++;
        rate.sleep();

        ros::spinOnce();
    }
    return 0;
}