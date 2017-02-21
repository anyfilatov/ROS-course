#include "cleaner.h"

const float Cleaner::ROTATE_STEP = 5;
const float Cleaner::MOVE_STEP = 0.1;

Cleaner::Cleaner(string pathToModel, string name) : GazeboModel(pathToModel, name) {
    NodeHandle nodeHandle;
    Subscriber subscriber = nodeHandle.subscribe("cleaner_controller_topic", 1000, scannerHandler);
}

void Cleaner::makeStep() {
    static float taskDist = 0;
    static float taskAngle = 0;

    if (taskDist > 0 && !inFrontObstacle) {
        cout << "Move forward: " << taskDist << endl;
        moveForward(MOVE_STEP);
        taskDist -= MOVE_STEP;
    } else if (taskAngle > 0) {
        cout << "Rotate on " << taskAngle << " degrees "<< endl;
        rotate(ROTATE_STEP);
        taskAngle -= ROTATE_STEP;
        if (taskAngle < 0) {
            taskAngle = 0;
        }
    } else if (taskAngle < 0) {
        cout << "rotate: " << taskAngle << endl;
        rotate(-ROTATE_STEP);
        taskAngle += ROTATE_STEP;
        if (taskAngle > 0) {
            taskAngle = 0;
        }
    } else {
        execute( taskDist, taskAngle );
        cout << "executeAlg: " << taskDist << " " << taskAngle << endl;
    }
}

void Cleaner::moveForward(float distance) {
    double radianAngle = angle * RADIAN_PER_DEGREE;
    state.pose.position.x += distance * cos(radianAngle);
    state.pose.position.y += distance * sin(radianAngle);
    setModelStateService.publish(state);
}

void Cleaner::rotate(float angleDegree) {
    angleDegree = ((int)(angleDegree + angle + 360) % 360);
    Quaternion quaternion(Vector3(0, 0, 1), angleDegree * RADIAN_PER_DEGREE);
    geometry_msgs::Quaternion odometerQuaternion;
    quaternionTFToMsg(quaternion, odometerQuaternion);
    state.pose.orientation = odometerQuaternion;
    setModelStateService.publish(state);
}

void Cleaner::scannerHandler(ConstLaserScanStampedPtr &report) {
//    gazebo::msgs::LaserScan scanner = report->scan();
//    for (int i = 0; i < scanner.count(); ++i) {
//        if (scanner.ranges(i) < 1.0) {
//            inFrontObstacle = true;
//            return;
//        }
//    }
//    inFrontObstacle = false;
}

void Cleaner::execute(float &distance, float &angle) {
    static float s = 0;
    static int flag = 1;
    distance = angle = 0.0;

    if (!inFrontObstacle) {
        distance = 0.5;
        s += distance;
    } else if (s > 0) {
        if (s < 5) {
            if (flag == 1) {
                angle = 90;
                flag = 0;
            } else {
                angle = -90;
                flag = 1;
            }
        } else {
            float tmpAngle = 180 - ((float)15/s) * 180 /3.14;
            if (flag == 1) {
                angle = tmpAngle;
                flag == 0;
            } else {
                angle = -tmpAngle;
                flag = 1;
            }
        }
        s = 0;
    } else {
        angle = 90;
    }
}

