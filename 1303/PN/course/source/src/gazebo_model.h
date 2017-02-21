#ifndef CLEANER_MODEL_H
#define CLEANER_MODEL_H

#include <fstream>
#include <ros/ros.h>

#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/DeleteModel.h"

using std::string;
using std::ifstream;
using std::stringstream;


using ros::Time;
using ros::Publisher;
using ros::NodeHandle;
using ros::ServiceClient;

using gazebo_msgs::ModelState;
using gazebo_msgs::DeleteModel;
using gazebo_msgs::SpawnModel;

class GazeboModel {

protected:
    static const float RADIAN_PER_DEGREE;

    ServiceClient deleteModelService;
    Publisher setModelStateService;
    ModelState state;

    /** Angle that model rotated. */
    float angle = 0;

    GazeboModel(string path, string name);
    ~GazeboModel();

private:
    static const string SERVICE_SPAWN_SDF_MODEL;
    static const string SERVICE_SET_MODEL_STATE;
    static const string SERVICE_DELETE_MODEL;

    const string name;

    string readFullFile(string path);
};


#endif //CLEANER_MODEL_H
