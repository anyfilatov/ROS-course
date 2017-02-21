#include "gazebo_model.h"

const float  GazeboModel::RADIAN_PER_DEGREE = 0.0174533;
const string GazeboModel::SERVICE_SPAWN_SDF_MODEL = "gazebo/spawn_sdf_model";
const string GazeboModel::SERVICE_SET_MODEL_STATE = "gazebo/set_model_state";
const string GazeboModel::SERVICE_DELETE_MODEL    = "gazebo/delete_model";

GazeboModel::GazeboModel(string path, string name) : name(name) {
    geometry_msgs::Pose position;
    state.pose = position;
    state.model_name = name;

    SpawnModel spawnModelCommand;
    spawnModelCommand.request.model_xml = readFullFile(path + "/" + name +"/model.sdf");
    spawnModelCommand.request.initial_pose = position;
    spawnModelCommand.request.model_name = name;

    NodeHandle nodeHandle;
    ros::service::waitForService(SERVICE_SPAWN_SDF_MODEL);
    ServiceClient spawnModelService = nodeHandle.serviceClient<SpawnModel>(SERVICE_SPAWN_SDF_MODEL);
    spawnModelService.call(spawnModelCommand);

    ros::service::waitForService(SERVICE_DELETE_MODEL);
    deleteModelService = nodeHandle.serviceClient<DeleteModel>(SERVICE_DELETE_MODEL);

    setModelStateService = nodeHandle.advertise<ModelState>(SERVICE_SET_MODEL_STATE, 10);
}

/**
 * Read all lines from file and save them to string.
 * @param path Path to file contains model.
 * @return String with XML-representation of model.
 */
string GazeboModel::readFullFile(string path) {
    ifstream fileInputStream(path);
    stringstream buffer;
    buffer << fileInputStream.rdbuf();
    return buffer.str();
}

GazeboModel::~GazeboModel() {
    DeleteModel deleteModelCommand;
    deleteModelCommand.request.model_name = name;
    deleteModelService.call(deleteModelCommand);
}
