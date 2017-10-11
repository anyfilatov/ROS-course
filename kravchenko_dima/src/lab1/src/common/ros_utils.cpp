#include "lab1/ros_utils.h"
#include "kravchenko_lab1/Move.h"
#include "lab1/move.h" // best name I came up with...

#include <iostream>

// ===== RosCommon =====

const std::string RosCommon::topic_name = "kravchenko_lab1_topic";

void RosCommon::init_ros(int argc, char *argv[])
{
    ros::init(argc, argv, this->get_node_name());
    this->node = new ros::NodeHandle();
}

RosCommon::~RosCommon()
{
// no need to do anything as we only destroy this object on exit
}

// ===== RosClient =====

std::string RosClient::get_node_name()
{
    return "kravchenko_lab1_client";
}

void RosClient::init_ros(int argc, char *argv[])
{
    RosCommon::init_ros(argc, argv);
    this->move_pub = this->node->advertise<kravchenko_lab1::Move>(RosCommon::topic_name, 100);
}

void RosClient::send_move(int move_id)
{
    const char *move_name = get_move_name(move_id);
    if (move_name == NULL)
        return;

    kravchenko_lab1::Move move;
    move.move_id = move_id;

    this->move_pub.publish(move);
}

RosClient::~RosClient()
{
// no need to do anything as we only destroy this object on exit
}

// ===== RosServer =====

std::string RosServer::get_node_name()
{
    return "kravchenko_lab1_server";
}

void RosServer::init_ros(int argc, char *argv[], void *callback)
{
    RosCommon::init_ros(argc, argv);
    this->move_sub = this->node->subscribe(RosCommon::topic_name, 100, (void (*)(const kravchenko_lab1::Move&))callback);
}

void RosServer::spin()
{
    ros::spin();
}

RosServer::~RosServer()
{
// no need to do anything as we only destroy this object on exit
}
