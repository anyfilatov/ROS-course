#include "lab1/ros_utils.h"
#include "kravchenko_lab1/Move.h"
#include "lab1/move.h"

#include <iostream>

RosServer *server;

static void move_handler(const kravchenko_lab1::Move& move)
{
    std::cout << get_move_name(move.move_id) << std::endl;
}

static void init_ros(int argc, char *argv[])
{
    server = new RosServer();
    server->init_ros(argc, argv, (void *)move_handler);
}

int main(int argc, char *argv[])
{
    init_ros(argc, argv);
    server->spin();
}
