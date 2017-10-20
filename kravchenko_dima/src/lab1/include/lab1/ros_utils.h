#pragma once

#include <ros/ros.h>

#include <string>

// When C++ resolves typedef, it does not see lab1::Move yet.
// Thus insted of this neat method we have to accept callback
// as a `void *` and cast it in .cpp
//
// typedef void (*f)(const lab1::Move&) sub_callback;

class RosCommon {
protected:
    ros::NodeHandle *node;
    static const std::string topic_name;
public:
    RosCommon() {};

    virtual std::string get_node_name() = 0;
    void init_ros(int argc, char *argv[]);

    virtual ~RosCommon();
};

class RosClient: public RosCommon {
private:
    ros::Publisher move_pub;
public:
    RosClient() {};

    virtual std::string get_node_name();
    void init_ros(int argc, char *argv[]);
    void send_move(int move);

    virtual ~RosClient();
};

class RosServer: public RosCommon {
private:
    ros::Subscriber move_sub;
public:
    RosServer() {};

    virtual std::string get_node_name();
    void init_ros(int argc, char *argv[], void *callback);
    void spin();

    virtual ~RosServer();
};
