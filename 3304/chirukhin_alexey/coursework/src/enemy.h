#ifndef ENEMY_H
#define ENEMY_H

#include "robot.h"
#include <time.h>
#include <vector>
#include <string>

class Enemy : public Robot
{
private:
    static const int shoot_pause_microseconds = 6000000;

    ros::Publisher m_shoot_publisher;
    ros::Subscriber m_appear_subscriber;

    std::thread* m_shoot_thread;
    bool m_shoot_canceled;

    std::vector<std::string> robots;

public:
    Enemy(ros::NodeHandle& hnd, int rate, std::string name) : Robot(hnd, rate, name)
    {
        srand(time(NULL));
        m_appear_subscriber = m_node_handle.subscribe("appear", 10, &Enemy::_appearCallback, this);
        m_shoot_canceled = false;
        m_shoot_thread = nullptr;
    }

    ~Enemy()
    {
        stopShooting();
    }

    void startShooting()
    {
        m_shoot_canceled = false;
        m_shoot_thread = new std::thread([=] { this->_shoot(); });
    }

    void stopShooting()
    {
        m_shoot_canceled = true;
        if (m_shoot_thread != nullptr)
        {
            m_shoot_thread->join();
            delete m_shoot_thread;
            m_shoot_thread = nullptr;
        }
    }

private:
    void _shoot()
    {
        while (!m_shoot_canceled)
        {
            if (!robots.empty())
            {
                int pos = rand() % robots.size();

                ROS_INFO_STREAM("Enemy shoot: " << robots.at(pos));

                m_shoot_publisher = m_node_handle.advertise<std_msgs::String>("shoot/" + robots.at(pos), 10);
                std_msgs::String msg;
                msg.data = std::string("DIE!");
                m_shoot_publisher.publish(msg);

                robots.erase(robots.begin() + pos);
            }

            usleep(shoot_pause_microseconds);
        }
    }

    void _appearCallback(const std_msgs::String& msg)
    {
        if (msg.data != getName())
        {
            robots.push_back(msg.data);
        }
    }
};

#endif // ENEMY_H
