#ifndef ENEMY_H
#define ENEMY_H

#include "robot.h"

class Enemy : public Robot
{
private:
    ros::Publisher m_shot_publisher;

    std::thread* m_shoot_thread;
    bool m_shoot_canceled;

public:
    Enemy(ros::NodeHandle& hnd, int rate, std::string name) : Robot(hnd, rate, name)
    {
        m_shoot_canceled = false;
        m_shoot_thread = new std::thread([=] { this->_shoot(); });
    }

    ~Enemy()
    {
        stopShooting();
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
            int counter = rand() % 10;
            std::string name = "shot/robot" + std::to_string(counter);
            ROS_INFO_STREAM("shoot: " << name);

            m_shot_publisher = m_node_handle.advertise<std_msgs::String>(name, 10);
            std_msgs::String msg;
            msg.data = std::string("DIE!");
            m_shot_publisher.publish(msg);

            for (int i = 0; i < 300; i++)
                m_rate.sleep();
        }
    }
};

#endif // ENEMY_H
