#ifndef ENEMY_H
#define ENEMY_H

#include "robot.h"

class Enemy : public Robot
{
private:
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
            m_rate.sleep();
        }
    }
};

#endif // ENEMY_H
