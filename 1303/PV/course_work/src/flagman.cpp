#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"

#include <string>
#include <vector>

using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace std;

vector<vector<Pose2D>> poseMatrix;

const int delta = 1;        

const int fireFactor = 3;

int matrixSize = 0;

Pose2D pose;

bool commanderWin = false;     

void poseMatrixCallback(Int32MultiArray msg)
{
    matrixSize = sqrt(msg.data.size() / 2);
    auto it = msg.data.begin();

    poseMatrix.resize(matrixSize);

    for(int i = 0; i < matrixSize; i++)
    {
        poseMatrix[i].resize(matrixSize);

        for(int j = 0; j < matrixSize; j++)
        {    
            Pose2D pose;
            pose.x = (*it++);
            pose.y = (*it++);

            poseMatrix[i][j] = pose;   
        }
    }
}

void winStatusCallback(String msg)
{
    commanderWin = true;         
}

float getDistance(Pose2D from, Pose2D to)
{
    float dx = from.x - to.x;
    float dy = from.y - to.y;

    return sqrt(dx * dx + dy * dy);
}

int main(int argc, char **argv)
{
    init(argc, argv, "flagman");

    NodeHandle nodeHandle;

    pose.x = 5 / 2;
    pose.y = 30;

    Publisher firePublisher = nodeHandle.advertise<Pose2D>("fire", 10);

    Publisher winPublisher = nodeHandle.advertise<String>("/flagman/iwin", 10);

    Publisher posePublisher = nodeHandle.advertise<Pose2D>("/flagman/pose", 10);

    Subscriber defSubscriber = nodeHandle.subscribe(
        "defengers",                        
        10,                                 
        poseMatrixCallback                  
    );

    Subscriber winStatusSubscriber = nodeHandle.subscribe(
        "/commander/iwin",                  
        10,                                 
        winStatusCallback                   
    );

    for (int i = 0; nodeHandle.ok(); i++)
    {    
        if(commanderWin)
        {
            cout << "\n\n\n * * * WE HAVE DIED * * * \n\n\n" << endl;
            sleep(5);
            return 0;
        }

        if (matrixSize > 0) 
        {
            Pose2D nearest = poseMatrix[0][0];
            float minDistance = 1000;

            // полный перебор, ищем ближайший корабль, чтобы пальнуть в него
            for(int i = 0; i < matrixSize; i++)
            {
                for(int j = 0; j < matrixSize; j++)
                {
                    if(poseMatrix[i][j].x == -1)
                        continue;

                    float distance = getDistance(pose, poseMatrix[i][j]);
                    if(distance < minDistance)
                    {
                        minDistance = distance;
                        nearest = poseMatrix[i][j];
                    }
                }
            }

            if(nearest.x == -1)
                continue;

            if(i % fireFactor == 0)
            {
                firePublisher.publish(nearest);
                cout << "FIRE TO (" << nearest.x << ", " << nearest.y << ")" << endl;
            }

            pose.y -= delta;
            posePublisher.publish(pose);
            cout << "POSE (" << pose.x << ", " << pose.y << ")" << endl;

            if(pose.y <= nearest.y)
            {
                String msg;
                winPublisher.publish(msg);

                cout << "\n\n\n * * * SUCCESS * * * \n\n\n" << endl;
                sleep(5);
                return 0;
            }
        }

        spinOnce();
        sleep(1);
    }
}