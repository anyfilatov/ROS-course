#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <string>
#include <vector>

using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace std;
using namespace visualization_msgs;


vector<vector<Marker>> markers;
   
vector<vector<Pose2D>> poseMatrix;

int matrixSize = 0;

Publisher * markerPublisher = 0;   

Pose2D flagmanPose;


void initMarkers()
{
    for(int i = 0; i < matrixSize; i++)
    {
        for(int j = 0; j < matrixSize; j++)
        {
            Marker marker;
            marker.header.frame_id = "/my_frame";
            marker.header.stamp = Time::now();

            marker.ns = "basic_shapes";
            marker.id = i * matrixSize + j;

            marker.type = Marker::SPHERE;
            marker.action = Marker::ADD;

            marker.pose.position.z = 0;

            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;

            marker.color.r = 1.0f;
            marker.color.g = 0.0f;
            marker.color.b = 0.0f;
            marker.color.a = 1.0;

            marker.pose.position.x = 5 / 2;
            marker.pose.position.y = 0;

            marker.lifetime = ros::Duration();
            markers[i][j] = marker;
        }
    }
}


// callback формирует матрицу позиций на основе сообщеники типа Int32MultiArray
void poseMatrixCallback(Int32MultiArray msg)
{ 
    matrixSize = sqrt(msg.data.size() / 2);
    auto it = msg.data.begin();

    poseMatrix.resize(matrixSize);
    markers.resize(matrixSize);

    for(int i = 0; i < matrixSize; i++)
    {
        poseMatrix[i].resize(matrixSize);
        markers[i].resize(matrixSize);

        for(int j = 0; j < matrixSize; j++)
        {    
            Pose2D pose;
            pose.x = (*it++);
            pose.y = (*it++);

            poseMatrix[i][j] = pose;   
        }
    }

    initMarkers();
}

void firePoseCallback(Pose2D msg)
{
    // todo: handle firePose        
}

void flagmanPoseCallback(Pose2D msg)
{
    cout << "flagman" << endl;
    flagmanPose = msg;         
}

int main(int argc, char **argv)
{
    init(argc, argv, "world");

    NodeHandle nodeHandle;

    Subscriber defSubscriber = nodeHandle.subscribe(
        "defengers",                        
        10,                                 
        poseMatrixCallback                  
    );

    Subscriber flagmanPoseSubscriber = nodeHandle.subscribe(
        "/flagman/pose",                    
        10,                                 
        flagmanPoseCallback                 
    );

    Subscriber firePoseSubscriber = nodeHandle.subscribe(
        "fire",                             
        10,                                 
        firePoseCallback                    
    );

    flagmanPose.x = -10000;
    flagmanPose.y = -10000;

    Publisher mp = nodeHandle.advertise<Marker>("visualization_marker", 100);
    markerPublisher = & mp;

    Marker planet;
    planet.header.frame_id = "/my_frame";
    planet.header.stamp = Time::now();

    planet.ns = "basic_shapes";
    planet.id = 1000;

    planet.type = Marker::SPHERE;
    planet.action = Marker::ADD;

    planet.pose.position.z = 0;

    planet.pose.orientation.x = 0.0;
    planet.pose.orientation.y = 0.0;
    planet.pose.orientation.z = 0.0;
    planet.pose.orientation.w = 1.0;

    planet.scale.x = 1.0;
    planet.scale.y = 1.0;
    planet.scale.z = 1.0;

    planet.color.r = 0.0f;
    planet.color.g = 1.0f;
    planet.color.b = 0.0f;
    planet.color.a = 1.0;

    planet.pose.position.x = 5 / 2;
    planet.pose.position.y = 0;

    planet.lifetime = ros::Duration();
    markerPublisher->publish(planet);

    Marker flagman;
    flagman.header.frame_id = "/my_frame";
    flagman.header.stamp = Time::now();

    flagman.ns = "basic_shapes";
    flagman.id = 2000;

    flagman.type = Marker::SPHERE;
    flagman.action = Marker::ADD;

    flagman.pose.position.z = 0;

    flagman.pose.orientation.x = 0.0;
    flagman.pose.orientation.y = 0.0;
    flagman.pose.orientation.z = 0.0;
    flagman.pose.orientation.w = 1.0;

    flagman.scale.x = 1.0;
    flagman.scale.y = 1.0;
    flagman.scale.z = 1.0;

    flagman.color.r = 0.0f;
    flagman.color.g = 0.0f;
    flagman.color.b = 1.0f;
    flagman.color.a = 1.0;

    flagman.pose.position.x = 5 / 2;
    flagman.pose.position.y = 0;

    flagman.lifetime = ros::Duration();
    markerPublisher->publish(flagman);


    Rate rate(10);

    while (nodeHandle.ok())
    {   
        flagman.pose.position.x = flagmanPose.x;
        flagman.pose.position.y = flagmanPose.y; 

        markerPublisher->publish(planet);
        markerPublisher->publish(flagman);

        for (int i = 0; i < matrixSize; i++)
        {
            for (int j = 0; j < matrixSize; j++)
            {
                if (poseMatrix[i][j].x == -1)
                    cout << "(   ) ";
                else
                {
                    cout << '(' << poseMatrix[i][j].x << ' ' << poseMatrix[i][j].y << ')' << ' ';
                    markers[i][j].pose.position.x = poseMatrix[i][j].x;
                    markers[i][j].pose.position.y = poseMatrix[i][j].y;
                }
                markerPublisher->publish(markers[i][j]);
            }
            cout << endl;
        }
        cout << endl;

        spinOnce();
        rate.sleep();
    }
}