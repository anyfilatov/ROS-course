#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

#include <string>
#include <queue>

using namespace ros;
using namespace geometry_msgs;
using namespace std_msgs;
using namespace std;
using namespace visualization_msgs;

#define MATRIX_SIZE 5

enum CellState
{
	EMPTY_CELL,                
	ASSIGNED_CELL,             
	CLOSED_CELL                
};

// ячейка
struct Cell
{
    Pose2D goal;               
    Pose2D pose;               
    CellState state;           
};

Cell matrix[MATRIX_SIZE][MATRIX_SIZE];
       
Pose2D firePose;               
Pose2D nullPose;               
Pose2D startPose;              

const int delta = 1;           
float step = 0.01;

bool flagmanWin = false;       

void firePointCallback(Pose2D msg)
{
    firePose = msg;
}

void winStatusCallback(String msg)
{
    flagmanWin = true;        
}

void initMatrix()
{
    for(int i = 0; i < MATRIX_SIZE; i++)
    {
        for(int j = 0; j < MATRIX_SIZE; j++)
        {
            matrix[i][j].state = EMPTY_CELL;
            matrix[i][j].pose.x = matrix[i][j].pose.y = -1;
            matrix[i][j].goal.x = (/*MATRIX_SIZE / 2 + */j) * delta;
            matrix[i][j].goal.y = (MATRIX_SIZE - i) * delta;
        }
    }
}

Int32MultiArray matrixToArray()
{
    Int32MultiArray msg;

    for(int i = 0; i < MATRIX_SIZE; i++)
    {
        for(int j = 0; j < MATRIX_SIZE; j++)
        {
            msg.data.push_back(matrix[i][j].pose.x);
            msg.data.push_back(matrix[i][j].pose.y);
        }
    }

    return msg;
}

bool operator == (Pose2D p1, Pose2D p2)
{
    return p1.x == p2.x && p1.y == p2.y;
}

bool operator != (Pose2D p1, Pose2D p2)
{
    return ! operator == (p1, p2);
}

bool areWeWin()
{
    for(int i = 0; i < MATRIX_SIZE; i++)
        for(int j = 0; j < MATRIX_SIZE; j++)
            if(matrix[i][j].state != CLOSED_CELL)
                return false;
    return true;
}

int main(int argc, char **argv)
{
	init(argc, argv, "commander");
    initMatrix();

    NodeHandle nodeHandle;

    startPose.x = MATRIX_SIZE / 2;
    startPose.y = 0;

    nullPose.x = -1;
    nullPose.y = -1;

    firePose = nullPose;

    Publisher publisher = nodeHandle.advertise<Int32MultiArray>("defengers", 10);

    Publisher winPublisher = nodeHandle.advertise<String>("/commander/iwin", 10);

    Subscriber fireSubscriber = nodeHandle.subscribe(
        "fire",                             
        10,                                 
        firePointCallback                   
    );

    Subscriber winStatusSubscriber = nodeHandle.subscribe(
        "/flagman/iwin",                    
        10,                                 
        winStatusCallback                   
    );

    while(nodeHandle.ok())
    {
        // обход матрицы по [i][j] - на каждом шаге отправляется очередной кораблик
        for(int i = MATRIX_SIZE - 1; i >= 0 && nodeHandle.ok(); i--)
        {
            for(int j = MATRIX_SIZE - 1; j >= 0 && nodeHandle.ok(); j--)
            {
                // если флагман победил, то нет смысла продолжать что-то делать
                if(flagmanWin)
                {
                    cout << "\n\n\n * * * WE HAVE DIED * * * \n\n\n" << endl;
                    sleep(5);
                    return 0;
                }

                // обход матрицы по [k][s] - обновляется позиция тех кораблей, которые уже в пути
                // + выполняется обработка выстрелов
                for(int k = MATRIX_SIZE - 1; k >= i && nodeHandle.ok(); k--)
                {
                    for(int s = MATRIX_SIZE - 1; s >= j && nodeHandle.ok(); s--)
                    {
                        // проверка был ли сделан очередной выстрел
                        if(firePose != nullPose)
                        {
                            bool someoneDied = false;

                            // смотрим в кого попали
                            for(int i = 0; i < MATRIX_SIZE && !someoneDied; i++)
                            {
                                for(int j = 0; j < MATRIX_SIZE && !someoneDied; j++)
                                {
                                    // в [i][j] попали, отмечаем что он мертв
                                    if(matrix[i][j].pose == firePose)
                                    {
                                        matrix[i][j].pose = nullPose;
                                        matrix[i][j].state = EMPTY_CELL;
                                        someoneDied = true;
                                        firePose = nullPose;
                                    }
                                }
                            }
                        }

                    
                        if(matrix[k][s].state == EMPTY_CELL)
                        {
                            matrix[k][s].state = ASSIGNED_CELL;
                            matrix[k][s].pose = startPose;
                        } 
                        else if(matrix[k][s].state == ASSIGNED_CELL)
                        {
                            Pose2D goal = matrix[k][s].goal;

                            if(matrix[k][s].pose.x != goal.x)
                                matrix[k][s].pose.x += matrix[k][s].pose.x < goal.x ? delta : -delta;

                            if(matrix[k][s].pose.y != goal.y)
                                matrix[k][s].pose.y += matrix[k][s].pose.y < goal.y ? delta : -delta;

                            // достиг ли кораблик своей позиции в матрице?
                            if(goal == matrix[k][s].pose)
                            {
                                matrix[k][s].state = CLOSED_CELL;
                            }
                        }
                    }
                }

                for(int i = 0; i < MATRIX_SIZE; i++)
                {
                    for(int j = 0; j < MATRIX_SIZE; j++)
                    {
                        if(matrix[i][j].pose.x == -1)
                            cout << "(   ) ";
                        else
                            cout << '(' << matrix[i][j].pose.x << ' ' << matrix[i][j].pose.y << ')' << ' ';
                    }
                    cout << endl;
                }
                cout << endl;

                if(matrix[i][j].state != CLOSED_CELL)
                    sleep(1);

                Int32MultiArray poseMatrixMsg = matrixToArray();
                publisher.publish(poseMatrixMsg);

                if(areWeWin())
                {
                    String msg;
                    winPublisher.publish(msg);

                    cout << "\n\n\n * * * WE ARE ALIVE * * * \n\n\n" << endl;
                    sleep(5);
                    return 0;
                }

                spinOnce();
            }
        }
    }
}