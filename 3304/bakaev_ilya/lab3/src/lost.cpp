#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>

using namespace std;

void render(ros::Publisher& publisher, float x, float y);
void sendTransform(float x, float y);
tf::StampedTransform listenTransform();
void foundCallback(const std_msgs::String& msg);


bool found = false;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lost");

    ros::NodeHandle n;
    ros::Publisher publisher = n.advertise<visualization_msgs::Marker>("coord_topic", 100);
    ros::Subscriber subscriber = n.subscribe("found_topic", 100, foundCallback);
    ros::Rate rate(5);

    srand(time(NULL));

    // random position inside 80x80 grid
    float x = (-40) + rand() % 81; // from -40 to 40
    float y = (-40) + rand() % 81; // from -40 to 40

    bool rescuerExited = false;
    float rescuer_x = 0.0;
    float rescuer_y = 0.0;

    while(ros::ok())
    {
        if (!found)
        {
            cout<<"not found, sending transform"<<endl;
            // move in random direction
            int i = rand() % 2;
            int j = rand() % 2;

            if (i == 1 && j == 1)
                x += 1;
            else if (i == 1 && j == 0)
                x -= 1;
            else if (i == 0 && j == 1)
                y += 1;
            else if (i == 0 && j == 0)
                y -= 1;

            sendTransform(x, y);
            cout<<"Isent it!"<<endl;
        }
        else
        {
            cout<<"found!"<<endl;
            if (!rescuerExited)
            {
                tf::StampedTransform transf = listenTransform();
                rescuer_x = transf.getOrigin().x();
                rescuer_y = transf.getOrigin().y();
            }

            if (0.0 == rescuer_x && 0.0 == rescuer_y)
            {
                rescuerExited = true;
            }

            if (0.0 == x && 0.0 == y)
            {
                ROS_INFO("Rescued!");
                return 0;
            }
            else
            {
                if (fabsf(rescuer_x - x) >= fabsf(rescuer_y - y))		
                    x += (rescuer_x - x) / fabsf(rescuer_x - x);
                else
                    y += (rescuer_y - y) / fabsf(rescuer_y - y);
            }
        }

        render(publisher, x, y);
        cout<<"rendered"<<endl;
        // ros::spinOnce();
        cout<<"spined"<<endl;
        rate.sleep();
        cout<<"sleeped"<<endl;
    }

    cout<<"rosnot ok..."<<endl;
    return 0;
}

void render(ros::Publisher& publisher, float x, float y)
{
    visualization_msgs::Marker msg;
    msg.header.frame_id = "/world";
    msg.header.stamp = ros::Time::now();
    msg.ns = "";
    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::POINTS;
    msg.id = 0;
    msg.scale.x = 0.5;
    msg.scale.y = 0.5;
    msg.color.r = 1.0;
    msg.color.g = 0.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    msg.points.push_back(p);
    publisher.publish(msg);
}

void sendTransform(float x, float y)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/lost"));
}

tf::StampedTransform listenTransform()
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    try
    {
        ros::Time now = ros::Time::now();
        listener.waitForTransform("/world", "/rescue", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("/world", "/rescue", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
    return transform;
}

void foundCallback(const std_msgs::String& msg)
{
    ROS_INFO_STREAM("Rescuer says: " << msg.data);
    found = true;
}
