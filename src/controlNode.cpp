#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

#include <unistd.h>
#include <cstdlib>
#include <sstream>
#include <vector>

using namespace std;


ros::Publisher com_pub;
ros::Subscriber com_sub;
ros::Publisher vel_pub;
ros::Subscriber vel_sub;


struct ControlCommand
{
  inline ControlCommand() {roll = pitch = yaw = gaz = ms = 0;}
  inline ControlCommand(double roll, double pitch, double yaw, double gaz, int ms)
  {
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
    this->gaz = gaz;
    this->ms = ms;
  }
  double yaw, roll, pitch, gaz;
  int ms;
};

struct point3D{
  inline point3D() {x = rand()%10, y = rand()%10, z = rand()%10}
  inline point3D(double x, double y, double z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  double x, y, z;
};

struct point2D{
  inline point3D() {x = rand()%10, y = rand()%10}
  inline point3D(double x, double y)
  {
    this->x = x;
    this->y = y;
  }
  double x, y;
};


vector<point3D> updateMap();
vector<point2D> flattenMap(map3D);
vector< vector<int> > gridFrom2D(map2D);
void publishMoveControl(ControlCommand cmd, int ms);
void publishStringCommand(std::string c);
void comCallback(const std_msgs::String::ConstPtr& msg);



int main(int argc, char **argv)
{

  ros::init(argc, argv, "control");
  ros::NodeHandle nh_;


  com_pub = nh_.advertise<std_msgs::String>("tum_ardrone/com", 1000);
  com_sub = nh_.subscribe("tum_ardrone/com", 1000, comCallback);

  vel_pub = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
  //vel_sub = nh_.subscribe(nh_.resolveName("cmd_vel"),50, &RosThread::velCb, this);
  
  ros::Rate loop_rate(10); //hz


  int sectorSize = 2; // how big is a sector relative to the coordinates of the world

  vector<point3D> map3D(8); // Point Cloud in world coordinates
  map3D.push_back(point3D(1,1.5,3));
  map3D.push_back(point3D(1,1.6,4));
  map3D.push_back(point3D(1,3,2));
  map3D.push_back(point3D(1,5,1));
  map3D.push_back(point3D(1,7,1));
  map3D.push_back(point3D(3,7,5));
  map3D.push_back(point3D(3,9,4));
  map3D.push_back(point3D(3,11,4));

  map2D = flattenMap(map3D);
  ROS_INFO("map2D size = ");
  ROS_INFO("%i", map2D.size());

  gridMap = gridFrom2D(map2D);


  int count = 0;
  while (ros::ok()){
   
    //If not flying --> Takeoff
    //initialize ptam

    //if map isnt good, take another keyframe
    //if map still isnt good, restart PTAM
    //if map is good, make 2d map
      // make 2d map into gridMap

    //**FOR NOW**
    //make a fake map


    // look for nearest unoccupied sector and go to it
      // Look through gridMap
        //if sector != 1 
          //(roll, pitch, gaz, yaw, duration) = calculateTrajectory()
          // publishMoveControl(CommandControl(roll, pitch, gaz, yaw), duration);

    for (i=0; i < gridMap.size(); i++){
      if gridMap[i] != 1){
        // cmd = calculateTrajectory();
        // publishMoveControl(CommandControl(roll, pitch, gaz, yaw, duration);
      }
    }

    ROS_INFO("RELOOP");

    publishMoveControl(ControlCommand(.02, .02, 1, 0, 500));


    map3D = updateMap();
    map2D = flattenMap(map3D);
    gridMap = gridFrom2D(map2D);



    // publishMoveControl(ControlCommand(0,0,0,0));
    // publishStringCommand("c moveByRel 2 2 0 45");

    // if (count == 5){
    //   publishMoveControl(ControlCommand(5, 5, 1, 0));
    //   ROS_INFO("%s", "BOOOOOOM Bayby!");
    // }

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}


void updateMap()
{
  vector<point3D> map3D(8); // Point Cloud in world coordinates
  map3D.push_back(point3D());
  map3D.push_back(point3D());
  map3D.push_back(point3D());
  map3D.push_back(point3D());
  map3D.push_back(point3D());
  map3D.push_back(point3D());
  map3D.push_back(point3D());
  map3D.push_back(point3D());

  return map3D;
}

vector<point2D> flattenMap(map3D)
{
  vector<point2D> map2D;

  for (int i = 0; i < map3D.size(); i++){ // fill flattened map
    point2D.x = map3D(i).x;
    point2D.y = map3D(i).y;

    map2D.push_back(point2D);
  }

  return map2D;
}

vector< vector<int> > gridFrom2D(map2D)
{
  vector< vector<int> > gridMap;

  // Biggest number in x direction is the number of columns
  // biggest number in Y direction is the number of rows
      // This includes the values of x and y of the drones position

  int upperBoundX = lowerBoundX = upperBoundY = lowerBoundY = 0;
  for point2D in map2D{
    if point2D.x > upperBoundX{
      upperBoundX = point2D.x;
    }
    if point2D.x < lowerBoundX{
      lowerBoundX = point2D.x;
    }
    if point2D.y > upperBoundY{
      upperBoundY = point2D.y;
    }
    if point2D.y < lowerBoundY{
      lowerBoundY = point2D.y;
    }
  }

  //dronePose = updateDronePose();

  if dronePose.x > upperBoundX{
    upperBoundX = dronePose.x;
  }
  if dronePose.x < lowerBoundX{
    lowerBoundX = dronePose.x;
  }
  if dronePose.y > upperBoundY{
    upperBoundY = dronePose.y;
  }
  if dronePose.y < lowerBoundY{
    lowerBoundY = dronePose.y;
  }

  int numCol = upperBoundX - lowerBoundX;
  int numRow = upperBoundY - lowerBoundY;

  gridMap.resize( numCol, vector<int>(numRow , 1));

  if 


  return gridMap;
}

void publishMoveControl(ControlCommand cmd, int ms)
{
  geometry_msgs::Twist cmdT;

  cmdT.angular.z = -cmd.yaw;
  cmdT.linear.z = cmd.gaz;
  cmdT.linear.x = -cmd.pitch;
  cmdT.linear.y = -cmd.roll;

  cmdT.angular.x = cmdT.angular.y = 1;

  int count = 0;
  while (count < ms){ 
    vel_pub.publish(cmdT);
    usleep(1000);
    count++;
  }
}

void publishStringCommand(std::string c)
{
  std_msgs::String msg;
  std::stringstream ss;

  ss << c.c_str(); ;
  msg.data = ss.str();

  ROS_INFO("%s", msg.data.c_str());
  ROS_INFO("Check");

  com_pub.publish(msg);
}

void comCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
