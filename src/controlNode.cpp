
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

/*
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

struct DronePose{
  double x,  y, z, yaw;
}

struct point3D{
  inline point3D() {x = rand()%10; y = rand()%10; z = rand()%10;}
  inline point3D(double x, double y, double z)
  {
    this->x = x;
    this->y = y;
    this->z = z;
  }
  double x, y, z;
};

struct point2D{
  inline point3D() {x = rand()%10; y = rand()%10;}
  inline point3D(double x, double y)
  {
    this->x = x;
    this->y = y;
  }
  double x, y;
};

struct Sector
{
  inline ControlCommand() {x = y = status = 0;}
  inline ControlCommand(int x, int y, int status)
  {
    this->x = x;
    this->y = y;
    this->status = status;
  }
  int x, y, status;
};
*/

// vector<point3D> updateMap();
// vector<point2D> flattenMap(map3D);
// vector< vector<int> > gridFrom2D(map2D);
// void publishMoveControl(ControlCommand cmd);
// void publishStringCommand(std::string c);
void comCallback(const std_msgs::String::ConstPtr& msg);



int main(int argc, char **argv)
{

  ros::init(argc, argv, "bing_ardrone");
  ros::NodeHandle nh_;

  com_pub = nh_.advertise<std_msgs::String>("tum_ardrone/com", 1000);
  com_sub = nh_.subscribe("tum_ardrone/com", 1000, comCallback);
  vel_pub = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("cmd_vel"),1);
  //vel_sub = nh_.subscribe(nh_.resolveName("cmd_vel"),50, &RosThread::velCb, this);
  
  ros::Rate loop_rate(10); //hz

  
  // map3D = updateMap();
  // map2D = flattenMap(map3D);
  // ROS_INFO("map2D size = ");
  // ROS_INFO("%i", map2D.size());
  // gridMap = gridFrom2D(map2D);
  

  int count = 0;
  while (ros::ok()){
  

    // If not flying --> Takeoff
    // initialize ptam
    // fly up and down to get good scale of map

    // if map isnt good, take another keyframe
    // if map still isnt good, restart PTAM
    // if map is good, make 2d map
    //   make 2d map into gridMap

    // **FOR NOW**
    // make a fake map


    // look for nearest unoccupied sector and go to it
    //   Look through gridMap
    //     if sector != 1 
    //       (roll, pitch, gaz, yaw, duration) = calculateTrajectory()
    //       publishMoveControl(CommandControl(roll, pitch, gaz, yaw), duration);

    // controlDecision();

    // //publishMoveControl(ControlCommand(.02, .02, 1, 0, 500));

    // ROS_INFO("Updating Map count = " << count);

    // map3D = updateMap();
    // map2D = flattenMap(map3D);
    // gridMap = gridFrom2D(map2D);


    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  return 0;
}

/*
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

  for (int i = 0; i < map3D.size(); i+=3){ // fill flattened map
    point2D.x = map3D(i);
    point2D.y = map3D(i+1);

    map2D.push_back(point2D);
  }

  return map2D;
}

vector< vector<int> > gridFrom2D(map2D)
{
  vector< vector<int> > gridMap;

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

  // Biggest number in x direction is the number of columns
  // Biggest number in Y direction is the number of rows
      // This includes the values of x and y of the drones position
  int numCol = upperBoundX - lowerBoundX;
  int numRow = upperBoundY - lowerBoundY;

  gridMap.resize( numCol, vector<int>(numRow , 1));

  int sectorSize = 1; //(i.e. gridMap scale is 1:1 with the world coords)
    //  temporary - sectorSize will be changed if grid resolution is too high(too big computationally)

  // *******
  // Upper Left Grid Map Origin

  // gridMap[0][0] is upper left corner (-x,y) -> (lowerBoundX, upperBoundY)
  // gridMap[upperBoundX-1][lowerBoundY-1] is bottom right corner of gridMap

  //(0,0) on map is gridMap[lowerBoundX][upperBoundY]
  //(x,y) on map is gridMap[lowerBoundX+x][upperBoundY+y]
  // *******


  for mapPoints in map2D{
    for (i=0; i < gridMap.size(); i++)
    {
      for (j=0; j < gridMap[i].size(); j++)
      {
        if (mapPoint.y < sector.y + sectorSize) & 
            (mapPoint.y > sector.y - sectorSize) & 
             (mapPoint.x < sector.x + sectorSize) & 
              (mapPoint.x > sector.x - sectorSize)
        {

          gridMap[i][j] = 2; //occupied
        
          break;
        }
      }
    }
  }

  // if points in front of the drone are closer than 4 feet
  //   turn away from that point

  return gridMap;
}


void controlDecision(dronePose pose, vector< vector<int> > gridMap)
{
  Sector targetSector();

  //look through gridMap
  for (i=0; i < gridMap.size(); i++)
  {
    for (j=0; j < gridMap[i].size(); j++)
    {
      if (gridMap[i][j] == 0) // unoccupied == 0 
      {
        // find the open sector that is closest to drone
        Sector currentSector = Sector(i, j, gridMap[i][j]);
        if (abs(currentSector.x-dronePose.x) < abs(targetSector.x-dronePose.x) && 
            abs(currentSector.y-dronePose.y) < abs(targetSector.y-dronePose.y))
        {
          Sector targetSector = currentSector;

        }
      }
    }
  }

  // move to that sector
  publishStringCommand("c goto %i %i %i" ,targetSector.x, targetSector.y, 4, dronePose.yaw radians(arctan(x/y)));
}

void publishMoveControl(ControlCommand cmd)
{
  geometry_msgs::Twist cmdT;

  cmdT.angular.z = -cmd.yaw;
  cmdT.linear.z = cmd.gaz;
  cmdT.linear.x = -cmd.pitch;
  cmdT.linear.y = -cmd.roll;

  cmdT.angular.x = cmdT.angular.y = 1;

  int count = 0;
  while (count < cmd.ms){ 
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
*/
void comCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
