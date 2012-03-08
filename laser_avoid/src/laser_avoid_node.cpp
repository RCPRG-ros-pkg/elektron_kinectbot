#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>


geometry_msgs::Twist vel;

void laserCallback(const sensor_msgs::LaserScanConstPtr& msg)
{
  double l_range, r_range;

  l_range = r_range = msg->range_max;

  for(unsigned int i = 0; i<msg->ranges.size(); i++)
  {
    if(i<(msg->ranges.size()/2))
    {
      if((l_range > msg->ranges[i]) && (msg->ranges[i] > 0.0))
        l_range = msg->ranges[i];
    } else
    { 
      if((r_range > msg->ranges[i]) && (msg->ranges[i] > 0.0))
        r_range = msg->ranges[i];
    } 
  }

  double min_dist = 0.2;
  double max_dist = 0.8;

//  double l = (1e5*r_range)/500-100;
//  double r = (1e5*l_range)/500-100;
  double l = (r_range - min_dist) / (max_dist-min_dist);
  double r = (l_range - min_dist) / (max_dist-min_dist);

  if (l > 1)
    l = 1;
  if (r > 1)
    r = 1;

  vel.linear.x = 0.1 * (r+l);

  double turnrate = 45;
  double turn = turnrate * (r-l);
  if(turn > turnrate)
    turn = turnrate;
  if(turn < -turnrate)
    turn = -turnrate;

  vel.angular.z = -turn * M_PI/180.0;

  ROS_INFO("l: %lf r: %lf x: %lf a: %lf", l_range, r_range, vel.linear.x, vel.angular.z);
  printf("l: %lf r: %lf x: %lf a: %lf\n", l, r, vel.linear.x, vel.angular.z);
  fflush(stdout);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_avoid");

  ros::NodeHandle nh;

  ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::Subscriber laser_sub = nh.subscribe("scan", 1, &laserCallback);

  ros::Rate loop_rate(25);

  vel.linear.x = 0.0;
  vel.angular.z = 0.0;

  while(ros::ok())
  {
    if (ros::isShuttingDown()) {
      vel.linear.x = 0.0;
      vel.angular.z = 0.0;
    }
    

    vel_pub.publish(vel);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

