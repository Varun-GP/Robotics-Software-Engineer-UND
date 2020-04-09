#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
// #include <nav_msgs/Odometry.h>
#include <std_msgs/UInt8.h>

// bool marker_reach_state = false;
uint8_t goal_reach_state = 0;

void goalReachCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   goal_reach_state = msg->data;
   return;
}




int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/goal_reached", 1, goalReachCallback);
  bool done = false;


  uint32_t shape = visualization_msgs::Marker::CUBE;

  ROS_INFO("Subscribed to desired goal");

  while (ros::ok()) {
    ros::spinOnce();
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    switch (goal_reach_state)
    {
      case 0: 
        {
          marker.action = visualization_msgs::Marker::ADD;
          n.getParam("/pick_up_loc/tx", marker.pose.position.x);
          n.getParam("/pick_up_loc/ty", marker.pose.position.y);
          n.getParam("/pick_up_loc/tz", marker.pose.position.z);
          n.getParam("/pick_up_loc/qx", marker.pose.orientation.x);
          n.getParam("/pick_up_loc/qy", marker.pose.orientation.y);
          n.getParam("/pick_up_loc/qz", marker.pose.orientation.z);
          n.getParam("/pick_up_loc/qw", marker.pose.orientation.w);
          break;
        } 

        case 1: 
          {
            sleep(2);
        
            marker.action = visualization_msgs::Marker::DELETE;
            break;
          } 

        case 2: 
          {
            marker.action = visualization_msgs::Marker::DELETE;
            break;
          }

        case 3:   
          {
            sleep(5);
            marker.action = visualization_msgs::Marker::ADD;
            n.getParam("/drop_off_loc/tx", marker.pose.position.x);
            n.getParam("/drop_off_loc/ty", marker.pose.position.y);
            n.getParam("/drop_off_loc/tz", marker.pose.position.z);
            n.getParam("/drop_off_loc/qx", marker.pose.orientation.x);
            n.getParam("/drop_off_loc/qy", marker.pose.orientation.y);
            n.getParam("/drop_off_loc/qz", marker.pose.orientation.z);
            n.getParam("/drop_off_loc/qw", marker.pose.orientation.w);
            done = true;
            break;
          }

    }


    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    marker_pub.publish(marker);

    if (done) {
      ROS_INFO("******  Goal Reached  ******");
      sleep(7);
      return 0;
      }

    r.sleep();
  } 

  return 0;
}
