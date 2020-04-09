#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  uint32_t shape = visualization_msgs::Marker::CUBE;
  uint32_t marker_counter = 0;

  while (ros::ok())
  {
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Wait...create a subscriber to marker");
      sleep(1);
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;

   
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

  
    switch (marker_counter) {
      case 0:
        {
          ROS_INFO("Publishing object pick marker zone at coordinate (0,3)");

          marker.action = visualization_msgs::Marker::ADD;
          n.getParam("/pick_up_loc/tx", marker.pose.position.x);
          n.getParam("/pick_up_loc/ty", marker.pose.position.y);
          n.getParam("/pick_up_loc/tz", marker.pose.position.z);
          n.getParam("/pick_up_loc/qx", marker.pose.orientation.x);
          n.getParam("/pick_up_loc/qy", marker.pose.orientation.y);
          n.getParam("/pick_up_loc/qz", marker.pose.orientation.z);
          n.getParam("/pick_up_loc/qw", marker.pose.orientation.w);

          marker_pub.publish(marker);
          marker_counter++;
          break;
        }

        case 1:  
        {
          sleep(5);
          ROS_INFO("Hiding object pick marker zone");
          marker.action = visualization_msgs::Marker::DELETE;
          marker_counter++;
          break;
        }





        case 2:  
        {
          sleep(5);
          ROS_INFO("Adding drop zone marker at coordinate (9,3)");
          marker.action = visualization_msgs::Marker::ADD;

          n.getParam("/drop_off_loc/tx", marker.pose.position.x);
          n.getParam("/drop_off_loc/ty", marker.pose.position.y);
          n.getParam("/drop_off_loc/tz", marker.pose.position.z);
          n.getParam("/drop_off_loc/qx", marker.pose.orientation.x);
          n.getParam("/drop_off_loc/qy", marker.pose.orientation.y);
          n.getParam("/drop_off_loc/qz", marker.pose.orientation.z);
          n.getParam("/drop_off_loc/qw", marker.pose.orientation.w);
          marker_pub.publish(marker);
          marker_counter++;
          break;
        }

        case 3: 
        {
          ROS_INFO("Finished...");
          break;
        }

        default:
          ROS_INFO("ERROR: Object marker fix");
    }

    r.sleep();
  }
}
