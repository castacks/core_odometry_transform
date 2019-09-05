#ifndef _ODOMETRY_TRANSFORM_H_
#define _ODOMETRY_TRANSFORM_H_

#include <base/BaseNode.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class OdometryTransform : public BaseNode {
private:
  enum OdometryOutputType {NONE, TRANSFORMED, RESTAMPED};
  
  // parameters
  bool convert_odometry_to_transform,
    convert_odometry_to_stabilized_transform,
    transform_odometry_to_new_frame,
    restamp_now;
  OdometryOutputType odometry_output_type;
  std::string transform_name, new_frame_id, new_child_frame_id;

  // subscribers
  ros::Subscriber odometry_sub;
  tf::TransformListener* listener;
  
  // publishers
  ros::Publisher odometry_pub;
  tf::TransformBroadcaster* broadcaster;

  // callbacks
  void odometry_callback(nav_msgs::Odometry odom);
  
public:
  OdometryTransform(std::string node_name);
  
  virtual bool initialize();
  virtual bool execute();
  virtual ~OdometryTransform();
  
};


#endif
