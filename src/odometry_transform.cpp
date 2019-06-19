#include <base/BaseNode.h>
#include <string>
#include <core_odometry_transform/odometry_transform.h>
#include <tflib/tflib.h>

OdometryTransform::OdometryTransform(std::string node_name)
  : BaseNode(node_name){
}

bool OdometryTransform::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();
  
  // init parameters
  convert_odometry_to_transform = pnh->param("convert_odometry_to_transform", false);
  transform_odometry_to_new_frame = pnh->param("transform_odometry_to_new_frame", false);
  transform_name = pnh->param("transform_name", std::string());
  new_frame_id = pnh->param("new_frame_id", std::string());
  new_child_frame_id = pnh->param("new_child_frame_id", std::string());
  
  // init subscribers
  odometry_sub = nh->subscribe("input_odometry", 10, &OdometryTransform::odometry_callback, this);
  listener = new tf::TransformListener();
  
  // init publishers
  odometry_pub = nh->advertise<nav_msgs::Odometry>("odometry", 10);
  broadcaster = new tf::TransformBroadcaster();
  
  return true;
}

bool OdometryTransform::execute(){
  
  return true;
}

void OdometryTransform::odometry_callback(nav_msgs::Odometry odom){
  try{
    tf::Transform odom_tf(tflib::to_tf(odom.pose.pose.orientation), tflib::to_tf(odom.pose.pose.position));
    tf::StampedTransform odom_stamped_tf(odom_tf, odom.header.stamp, odom.header.frame_id, transform_name);
    broadcaster->sendTransform(odom_stamped_tf);
    
    nav_msgs::Odometry out_odom = tflib::transform_odometry(listener, odom, new_frame_id, new_child_frame_id, ros::Duration(0.1));
    odometry_pub.publish(out_odom);
  }
  catch(tf::TransformException& ex){
    ROS_ERROR_STREAM("TransformException while transforming odometry: " << ex.what());
  }
}

OdometryTransform::~OdometryTransform(){
}

BaseNode* BaseNode::get(){
  OdometryTransform* odometry_transform = new OdometryTransform("OdometryTransform");
  return odometry_transform;
}
