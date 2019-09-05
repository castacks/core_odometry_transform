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
  convert_odometry_to_stabilized_transform = pnh->param("convert_odometry_to_stabilized_transform", false);
  transform_odometry_to_new_frame = pnh->param("transform_odometry_to_new_frame", false);
  odometry_output_type = static_cast<OdometryOutputType>(pnh->param("odometry_output_type", (int)NONE));
  restamp_now = pnh->param("restamp_now", false);
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
  ros::Time stamp = odom.header.stamp;
  if(restamp_now)
    stamp = ros::Time::now();
    
  if(convert_odometry_to_transform){
    tf::Transform odom_tf(tflib::to_tf(odom.pose.pose.orientation), tflib::to_tf(odom.pose.pose.position));
    tf::StampedTransform odom_stamped_tf(odom_tf, stamp, odom.header.frame_id, transform_name);
    broadcaster->sendTransform(odom_stamped_tf);
  }

  if(convert_odometry_to_stabilized_transform){
    tf::Transform odom_tf(tflib::to_tf(odom.pose.pose.orientation), tflib::to_tf(odom.pose.pose.position));
    tf::Transform stabilized_tf = tflib::get_stabilized(odom_tf);
    tf::StampedTransform odom_stamped_tf(stabilized_tf, stamp, odom.header.frame_id, transform_name+"_stabilized");
    broadcaster->sendTransform(odom_stamped_tf);
  }

  if(odometry_output_type == TRANSFORMED){
    try{
      if(transform_odometry_to_new_frame){
	nav_msgs::Odometry out_odom = tflib::transform_odometry(listener, odom, new_frame_id, new_child_frame_id, ros::Duration(0.1));
	out_odom.header.stamp = stamp;
	odometry_pub.publish(out_odom);
      }
    }
    catch(tf::TransformException& ex){
      ROS_ERROR_STREAM("TransformException while transforming odometry: " << ex.what());
    }
  }
  else if(odometry_output_type == RESTAMPED){
    nav_msgs::Odometry out_odom = odom;
    out_odom.header.stamp = stamp;
    out_odom.header.frame_id = new_frame_id;
    out_odom.child_frame_id = new_child_frame_id;
    odometry_pub.publish(out_odom);
  }
}

OdometryTransform::~OdometryTransform(){
}

BaseNode* BaseNode::get(){
  OdometryTransform* odometry_transform = new OdometryTransform("OdometryTransform");
  return odometry_transform;
}
