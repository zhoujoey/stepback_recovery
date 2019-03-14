/*********************************************************************
 * copyright: Tencent Technology (Beijing) Co. ltd
 * author: williszhou@tencent.com
*********************************************************************/
#include <stepback_recovery/stepback_recovery.h>
#include <pluginlib/class_list_macros.h>

//register this planner as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(stepback_recovery::StepbackRecovery, nav_core::RecoveryBehavior)

namespace stepback_recovery {
StepbackRecovery::StepbackRecovery(): global_costmap_(NULL), local_costmap_(NULL), 
  tf_(NULL), initialized_(false), world_model_(NULL) {} 

void StepbackRecovery::initialize(std::string name, tf::TransformListener* tf,
    costmap_2d::Costmap2DROS* global_costmap, costmap_2d::Costmap2DROS* local_costmap){
  if(!initialized_){
    name_ = name;
    tf_ = tf;
    global_costmap_ = global_costmap;
    local_costmap_ = local_costmap;

    //get some parameters from the parameter server
    ros::NodeHandle private_nh("~/" + name_);
    private_nh.param("frequency", frequency_, 10.0);
    private_nh.param("back_vel_x", back_vel_x_, -0.2);
    private_nh.param("acc_lim_vel", acc_lim_x_, 0.5);
    private_nh.param("safty_padding", safty_padding_, 2.0);
    private_nh.param("stepback_dist", stepback_dist_, 2.0);
    private_nh.param("tolerance", tolerance_, 0.1);
    world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

    initialized_ = true;
  }
  else{
    ROS_ERROR("You should not call initialize twice on this object, doing nothing");
  }
}

StepbackRecovery::~StepbackRecovery(){
  delete world_model_;
}

void StepbackRecovery::runBehavior(){
  if(!initialized_){
    ROS_ERROR("This object must be initialized before runBehavior is called");
    return;
  }
  if(global_costmap_ == NULL || local_costmap_ == NULL){
    ROS_ERROR("The costmaps passed to the StepbackRecovery object cannot be NULL. Doing nothing.");
    return;
  }
  ROS_WARN("Stepback recovery behavior started.");

  ros::Rate r(frequency_);
  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  
  tf::Stamped<tf::Pose> global_pose;
  double min_stop_dist = safty_padding_ * (back_vel_x_ / acc_lim_x_)* back_vel_x_;
  double sim_step = (1.0 / frequency_)* fabs(back_vel_x_);
  double safty_check_dist = sim_step + min_stop_dist;
  double back_dist = 0.0;
  while(n.ok()){
    double sim_dist = 0.0;
    while(sim_dist < safty_check_dist){
      local_costmap_->getRobotPose(global_pose);
      /*
      //todo : get global pose back sim_dist of base_link
      tf::TransformListener transform_listener;
      try{
        tf::Stamped<tf::Pose> global_pose;
        tf::Stamped<tf::Pose> local_pose(tf::Pose(tf::Quaternion(0, 0, 0, 1), tf::Vector3(-1 * sim_dist, 0, 0.0)), ros::Time(0), "base_link");
        transform_listener.transformPose ("map", local_pose, global_pose);
      }catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
      }
      */
      double x = global_pose.getOrigin().x()- sim_dist , y = global_pose.getOrigin().y();
      double theta = tf::getYaw(global_pose.getRotation());
      double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if(footprint_cost < 0.0){
        geometry_msgs::Twist zero_vel;
        vel_pub.publish(zero_vel);
        r.sleep();  
        ROS_ERROR("Stepback recovery can't stepback because there is a potential collision. Cost: %.2f", footprint_cost);
        return;
      }
      sim_dist += sim_step;
    }
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = back_vel_x_;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub.publish(cmd_vel);
    back_dist += fabs(cmd_vel.linear.x) * (1.0 / frequency_);;
    if(back_dist > stepback_dist_)
    {
      ROS_INFO("finish step back recovery");
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      vel_pub.publish(cmd_vel);
      ros::Duration(1.0).sleep();   
      return;
    }
    r.sleep();
  }
}
};