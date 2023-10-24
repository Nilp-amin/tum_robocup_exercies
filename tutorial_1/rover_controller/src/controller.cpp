#include <rover_controller/controller.h>

namespace rover_controller
{

  Controller::Controller() : 
    has_goal_(false),
    map_frame_("map"),
    rover_frame_("rover"),
    rover_radius_(0.8)
  {
  }

  Controller::~Controller()
  {
  }

  bool Controller::initialize(ros::NodeHandle& nh)
  {
    // setup ros connections

    //#>>>>TODO: advertise a geometry_msgs::Twist to replace the previous keyboard topic
    command_pub_ = nh.advertise<geometry_msgs::Twist>("/key_vel", 10);

    field_pub_ = nh.advertise<geometry_msgs::PoseArray>("/field", 10); //#>>>>TODO: advertise a geometry_msgs::PoseArray with name "/field"
    //#>>>> Hint: see http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseArray.html

    goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, 
                             &Controller::goalCallback,
                             this);

    //#>>>>TODO: Subscribe to the obstacles msg published by the package object_server                     
    obstacles_sub_ = nh.subscribe("/obstacles", 1, 
                             &Controller::obstacleCallback, this);

    // get the parameters of this node (see config.yaml)

    std::vector<double> v;
    double d;

    if(!ros::param::get("attractor_gain", v))
      return false;
    K_attractor_ = Eigen::Vector3d(v[0], v[1], v[2]).asDiagonal();

    if(!ros::param::get("repulsive_gain", d))
      return false;
    k_repulsive_ = d;

    if(!ros::param::get("lambda_repulsive", d))
      return false;
    lambda_repulsive_ = d;
  
    if(!ros::param::get("vortex_gain", d))
      return false;
    k_vortex_ = d;

    if(!ros::param::get("lambda_vortex", d))
      return false;
    lambda_vortex_ = d;

    // setup variables
    createVectorField(field_points_w_);

    prev_time_ = ros::Time::now();
    return true;
  }

  void Controller::update(const ros::Time& time, const ros::Duration durtation)
  {
    // get the rover pose in the map frame
    Eigen::Vector3d rover_pos_w = roverPos();

    // Rotation rover with respect to world
    Eigen::Matrix3d R_b_w = Eigen::Matrix3d::Identity();
    R_b_w.topLeftCorner(2,2) = Eigen::Rotation2Dd(rover_pos_w.z()).matrix();

    // compute the thrust vector of the rover
    Eigen::Vector3d thrust_w = computeForce(
      obstacles_, goal_pos_w_, rover_pos_w);

    // transform into body frame for the rover command
    Eigen::Vector3d thrust_b = R_b_w.transpose()*thrust_w;

    // send to the rover
    publishCommand(thrust_b);

    // visualize the field at low frequency
    if(time - prev_time_ > ros::Duration(1./5.))
    {
      prev_time_ = time;
      visualizerVectorField(obstacles_, goal_pos_w_, field_points_w_);
    }
  }

  Eigen::Vector3d Controller::computeForce(
    std::vector<Obstacle> obstacles, const Eigen::Vector3d& goal, 
    const Eigen::Vector3d& pos)
  {
    // compute the thrust vector F with respect to the world frame as:
    // F = F_attractor + F_repulsive + F_vortex

    Eigen::Vector3d F = Eigen::Vector3d::Zero();  
    if(has_goal_)
    {
      // pull to goal
      F += goalAttractorForce(goal_pos_w_, pos);
    }

    // avoid collision
    F += obstacleRepulsiveForce(obstacles_, pos);

    // add vortex/curl field
    F += obstacleVortexForce(obstacles_, pos);
    
    return F;
  }

  Eigen::Vector3d Controller::goalAttractorForce(
    const Eigen::Vector3d& goal, const Eigen::Vector3d& pos)
  {
    Eigen::Vector3d e;
    e = goal - pos; 
    if (e.norm() > 1)
    {
      e.normalize();
    }

    return K_attractor_ * e;
  }

  Eigen::Vector3d Controller::obstacleRepulsiveForce(
    std::vector<Obstacle> obstacles, const Eigen::Vector3d& pos)
  {
    Eigen::Vector3d force = Eigen::Vector3d::Zero();

    Eigen::Vector3d v;
    double lambda = lambda_repulsive_;
    for(size_t i = 0; i < obstacles.size(); ++i)
    {
      v << obstacles[i].pos - pos; //#>>>>TODO: 2d distance of obstacle and rover (note z component is zero!)
      double v_norm = v.norm(); //#>>>>TODO: norm of v
      double a = obstacles[i].radius + rover_radius_; //#>>>>TODO: radius obstacle + radius rover
      double d = std::max(0.0, v_norm - a); /*#>>>>TODO: the surface distance rover obstacle*/;
    
      double f = k_repulsive_*exp(-lambda*d); //#>>>>TODO: force magnitude
    
      force -= f*v; //#>>>>TODO: add up the repulsive force vector for this obstacle
    }
    return force;
  }

  Eigen::Vector3d Controller::obstacleVortexForce(
    std::vector<Obstacle> obstacles, const Eigen::Vector3d& pos)
  {
    Eigen::Vector3d force = Eigen::Vector3d::Zero();

    //#>>>>OPTIONAL: Uncomment this part to implement the vortex force

    // // get the heading vector of the rover
    // Eigen::Vector3d n_rover(std::sin(pos[2]), std::cos(pos[2]), 0.0);

    // Eigen::Vector3d v;
    // Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    // for(size_t i = 0; i < obstacles.size(); ++i)
    // {
    //   // get the vector rover to obstacle 
    //   v << (obstacles[i].pos - pos).head(2), 0;
    //   Eigen::Vector3d n_obs = v.normalized();

    //   // the sign of the dot product between obstacle vector and heading
    //   // tells us if an object is to the right or to the left
    //   double alpha = n_rover.dot(n_obs);
    //   if(alpha < 0)
    //   { 
    //     //#>>>>TODO: set the upper left 2x2 corner of the rotation matrix R
    //     //#>>>>TODO: to create - pi/2 = - 90 degree rotation
    //     R.topLeftCorner(2,2) = //#>>>>TODO: 
    //   }
    //   else
    //   {
    //     //#>>>>TODO: set the upper left 2x2 corner of the rotation matrix R
    //     //#>>>>TODO: to create + pi/2 = + 90 degree rotation
    //     R.topLeftCorner(2,2) = //#>>>>TODO: 
    //   }

    //   // compute vortex field
    //   double v_norm = v.norm();
    //   double a = obstacles[i].radius + rover_radius_;
    //   double d = std::max(0.0, d - a);
    //   double f = k_vortex_*std::exp(-lambda_vortex_*d);
    //   force -= f * R * v.normalized();
    // }
    return force;
  }

  void Controller::publishCommand(const Eigen::Vector3d& thrust)
  {
    geometry_msgs::Twist msg;
    msg.linear.x = thrust.x();
    msg.linear.y = thrust.y();
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = thrust.z();
    command_pub_.publish(msg);
  }

  void Controller::createVectorField(std::vector<Eigen::Vector3d>& points)
  {
    // create a 10x10 mesh with evenly spaced points
    size_t n = 100;
    double len = 10.0;
    double dx = len / n;
    double dy = len / n;
    Eigen::Vector3d p;
    points.reserve(n*n);
    for(double x = -5.0; x < 5.0; x+=dx)
    {
      for(double y = -5.0; y < 5.0; y+=dx)
      {
        p << x, y, 0.0;
        points.push_back(p);
      }
    }
  }

  void Controller::visualizerVectorField(
    std::vector<Obstacle> obstacles, 
    const Eigen::Vector3d& goal,
    const std::vector<Eigen::Vector3d>& points)
  {
    geometry_msgs::PoseArray field_msg;
    field_msg.header.frame_id = map_frame_;
    field_msg.header.stamp = ros::Time::now();

    // evaluate the field at the points, obtain the directions
    field_msg.poses.resize(points.size());
    for(size_t i = 0; i < points.size(); ++i)
    {
      const Eigen::Vector3d& p = points[i];
      Eigen::Vector3d v = computeForce(obstacles, goal, p).normalized();
      Eigen::Quaterniond Q = Eigen::Quaterniond::FromTwoVectors(v, p);

      geometry_msgs::Pose& pose = field_msg.poses[i];
      pose.position.x = p.x();
      pose.position.y = p.y();
      pose.position.z = p.z();

      pose.orientation = tf::createQuaternionMsgFromYaw( std::atan2(v[1], v[0] ));
    }
    field_pub_.publish(field_msg);
  }

  //----------------------------------------------------------------------------
  // tfs

  Eigen::Vector3d Controller::roverPos()
  {
    // get the current rover pose in the map frame
    // returns a Eigen::Vector3d containing [x,y,theta]
    
    tf::StampedTransform transform;
    try {
      listener_.lookupTransform(
        "map", "rover", ros::Time(0), transform);
    }
    catch (tf::TransformException ex) 
    {
      transform.setIdentity();
    }

    // extract the position 
    Eigen::Vector3d pos;
    pos.x() = transform.getOrigin().x();
    pos.y() = transform.getOrigin().y();

    // extract the theta angle of the rover
    Eigen::Quaterniond Q(
      transform.getRotation().getW(),
      transform.getRotation().getX(),
      transform.getRotation().getY(),
      transform.getRotation().getZ());
    Eigen::Matrix2d M = Q.toRotationMatrix().topLeftCorner(2,2);
    pos.z() = Eigen::Rotation2Dd(M).angle();

    return pos;
  }

  //----------------------------------------------------------------------------
  // callbacks

  void Controller::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    // note: the goal is given in the map frame and should be stored in the 
    // member variable goal_pos_w_ = [pos_x, pos_y, theta] \in R^3

    has_goal_ = true;
    goal_pos_w_.x() = msg->pose.position.x;
    goal_pos_w_.y() = msg->pose.position.y;

    Eigen::Quaterniond Q_b_w(
      msg->pose.orientation.w,
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z);

    // Next we use Eigen to convert the Quaternion into a RotationMatrix and 
    // extract the theta angle from its upper 2x2 part
    Eigen::Matrix2d M = Q_b_w.toRotationMatrix().topLeftCorner(2,2);
    goal_pos_w_.z() = Eigen::Rotation2Dd(M).angle();

    // print the new goal
    ROS_INFO_STREAM_THROTTLE(0.5, "RoverController::goalCallback=" << goal_pos_w_.transpose());
  }

  void Controller::obstacleCallback(const object_msgs::ObjectsConstPtr& msg)
  {
    // note: all obstacle positions (x,y,z) are stored inside the Objects message
    // This callback copies the position into the member variable obstacles_
    // Hint: See the Obstacle struct definition in the header file.

    // Hint: The message obstacle is defined by me inside the package:
    // utilites/object_msgs/msg/Object.msg

    obstacles_.resize(msg->objects.size());
    for(size_t i = 0; i < msg->objects.size(); ++i)
    {
      obstacles_[i].pos << msg->objects[i].pose.position.x, msg->objects[i].pose.position.y, msg->objects[i].pose.position.z;
      obstacles_[i].radius = msg->objects[i].radius.data;
    }
  }

}