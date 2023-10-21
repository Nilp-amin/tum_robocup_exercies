#include <object_server/mars_environment.h>
#include <random>

namespace object_server
{
  MarsEnvironment::MarsEnvironment() : 
    map_frame_("map")
  {
  }

  MarsEnvironment::~MarsEnvironment()
  {
  }

  bool MarsEnvironment::initialize(ros::NodeHandle &nh)
  {
    // call the init of the base class
    Base::initialize(nh);

    // create ros connections for communication
    terrain_pub_ = nh.advertise<visualization_msgs::MarkerArray>("terrain", 1);
    obstacles_pub_ = nh.advertise<object_msgs::Objects>("obstacles", 1);

    add_objects_srv_ = nh.advertiseService("add_objects", &MarsEnvironment::addObstaclesHandler, this);

      // setup the terrain map
    terrain_marker_.markers.push_back(object_server::create_mesh_marker(
      "package://object_server/launch/meshes/mars/mars.dae", 
      0,
      0.0, 0.0, -0.02,
      0 ,0 ,0, 1,
      10));
    terrain_marker_.markers.back().header.frame_id = map_frame_;
    terrain_marker_.markers.back().header.stamp = ros::Time::now();

    return true;
  }

  void MarsEnvironment::update(const ros::Time& time)
  {
    terrain_pub_.publish(terrain_marker_);
  }

  bool MarsEnvironment::sample(size_t num_obstacles, int seed)
  {
    // mapping obstacle id to known radius
    static const std::vector<double> RadiusMap = {0, 0.8, 0.5, 0.4, 0.3, 0.0};

    // c++ random engine
    static std::default_random_engine engine;
    static std::uniform_real_distribution<double> real_distibution(-5, 5);
    static std::uniform_int_distribution<int> int_distribution(1, 4);  

    // override prev stuff
    Base::clear();
    obstacle_radius_.resize(num_obstacles);

    std_msgs::Header header;
    header.frame_id = map_frame_;
    header.stamp = ros::Time::now();

    geometry_msgs::Pose pose;
    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = 0.0;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 1.0;

    visualization_msgs::Marker obstacle;
    int random_obstacle;
    std::string mesh_name, obstacle_name;

    for(size_t i = 0; i < num_obstacles; ++i)
    { 
      // random obstacle and position
      random_obstacle = int_distribution(engine);
      pose.position.x = real_distibution(engine);
      pose.position.y = real_distibution(engine);
      pose.position.z = 0.0;

      // load mesh
      mesh_name = "FallingRock_" + std::to_string(random_obstacle) + ".dae";
      obstacle = create_mesh_marker(
        "package://object_server/launch/meshes/rocks/meshes/" + mesh_name, i);
      obstacle.header = header;

      // add
      obstacle_name = "obstacle_" + std::to_string(i);
      Base::add(obstacle_name, obstacle, pose);
      obstacle_radius_[i] = RadiusMap[random_obstacle];
    }
    return true;
  }

  void MarsEnvironment::feedback(const std::vector<geometry_msgs::Pose>& poses, size_t obj_idx)
  {
    // fill and publish the object msg, containing all positions
    object_msgs::Objects objects_msg;
    objects_msg.header.stamp = ros::Time::now();
    objects_msg.header.frame_id = map_frame_;

    objects_msg.objects.resize(poses.size());
    for(size_t i = 0; i < poses.size(); ++i)
    {
      objects_msg.objects[i].id.data = i;
      objects_msg.objects[i].pose = poses[i];
      objects_msg.objects[i].radius.data = obstacle_radius_[i];
    }
    obstacles_pub_.publish(objects_msg);
  }

  bool MarsEnvironment::addObstaclesHandler(object_msgs::AddObjectsRequest& req, object_msgs::AddObjectsResponse& res)
  {
    // sample new obstacles
    return sample(req.number.data, req.seed.data);
  }
}