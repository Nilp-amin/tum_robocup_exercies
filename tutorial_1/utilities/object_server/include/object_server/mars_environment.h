#ifndef OBJECT_SERVER_MARS_ENVIRONMENT_H_
#define OBJECT_SERVER_MARS_ENVIRONMENT_H_

// base class
#include <object_server/object_server.h>

// ros messages
#include <object_msgs/Objects.h>
#include <visualization_msgs/MarkerArray.h>

// ros services
#include <object_msgs/AddObjects.h>

namespace object_server
{

  class MarsEnvironment : public ObjectServer
  {
  public:
    typedef ObjectServer Base;

  private:
    std::string map_frame_;
    std::vector<double> obstacle_radius_;

    ros::Publisher obstacles_pub_;
    ros::Publisher terrain_pub_;

    ros::ServiceServer add_objects_srv_;

    visualization_msgs::MarkerArray terrain_marker_;

  public:
    MarsEnvironment();

    ~MarsEnvironment();

    virtual bool initialize(ros::NodeHandle &nh);

    void update(const ros::Time& time);

    bool sample(size_t num_obstacles, int seed=-1);

  protected:
    virtual void feedback(const std::vector<geometry_msgs::Pose>& poses, size_t obj_idx);

  private:
    bool addObstaclesHandler(object_msgs::AddObjectsRequest& req, object_msgs::AddObjectsResponse& res);

  };

}

#endif