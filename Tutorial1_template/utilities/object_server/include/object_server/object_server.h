#ifndef OBJECT_SERVER_H_
#define OBJECT_SERVER_H_

#include <interactive_markers/interactive_marker_server.h>

#include <object_server/utilities.h>

namespace object_server
{

  class ObjectServer
  {
  protected:
    size_t object_count_;
    interactive_markers::InteractiveMarkerServer server_;
    std::vector<geometry_msgs::Pose> poses_;

  public:
    ObjectServer();

    virtual ~ObjectServer();

    virtual bool initialize(ros::NodeHandle &nh);

    bool add(
      const std::string& name,
      const visualization_msgs::Marker& marker,
      const geometry_msgs::Pose& pose);

    bool clear();

  protected:
    virtual void feedback(const std::vector<geometry_msgs::Pose>& poses, size_t obj_idx) = 0;

  private:
    void interactiveMarkerFeedback(
      const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback,
      size_t obj_idx);
  };

}

#endif