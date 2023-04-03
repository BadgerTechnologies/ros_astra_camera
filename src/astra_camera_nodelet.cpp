#include <signal.h>

#include <memory>

#include <nodelet/nodelet.h>

#include "astra_camera/ob_camera_node_factory.h"

void term_handler(int) {
  ros::requestShutdown();
}

namespace astra_camera
{

class AstraDriverNodelet : public nodelet::Nodelet
{
public:
  AstraDriverNodelet()  {};

  ~AstraDriverNodelet() {}

private:
  virtual void onInit()
  {
    /*
     * handle SIGTERM and SIGINT so that we can shutdown cleanly. We have seen
     * cameras get into bad states when this is not shutdown cleanly.
     */
    signal(SIGTERM, term_handler);
    signal(SIGINT, term_handler);
    node_factory_ptr_ = std::make_shared<OBCameraNodeFactory>(getNodeHandle(), getPrivateNodeHandle());
  };

  std::shared_ptr<OBCameraNodeFactory> node_factory_ptr_;
};

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(astra_camera::AstraDriverNodelet, nodelet::Nodelet)
