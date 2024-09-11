#ifndef INTERACTIVE_IMAGE_DISPLAY_H
#define INTERACTIVE_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/properties/tf_frame_property.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <rviz/frame_manager.h>
#endif


namespace Ogre
{
class SceneNode;
class Rectangle2D;
} // namespace Ogre

namespace rviz_plugin_interactive_image
{

class RenderPanel : public  rviz::RenderPanel
{
Q_OBJECT
public:
  virtual void mousePressEvent(QMouseEvent* event);
  virtual void mouseReleaseEvent(QMouseEvent* event);
  virtual void mouseDoubleClickEvent(QMouseEvent* event);
  virtual void mouseMoveEvent(QMouseEvent* event);
  Q_SIGNALS:
  void mousePosition( int x, int y, Qt::MouseButtons buttons, int type );

};

/**
 * \class InteractiveImageDisplay
 *
 */
class InteractiveImageDisplay : public rviz::ImageDisplayBase
{
  Q_OBJECT
public:
  InteractiveImageDisplay();
  ~InteractiveImageDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  virtual void updateNormalizeOptions();
  void gotInteraction( int x, int y, Qt::MouseButtons buttons, int type);
  virtual void updateSendTopic();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);
  Ogre::SceneManager* img_scene_manager_;

  rviz::ROSImageTexture texture_;

  //rviz::RenderPanel* render_panel_;
  RenderPanel* render_panel_;

  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  rviz::BoolProperty* normalize_property_;
  rviz::FloatProperty* min_property_;
  rviz::FloatProperty* max_property_;
  rviz::IntProperty* median_buffer_size_property_;

  rviz::RosTopicProperty* publish_topic_property_;
  rviz::RosTopicProperty* camera_info_property_;
  rviz::StringProperty* parent_frame_property_;
  rviz::BoolProperty* react_click_property_;
  rviz::BoolProperty* react_release_property_;
  rviz::BoolProperty* react_dblclk_property_;
  rviz::BoolProperty* react_move_property_;
  rviz::BoolProperty* use_bbx_property_;
  rviz::FloatProperty* x_property_;
  rviz::FloatProperty* y_property_;
  rviz::FloatProperty* z_property_;
  rviz::FloatProperty* roll_property_;
  rviz::FloatProperty* pitch_property_;
  rviz::FloatProperty* yaw_property_;
  rviz::FloatProperty* scale_property_;
  float scale_{0};
  float x_move_{0};
  float y_move_{0};
  float z_move_{0};
  float roll_rotate_{0};
  float pitch_to_rotate_{0};
  float yaw_to_rotate_{0};
  bool got_float_image_;

  QString parent_frame_;
  QString output_topic_;
  QString input_topic_;
  ros::Publisher output_publisher_;
  ros::Subscriber cam_info_sub_;
//   image_transport::Subscriber depth_image_sub;
  double cx, cy, fx, fy;
  bool cam_info_received = false;

  cv_bridge::CvImagePtr cv_ptr;
  // The ROS node handle.
  ros::NodeHandle nh_;

};

} // namespace rviz_plugin_interactive_image

#endif
