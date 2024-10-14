#include <boost/bind/bind.hpp>

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>
#include <OgreTechnique.h>
#include <OgreCamera.h>
#include <OgreHardwarePixelBuffer.h>
#include <QMouseEvent>

#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>

#include <sensor_msgs/image_encodings.h>


#include "interactive_image_display.h"

namespace rviz_plugin_interactive_image
{
InteractiveImageDisplay::InteractiveImageDisplay() : rviz::ImageDisplayBase(), texture_()
{
    publish_topic_property_ = new rviz::RosTopicProperty("Point Topic", "",
                           QString::fromStdString(ros::message_traits::datatype<geometry_msgs::Point>()),
                           "geometry_msgs::Point topic to publish.", this, SLOT(updateSendTopic()));
    
    camera_info_property_ = new rviz::RosTopicProperty("Camera_Info Topic", "",
                           QString::fromStdString(ros::message_traits::datatype<sensor_msgs::CameraInfo>()),
                           "geometry_msgs::Point topic to publish.", this, SLOT(updateSendTopic()));
    

    // Reupdate if tf is updated
    parent_frame_property_ = new rviz::StringProperty(
        "Parent tf",       // Name of the property
        "realsense_phantom_link",        // Default value
        "Description of what parents tf is used.",  // Tooltip or description
        this,                  // Parent property, typically 'this' for the top-level property
        SLOT(updateSendTopic()));  // Slot to handle updates

    scale_property_ = new rviz::FloatProperty(
        "Scale unit", 0.001, 
        "Sacle unit.", this, SLOT(updateSendTopic()));

    x_property_ = new rviz::FloatProperty(
        "X (m)", 0.0, 
        "X axis in meters.", this, SLOT(updateSendTopic()));

    y_property_ = new rviz::FloatProperty(
        "Y (m)", 0.0, 
        "Y axis in meters.", this, SLOT(updateSendTopic()));

    z_property_ = new rviz::FloatProperty(
        "Z (m)", 0.0,
        "Z axis in meters.", this, SLOT(updateSendTopic()));

    roll_property_ = new rviz::FloatProperty(
        "Roll (deg)", 0.0,
        "Angle (in degrees) to which to rotate the image.", this, SLOT(updateSendTopic()));
    
    roll_property_->setMin(-360.0);
    roll_property_->setMax(360.0);

    pitch_property_ = new rviz::FloatProperty(
        "Pitch (deg)", 0.0,
        "Angle (in degrees) to which to rotate the image.", this, SLOT(updateSendTopic()));

    pitch_property_->setMin(-360.0);
    pitch_property_->setMax(360.0);

    yaw_property_ = new rviz::FloatProperty(
        "Yaw (deg)", 0.0,
        "Angle (in degrees) to which to rotate the image.", this, SLOT(updateSendTopic()));

    yaw_property_->setMin(-360.0);
    yaw_property_->setMax(360.0);

  normalize_property_ = new rviz::BoolProperty(
      "Normalize Range", false,
      "If set to true, will try to estimate the range of possible values from the received images.",
      this, SLOT(updateNormalizeOptions()));

  min_property_ = new rviz::FloatProperty("Min Value", 0.0, "Value which will be displayed as black.", this,
                                    SLOT(updateNormalizeOptions()));

  max_property_ = new rviz::FloatProperty("Max Value", 1.0, "Value which will be displayed as white.", this,
                                    SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ =
      new rviz::IntProperty("Median window", 5, "Window size for median filter used for computin min/max.",
                      this, SLOT(updateNormalizeOptions()));

  got_float_image_ = false;

  react_click_property_ = new rviz::BoolProperty(
      "React to Mouse Clicks", true,
      "If set to true, will emit mouse click position on single click.", this);
  react_release_property_ = new rviz::BoolProperty(
      "React to Mouse Release", false,
      "If set to true, will emit mouse click position when the mouse button is released.", this);
  react_dblclk_property_ = new rviz::BoolProperty(
      "React to Mouse DoubleClick", false,
      "If set to true, will emit mouse click position on doubleclick event.", this);
  react_move_property_ = new rviz::BoolProperty(
      "React to Mouse Move", false,
      "If set to true, will emit mouse positions when the mouse is moved over the widget.", this);


}

void InteractiveImageDisplay::onInitialize()
{
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "InteractiveImageDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ = img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "InteractiveImageDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu = material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);
    tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    rviz::setMaterial(*screen_rect_, material_);
    img_scene_node_->attachObject(screen_rect_);
  }

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget(render_panel_);

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  updateNormalizeOptions();
  connect (render_panel_, SIGNAL( mousePosition( int, int, Qt::MouseButtons, int )), this, SLOT( gotInteraction( int, int, Qt::MouseButtons, int)));

}

InteractiveImageDisplay::~InteractiveImageDisplay()
{
  if (initialized())
  {
    delete render_panel_;
    delete screen_rect_;
    rviz::removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(), img_scene_node_);
  }
}

void InteractiveImageDisplay::onEnable()
{
  ImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void InteractiveImageDisplay::onDisable()
{
  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  reset();
}

void InteractiveImageDisplay::updateNormalizeOptions()
{
  if (got_float_image_)
  {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(), max_property_->getFloat());
    texture_.setMedianFrames(median_buffer_size_property_->getInt());
  }
  else
  {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void InteractiveImageDisplay::update(float wall_dt, float ros_dt)
{
  Q_UNUSED(wall_dt)
  Q_UNUSED(ros_dt)
  try
  {
    texture_.update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0)
    {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect)
      {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                                 -1.0f * win_aspect / img_aspect, false);
      }
      else
      {
        screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f, 1.0f * img_aspect / win_aspect,
                                 -1.0f, false);
      }
    }

    render_panel_->getRenderWindow()->update();
  }
  catch (rviz::UnsupportedImageEncoding& e)
  {
    setStatus(rviz::StatusProperty::Error, "Image", e.what());
  }
}

void InteractiveImageDisplay::reset()
{
  ImageDisplayBase::reset();
  texture_.clear();
  render_panel_->getCamera()->setPosition(Ogre::Vector3(999999, 999999, 999999));
}

/* This is called by incomingMessage(). */
void InteractiveImageDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg)
{
  bool got_float_image = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                         msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
                         msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_)
  {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }


    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
    }

    if (x_move_ || y_move_ || z_move_) {

        try {
            static tf2_ros::TransformBroadcaster br;
            geometry_msgs::TransformStamped transformStamped;
            
            transformStamped.header.stamp = ros::Time::now();
            // transformStamped.header.frame_id = "oak_rgb_camera_optical_frame";
            transformStamped.header.frame_id = parent_frame_.toStdString();
            transformStamped.child_frame_id = "rviz_frame";
            transformStamped.transform.translation.x = x_move_;
            transformStamped.transform.translation.y = y_move_;
            transformStamped.transform.translation.z = z_move_;
            tf2::Quaternion q;
            q.setRPY(roll_rotate_*3.14159/180, pitch_to_rotate_*3.14159/180, yaw_to_rotate_*3.14159/180);
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            br.sendTransform(transformStamped);

            ros::Duration(0.03).sleep();

        } catch (const std::exception& e) {
            ROS_ERROR("Error broadcasting transform: %s", e.what());
        }

    }

  texture_.addMessage(msg);
}

void InteractiveImageDisplay::gotInteraction( int x, int y, Qt::MouseButtons buttons, int type)
{
    if (ros::ok() && output_publisher_ ) {
        if (type==1 && ! react_click_property_->getBool()) return;
        if (type==2 && ! react_release_property_->getBool()) return;
        if (type==3 && ! react_dblclk_property_->getBool()) return;
        if (type==0 && ! react_move_property_->getBool()) return;

        uint64_t encoded_buttons=(((uint64_t)(buttons))<<2) + (type & 3);

        // make sure the aspect ratio of the image is preserved
        float win_width = render_panel_->width();
        float win_height = render_panel_->height();

        float img_width = texture_.getWidth();
        float img_height = texture_.getHeight();

        if (img_width != 0 && img_height != 0 && win_width != 0 && win_height != 0 && cam_info_received) {
            float img_aspect = img_width / img_height;
            float win_aspect = win_width / win_height;
            float newx = ((float)x)/win_width;
            float newy = ((float)y)/win_height;

            if (img_aspect>win_aspect) {
                newy *= img_aspect/win_aspect;
                newy += 0.5*(1.0-img_aspect/win_aspect);
            } else {
                newx *= win_aspect/img_aspect;
                newx += 0.5*(1.0-win_aspect/img_aspect);
            }

            // Normalize coordinates (assuming they are provided as member variables or come from somewhere)
            int pixel_x = static_cast<int>(newx * cv_ptr->image.cols); // normalized_x should be between 0 and 1
            int pixel_y = static_cast<int>(newy * cv_ptr->image.rows); // normalized_y should be between 0 and 1

            // Ensure we are within the bounds of the image
            pixel_x = std::max(0, std::min(pixel_x, cv_ptr->image.cols - 1));
            pixel_y = std::max(0, std::min(pixel_y, cv_ptr->image.rows - 1));

            // float z_m = cv_ptr->image.at<float>(pixel_y, pixel_x) / 1000.0f;
            float z_m = cv_ptr->image.at<float>(pixel_y, pixel_x) * scale_;
            float x_m = (pixel_x - cx) * z_m / fx; // x in meters
            float y_m = (pixel_y - cy) * z_m / fy; // y in meters

            // we got a click, publish message
            geometry_msgs::Point msg;
            // msg.x=newx;
            // msg.y=newy;
            // msg.z=(double)(encoded_buttons);
            msg.x=(double)x_m;
            msg.y=(double)y_m;
            msg.z=(double)(z_m);
            output_publisher_.publish(msg);

            x_move_ = msg.x;
            y_move_ = msg.y;
            z_move_ = msg.z;
            x_property_->setValue(x_m);
            y_property_->setValue(y_m);
            z_property_->setValue(z_m);
        }
    }
}

void InteractiveImageDisplay::updateSendTopic()
{
    QString new_topic(publish_topic_property_->getString());
    // Only take action if the name has changed.
    if( new_topic != output_topic_ ) { 
        output_topic_ = new_topic;
        // If the topic is the empty string, don't publish anything.
        if( output_topic_ == "" ) {
            output_publisher_.shutdown();
        } else {
            // The old ``velocity_publisher_`` is destroyed by this assignment,
            // and thus the old topic advertisement is removed.  The call to
            // nh_advertise() says we want to publish data on the new topic name.
            output_publisher_ = nh_.advertise<geometry_msgs::Point>(
                  output_topic_.toStdString(), 1 );
        }
    }

    QString new_topic2(camera_info_property_->getString());
    if( new_topic2 != input_topic_ ) { 
        input_topic_ = new_topic2;
        if( input_topic_ == "" ) {
            cam_info_sub_.shutdown();
        } else {
            cam_info_sub_ = nh_.subscribe(input_topic_.toStdString(), 1, &InteractiveImageDisplay::cameraInfoCallback, this);
        }
    }

    // cam_info_sub_ = nh_.subscribe("/camera/stereo/camera_info", 1, &InteractiveImageDisplay::cameraInfoCallback, this);
    
    x_move_           = x_property_->getFloat();
    y_move_           = y_property_->getFloat();
    z_move_           = z_property_->getFloat();
    roll_rotate_      = roll_property_->getFloat();
    pitch_to_rotate_  = pitch_property_->getFloat();
    yaw_to_rotate_    = yaw_property_->getFloat();
    scale_            = scale_property_->getFloat();

    parent_frame_ = parent_frame_property_->getString();

}

void InteractiveImageDisplay::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    cx = cam_info->K[2]; // u-coordinate of the optical center
    cy = cam_info->K[5]; // v-coordinate of the optical center
    fx = cam_info->K[0]; // Focal length in x-axis
    fy = cam_info->K[4]; // Focal length in y-axis
    cam_info_received = true;
}

void RenderPanel::mousePressEvent( QMouseEvent* event )
{
  Q_EMIT mousePosition( event->x(), event->y(), event->buttons(), 1 );
  //onRenderWindowMouseEvents(event); // avoid buggy behaviour in original image display panel
}
void RenderPanel::mouseReleaseEvent( QMouseEvent* event )
{
  Q_EMIT mousePosition( event->x(), event->y(), event->buttons(), 2 );
  //onRenderWindowMouseEvents(event); // avoid buggy behaviour in original image display panel
}
void RenderPanel::mouseDoubleClickEvent( QMouseEvent* event )
{
  Q_EMIT mousePosition( event->x(), event->y(), event->buttons(), 3 );
  //onRenderWindowMouseEvents(event); // avoid buggy behaviour in original image display panel
}
void RenderPanel::mouseMoveEvent( QMouseEvent* event )
{
  Q_EMIT mousePosition( event->x(), event->y(), event->buttons(), 0 );
  //onRenderWindowMouseEvents(event); // avoid buggy behaviour in original image display panel
}

} // namespace rviz_plugin_interactive_image

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_interactive_image::InteractiveImageDisplay, rviz::Display)

