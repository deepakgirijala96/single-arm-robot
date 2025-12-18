#ifndef _GAZEBO_VISUAL_CONTROLLER_PLUGIN_HH_
#define _GAZEBO_VISUAL_CONTROLLER_PLUGIN_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <memory>
#include <vector>
#include <string>

// ✅ Updated include: Ignition Math color instead of Gazebo Color
#include <ignition/math/Color.hh>

namespace gazebo {
  class VisualControllerPlugin : public WorldPlugin {
    public: 
      void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    private:
      void OnPumpMsg(const std_msgs::Bool::ConstPtr& msg);
      void OnLamLedMsg(const std_msgs::Bool::ConstPtr& msg);
      void OnLcdMsg(const std_msgs::String::ConstPtr& msg);
      void SetVisualMaterial(const std::string& visualName, const std::string& materialName);

      // ✅ Updated Color reference to ignition::math::Color
      void SetVisualsColor(const std::vector<std::string>& visualNames, const ignition::math::Color& color);

      transport::NodePtr gz_node_;
      transport::PublisherPtr vis_pub_;
      std::unique_ptr<ros::NodeHandle> ros_node_;
      ros::Subscriber pump_sub_, lam_led_sub_, lcd_sub_;
      std::string pump_visual_name_, lcd_visual_name_;
      std::vector<std::string> letter_l_visuals_, letter_a_visuals_, letter_m_visuals_;
  };
}

#endif

