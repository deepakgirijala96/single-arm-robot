#include "lam_challenge_sim/VisualControllerPlugin.hh"

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/gazebo.hh>

using namespace gazebo;
GZ_REGISTER_WORLD_PLUGIN(VisualControllerPlugin)

void VisualControllerPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
    if (!ros::isInitialized()) { ROS_FATAL_STREAM("ROS is not initialized."); return; }

    // Read names from SDF
    if (_sdf->HasElement("pump_visual"))
        pump_visual_name_ = _sdf->Get<std::string>("pump_visual");
    if (_sdf->HasElement("lcd_visual"))
        lcd_visual_name_ = _sdf->Get<std::string>("lcd_visual");

    auto load_visuals = [&](const std::string& tag, std::vector<std::string>& vec) {
        if (_sdf->HasElement(tag)) {
            sdf::ElementPtr elem = _sdf->GetElement(tag);
            while (elem) {
                vec.push_back(elem->Get<std::string>());
                elem = elem->GetNextElement(tag);
            }
        }
    };
    load_visuals("letter_l_visual", letter_l_visuals_);
    load_visuals("letter_a_visual", letter_a_visuals_);
    load_visuals("letter_m_visual", letter_m_visuals_);

    gz_node_ = transport::NodePtr(new transport::Node());
    gz_node_->Init();
    // advertise world-scoped visual topic (keeps behavior consistent)
    vis_pub_ = gz_node_->Advertise<msgs::Visual>("~/visual");
    ros_node_.reset(new ros::NodeHandle("visual_controller"));

    // subscribe to the absolute ROS topics (plugin will receive them)
    pump_sub_ = ros_node_->subscribe("/pump_control", 1, &VisualControllerPlugin::OnPumpMsg, this);
    lam_led_sub_ = ros_node_->subscribe("/lam_led_control", 1, &VisualControllerPlugin::OnLamLedMsg, this);
    lcd_sub_ = ros_node_->subscribe("/lcd_display", 1, &VisualControllerPlugin::OnLcdMsg, this);

    ROS_INFO("✅ Visual Controller Plugin Loaded! (Image Display Version)");
}

/*
  NOTE:
  - We set ambient, diffuse, and emissive together for robust visibility
  - msgs::Set accepts ignition::math::Color; this forces Gazebo to update the visual.
*/

void VisualControllerPlugin::OnPumpMsg(const std_msgs::Bool::ConstPtr& msg) {
    ignition::math::Color color = msg->data ?
        ignition::math::Color(0.1, 0.8, 0.8, 1.0) :
        ignition::math::Color(0.0, 0.3, 0.3, 1.0);

    msgs::Visual visual_msg;
    visual_msg.set_name(pump_visual_name_);
    msgs::Material *mat_msg = visual_msg.mutable_material();

    // Keep the material script name so existing scripts still apply
    mat_msg->mutable_script()->set_name("Gazebo/Color");

    // Set all three channels for reliability across Gazebo builds
    msgs::Set(mat_msg->mutable_ambient(), color);
    msgs::Set(mat_msg->mutable_diffuse(), color);
    msgs::Set(mat_msg->mutable_emissive(), color);

    vis_pub_->Publish(visual_msg);
}

void VisualControllerPlugin::OnLamLedMsg(const std_msgs::Bool::ConstPtr& msg) {
    ignition::math::Color color = msg->data ?
        ignition::math::Color(1.0, 0.0, 0.0, 1.0) :
        ignition::math::Color(0.2, 0.2, 0.2, 1.0);

    SetVisualsColor(letter_l_visuals_, color);
    SetVisualsColor(letter_a_visuals_, color);
    SetVisualsColor(letter_m_visuals_, color);
}

void VisualControllerPlugin::OnLcdMsg(const std_msgs::String::ConstPtr& msg) {
    // If the message contains "Success" we try to switch the LCD material to the named material
    if (msg->data.find("Success") != std::string::npos) {
        // Set the material script (if present) and also set colors as a fallback
        SetVisualMaterial(lcd_visual_name_, "LCD/SuccessImage");
    }
}

void VisualControllerPlugin::SetVisualMaterial(const std::string& visualName, const std::string& materialName) {
    msgs::Visual visual_msg;
    visual_msg.set_name(visualName);
    msgs::Material *mat_msg = visual_msg.mutable_material();

    // Primary action: request the material script (user must provide the .material file)
    mat_msg->mutable_script()->set_name(materialName);

    // Fallback: also set neutral bright colors so the visual will update visibly
    ignition::math::Color white = ignition::math::Color(1.0, 1.0, 1.0, 1.0);
    msgs::Set(mat_msg->mutable_ambient(), white);
    msgs::Set(mat_msg->mutable_diffuse(), white);
    msgs::Set(mat_msg->mutable_emissive(), white);

    vis_pub_->Publish(visual_msg);
}

void VisualControllerPlugin::SetVisualsColor(const std::vector<std::string>& visualNames, const ignition::math::Color& color) {
    for (const auto& name : visualNames) {
        msgs::Visual visual_msg;
        visual_msg.set_name(name);
        msgs::Material *mat_msg = visual_msg.mutable_material();

        mat_msg->mutable_script()->set_name("Gazebo/Color");

        // Set ambient, diffuse, emissive for visible change
        msgs::Set(mat_msg->mutable_ambient(), color);
        msgs::Set(mat_msg->mutable_diffuse(), color);
        msgs::Set(mat_msg->mutable_emissive(), color);

        vis_pub_->Publish(visual_msg);
    }
}

