#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ui_calibration_panel.h"

#define PFLN ROS_INFO("file %s line %d\n",__FILE__,__LINE__);

namespace atom_rviz_plugin
{
    CalibrationPanel::CalibrationPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::CalibUI())
    {
      ui_->setupUi(this);
      int argc = 0;
      char** argv;
      ros::init(argc, NULL, "talker");
    }

    CalibrationPanel::~CalibrationPanel() = default;

    void CalibrationPanel::onInitialize()
    {
      // Get number of sensors to put on ComboBox for the configuration file
      std::vector<std::string> parameters;
      nh.getParamNames(parameters);

      for (size_t i = 0; i < parameters.size(); i++) {
        std::string parameters_i = parameters[i];
        size_t idx = parameters_i.find("/sensors/");
        if (idx!=std::string::npos) {
          size_t idx_2 = parameters_i.find("/topic_name");
          if (idx_2!=std::string::npos) {
            int sensor_length = parameters_i.size() - std::string("/sensors/").size() - std::string("/topic_name").size();
            std::string sensor_name = parameters_i.substr(std::string("/sensors/").length(),sensor_length);
            QString str_to_combo_box = QString::fromUtf8(sensor_name.c_str());
            ui_->sensorsComboBox->addItem(str_to_combo_box);
            ui_->initEstimateSensorsComboBox->addItem(str_to_combo_box);
/*        ROS_INFO_STREAM("Reading parameter " << parameters_i);
        ROS_INFO_STREAM("Sensor " << sensor_name);*/
          }
        }
      }

      connect(ui_->configReadButton, SIGNAL(clicked()), this, SLOT(configReadButtonClicked()));
      connect(ui_->configWriteButton, SIGNAL(clicked()), this, SLOT(configWriteButtonClicked()));

      connect(ui_->initialEstimateCheckBox, SIGNAL(clicked(bool)), this, SLOT(initEstimateCheckbox()));

      parentWidget()->setVisible(true);

    }

}  //namespace atom_rviz_plugin