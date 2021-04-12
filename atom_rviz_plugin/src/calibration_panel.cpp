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
      ros::init(argc, NULL, "rviz_panel");
    }

    CalibrationPanel::~CalibrationPanel() = default;

    void CalibrationPanel::onInitialize()
    {
      // Functions to run when rviz opens
      getSensors();

      // Qt events for buttons, checkboxes, combobox,...
      connect(ui_->configReadButton, SIGNAL(clicked()), this, SLOT(configReadButtonClicked()));
      connect(ui_->configWriteButton, SIGNAL(clicked()), this, SLOT(configWriteButtonClicked()));

      connect(ui_->initEstimateSensorsComboBox, SIGNAL(currentTextChanged(QString)), this, SLOT(initEstimateComboBoxTextChanged()));
      connect(ui_->initialEstimateCheckBox, SIGNAL(clicked(bool)), this, SLOT(initEstimateCheckboxChanged()));
      connect(ui_->initialEstimateSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinBoxInputChanged()));
      connect(ui_->initEstimateSaveButton, SIGNAL(clicked()), this, SLOT(initEstimateSaveButtonClicked()));
      connect(ui_->initEstimateResetButton, SIGNAL(clicked()), this, SLOT(initEstimateResetButtonClicked()));

      parentWidget()->setVisible(true);

    } //function onInitialize()

    void CalibrationPanel::getSensors()
    {
      // Get number of sensors to put on ComboBox for the configuration file
      std::vector<std::string> parameters;
      nh.getParamNames(parameters);

      ui_->initEstimateSensorsComboBox->addItem("");
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
//            ROS_INFO_STREAM(sensor_name);
          }
        }
      }
    } //function getSensors()
}  //namespace atom_rviz_plugin