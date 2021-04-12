#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::initEstimateComboBoxTextChanged()
    {
      ROS_INFO_STREAM("Read the sensor on combobox;then, read the service to check scale of the marker and if its visible or not");

      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      ROS_INFO_STREAM(combobox_sensor);
      if (combobox_sensor.empty()){
        ui_->initialEstimateCheckBox->setChecked(false);

        // Blocking signals for when value is changed programmatically instead of manually
        bool change_not_manual = ui_->initialEstimateSpinBox->blockSignals(true);
        ui_->initialEstimateSpinBox->setValue(0);
        ui_->initialEstimateSpinBox->blockSignals(change_not_manual);
        return;
      }

      // Code to read the service for the combobox_sensor (server)

      ui_->initialEstimateCheckBox->setChecked(false); //change 'false' for the value read from service
      ui_->initialEstimateSpinBox->setValue(10); //change '10' for the value read from service

    } // function initEstimateComboBoxTextChanged()


    void CalibrationPanel::initEstimateCheckboxChanged()
    {
      // Code to call the service (client)
      ros::ServiceClient client = nh.serviceClient<atom_rviz_plugin::SetSensorInteractiveMarker>("set_interactive_marker");

      if (ui_->initialEstimateCheckBox->isChecked()) {
        ROS_INFO_STREAM("show marker");
      } else {
        ROS_INFO_STREAM("hide marker");
      }

    } //function initEstimateCheckbox()

    void CalibrationPanel::initEstimateSpinBoxInputChanged() {
      // Code to call the service (client)

      ROS_INFO_STREAM("scale changed");
    } // function initEstimateSpinBoxInputChanged()

    void CalibrationPanel::initEstimateSaveButtonClicked() {
      ROS_INFO_STREAM("Save");
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      ROS_INFO_STREAM("Reset");
    } // function initEstimateResetButtonClicked()
}  //namespace atom_rviz_plugin
