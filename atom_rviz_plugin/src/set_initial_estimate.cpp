#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::initEstimateComboBoxTextChanged()
    {
      ROS_INFO_STREAM("Read the sensor on combobox;then, read the service to check scale of the marker and if its visible or not");

      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();

      // Code to read the service for the combobox_sensor (server)

      ui_->initialEstimateCheckBox->setChecked(true); //change 'false' for the value read from service
      ui_->initialEstimateSpinBox->setValue(1); //change '1' for the value read from service

    } // function initEstimateComboBoxTextChanged()


    void CalibrationPanel::initEstimateCheckboxOrSpinBoxInputChanged() {
      // Code to call the service (client)
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      std::string service_name = "/set_initial_estimate/" + combobox_sensor + "/set_sensor_interactive_marker";

      //Service clients should not be transient, which means they should be initialized in the class construction and then used here
      // TODO Put this is the constructor
      ros::ServiceClient client = nh.serviceClient<atom_msgs::SetSensorInteractiveMarker>(service_name);

      atom_msgs::SetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      srv.request.visible = ui_->initialEstimateCheckBox->isChecked();
      srv.request.scale = ui_->initialEstimateSpinBox->value();;

      int ret = client.call(srv);
      std::cout << ret << std::endl;
    } // function initEstimateCheckboxOrSpinBoxInputChanged()


    void CalibrationPanel::initEstimateSaveButtonClicked() {
      ROS_INFO_STREAM("Save");
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      ROS_INFO_STREAM("Reset");
    } // function initEstimateResetButtonClicked()
}  //namespace atom_rviz_plugin
