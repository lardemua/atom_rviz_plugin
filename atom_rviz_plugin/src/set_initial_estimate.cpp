#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>
#include <atom_msgs/GetSensorInteractiveMarker.h>

#include <map>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::initEstimateComboBoxTextChanged()
    {
      ROS_INFO_STREAM("Read the sensor on combobox;then, read the service to check scale of the marker and if its visible or not");

      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      std::string service_name = "/set_initial_estimate/" + combobox_sensor + "/get_sensor_interactive_marker";

      ros::ServiceClient client = nh.serviceClient<atom_msgs::GetSensorInteractiveMarker>(service_name);

      atom_msgs::GetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      int ret = client.call(srv);

      ui_->initialEstimateCheckBox->setChecked(srv.response.visible); //change 'false' for the value read from service
      ui_->initialEstimateSpinBox->setValue(srv.response.scale); //change '1' for the value read from service

    } // function initEstimateComboBoxTextChanged()


    void CalibrationPanel::initEstimateCheckboxOrSpinBoxInputChanged() {
      // Code to call the service (client)
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      std::string service_name = "/set_initial_estimate/" + combobox_sensor + "/set_sensor_interactive_marker";

      ros::ServiceClient client2 = nh.serviceClient<atom_msgs::SetSensorInteractiveMarker>(service_name);

      atom_msgs::SetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      srv.request.visible = ui_->initialEstimateCheckBox->isChecked();
      srv.request.scale = ui_->initialEstimateSpinBox->value();

      int ret = client2.call(srv);
      std::cout << ret << std::endl;
    } // function initEstimateCheckboxOrSpinBoxInputChanged()


    void CalibrationPanel::initEstimateSaveButtonClicked() {
      ROS_INFO_STREAM("Save");
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      ROS_INFO_STREAM("Reset");
    } // function initEstimateResetButtonClicked()
}  //namespace atom_rviz_plugin
