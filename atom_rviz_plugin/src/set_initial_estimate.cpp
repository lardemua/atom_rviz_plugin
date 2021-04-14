#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>
#include <atom_msgs/GetSensorInteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::initEstimateComboBoxTextChanged()
    {
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      if (combobox_sensor.empty()){
        ui_->init_estimate_sensor_label->setVisible(true);
      } else {
        ui_->init_estimate_sensor_label->setVisible(false);
      }

      std::string service_name = "/set_initial_estimate/" + combobox_sensor + "/get_sensor_interactive_marker";

      ros::ServiceClient client = nh.serviceClient<atom_msgs::GetSensorInteractiveMarker>(service_name);

      atom_msgs::GetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      client.call(srv);

      ui_->initialEstimateCheckBox->setChecked(srv.response.visible); //change 'false' for the value read from service
      ui_->initialEstimateSpinBox->setValue(srv.response.scale); //change '1' for the value read from service

    } // function initEstimateComboBoxTextChanged()


    void CalibrationPanel::initEstimateCheckboxOrSpinBoxInputChanged() {
      // Code to call the service (client)
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      if (combobox_sensor.empty()){ return;}

      std::string service_name = "/set_initial_estimate/" + combobox_sensor + "/set_sensor_interactive_marker";

      ros::ServiceClient client2 = nh.serviceClient<atom_msgs::SetSensorInteractiveMarker>(service_name);

      atom_msgs::SetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      srv.request.visible = ui_->initialEstimateCheckBox->isChecked();
      srv.request.scale = ui_->initialEstimateSpinBox->value();

      client2.call(srv);

    } // function initEstimateCheckboxOrSpinBoxInputChanged()


    void CalibrationPanel::initEstimateSaveButtonClicked() {
      pubSaveResetMsg(1);
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      pubSaveResetMsg(2);
    } // function initEstimateResetButtonClicked()


    void CalibrationPanel::pubSaveResetMsg(int menu_entry) {
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      if (combobox_sensor.empty()){ return;}

      visualization_msgs::InteractiveMarkerFeedback marker;

      marker.client_id = "/rviz/MoveSensors-InteractiveMarker";
      marker.marker_name = combobox_sensor;
      marker.event_type = 2;
      marker.menu_entry_id = menu_entry;

      initial_estimate_pub.publish(marker);
    } // function pubSaveResetMsg(int menu_entry)
}  //namespace atom_rviz_plugin
