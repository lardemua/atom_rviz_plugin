#include <atom_rviz_plugin/calibration_panel.h>

#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>
#include <atom_msgs/GetSensorInteractiveMarker.h>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::initEstimateComboBoxTextChanged()
    {
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
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
      std::string service_name = "/set_initial_estimate/" + combobox_sensor + "/set_sensor_interactive_marker";

      ros::ServiceClient client2 = nh.serviceClient<atom_msgs::SetSensorInteractiveMarker>(service_name);

      atom_msgs::SetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      srv.request.visible = ui_->initialEstimateCheckBox->isChecked();
      srv.request.scale = ui_->initialEstimateSpinBox->value();

      client2.call(srv);

    } // function initEstimateCheckboxOrSpinBoxInputChanged()


    void CalibrationPanel::initEstimateSaveButtonClicked() {
      ros::Publisher interactive_marker_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/set_initial_estimate/feedback", 1);

      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      visualization_msgs::InteractiveMarkerFeedback save_marker;

      // Set the frame ID and timestamp.  See the TF tutorials for information on these.
      save_marker.header.frame_id = "ee_link";
      save_marker.header.stamp = ros::Time::now();

      save_marker.client_id = "/rviz/MoveSensors-InteractiveMarker";
      save_marker.marker_name = combobox_sensor;
      save_marker.control_name = "";
      save_marker.event_type = 2;

      // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
      save_marker.pose.position.x = -0.02;
      save_marker.pose.position.y = 0.05;
      save_marker.pose.position.z = 0.07;
      save_marker.pose.orientation.x = 0.0;
      save_marker.pose.orientation.y = 0.0;
      save_marker.pose.orientation.z = 0.0;
      save_marker.pose.orientation.w = 1.0;

      save_marker.menu_entry_id = 1;

/*      save_marker.mouse_point.x = 0.0;
      save_marker.mouse_point.y = 0.0;
      save_marker.mouse_point.z = 0.0;*/

      save_marker.mouse_point_valid = 1;

      interactive_marker_pub.publish(save_marker);
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      ROS_INFO_STREAM("Reset");
    } // function initEstimateResetButtonClicked()
}  //namespace atom_rviz_plugin
