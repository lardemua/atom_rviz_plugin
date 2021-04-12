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
//      ROS_INFO_STREAM(combobox_sensor);

      // Code to read the service for the combobox_sensor (server)

      ui_->initialEstimateCheckBox->setChecked(false); //change 'false' for the value read from service
      ui_->initialEstimateSpinBox->setValue(10); //change '10' for the value read from service

    } // function initEstimateComboBoxTextChanged()


    void CalibrationPanel::initEstimateCheckboxChanged()
    {
      // Code to call the service (client)
      std::string service_name = "/set_initial_estimate/3dlidar/set_sensor_interactive_marker"; // TODO should be changed according to the sensor that is selected

      //Service clients should not be transient, which means they should be initialized in the class construction and then used here
      // TODO Put this is the constructor
      ros::ServiceClient client = nh.serviceClient<atom_msgs::SetSensorInteractiveMarker>(service_name);

      atom_msgs::SetSensorInteractiveMarker srv;
      //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

      if (ui_->initialEstimateCheckBox->isChecked()) {
        ROS_INFO_STREAM("show marker");
        srv.request.visible = 1;
      } else {
        ROS_INFO_STREAM("hide marker");
        srv.request.visible = 0;
      }

      srv.request.scale = 2; // Change to the scale value that is selected here

      int ret = client.call(srv);
      std::cout << ret << std::endl;
//      if (client.call(srv))
//      {
//        ROS_INFO_STREAM(srv.response.message);
//      }
//      else
//      {
//        ROS_INFO_STREAM(srv.response.message);
//      }

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
