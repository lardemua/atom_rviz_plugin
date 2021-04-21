#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>
#include <atom_msgs/GetSensorInteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "ui_calibration_panel.h"

using namespace std;

namespace atom_rviz_plugin
{
    void CalibrationPanel::setTable(std::vector <QString> sensors_for_table){

      // TableWidget for sensors in the initial estimate tab
      ui_->tableWidget->verticalHeader()->setVisible(false);
      ui_->tableWidget->setColumnCount(2);
      ui_->tableWidget->setColumnWidth(0, 175);
      ui_->tableWidget->horizontalHeader()->setSectionResizeMode( 1, QHeaderView::Stretch );

      for (size_t i = 0; i < sensors_for_table.size(); i++) {
        QString sensor = sensors_for_table[i];
        std::string sensor_str = sensor.toUtf8().constData();

        // Get State of sensor marker (visibility and scale)
        std::string service_name = "/set_initial_estimate/" + sensor_str + "/get_sensor_interactive_marker";
        ros::ServiceClient client = nh.serviceClient<atom_msgs::GetSensorInteractiveMarker>(service_name);

        atom_msgs::GetSensorInteractiveMarker srv;
        //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

        client.waitForExistence(); // Wait for the service to be available before calling
        if (client.call(srv)){
         ROS_INFO("Service %s called successfully.", service_name.c_str() );
       }
       else{
        ROS_ERROR("Failed to call service %s", service_name.c_str());
        return; // perhaps here we should do a shutdown or an exit?
       }
//       cout <<  << srv.response.visible << endl;
       ROS_INFO("srv.response.visible=%d", srv.response.visible); // just for testing, return visible=1 so it should be fine

        //Add row
        ui_->tableWidget->insertRow( ui_->tableWidget->rowCount() );

        //Table Headers
        QTableWidgetItem *sensors_header = new QTableWidgetItem();
        sensors_header->setText("Sensors");
        ui_->tableWidget->setHorizontalHeaderItem(0,sensors_header);

        QTableWidgetItem *scale_header = new QTableWidgetItem();
        scale_header->setText("Scale");
        ui_->tableWidget->setHorizontalHeaderItem(1,scale_header);

        // First column of the table (checkboxes for the sensors)
        QTableWidgetItem *sensor_item = new QTableWidgetItem(sensor);
        sensor_item->setCheckState(srv.response.visible ? Qt::Checked : Qt::Unchecked  );
        sensor_item->setFlags(sensor_item->flags() ^ Qt::ItemIsEditable);
        ui_->tableWidget->setItem(ui_->tableWidget->rowCount()-1, 0, sensor_item);

        // Second column of the table (spinbox for the scale of the marker)
        ui_->tableWidget->setCellWidget(ui_->tableWidget->rowCount()-1, 1, new QSpinBox);
        ui_->initialEstimateSpinBox->setValue(srv.response.scale);
      }
    } //function setTable()

    void CalibrationPanel::initEstimateComboBoxTextChanged()
    {
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      if (combobox_sensor.empty()){
        ui_->initEstimateChooseSensorLabel->setVisible(true);
      } else {
        ui_->initEstimateChooseSensorLabel->setVisible(false);
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
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      pubSaveResetMsg(1, "menu");
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      std::string combobox_sensor = ui_->initEstimateSensorsComboBox->currentText().toUtf8().constData();
      pubSaveResetMsg(1, combobox_sensor + "_menu");
    } // function initEstimateResetButtonClicked()


    void CalibrationPanel::initEstimateResetAllButtonClicked() {
      pubSaveResetMsg(2, "menu");
    } // function initEstimateResetAllButtonClicked()


    void CalibrationPanel::pubSaveResetMsg(int menu_entry, const std::string& marker_event) {
      visualization_msgs::InteractiveMarkerFeedback marker;

      marker.client_id = "/rviz/MoveSensors-InteractiveMarker";
      marker.marker_name = marker_event;
      marker.event_type = 2;
      marker.menu_entry_id = menu_entry;

      initial_estimate_pub.publish(marker);
    } //  function pubSaveResetMsg(int menu_entry, const std::string& marker_event)
}  // namespace atom_rviz_plugin
