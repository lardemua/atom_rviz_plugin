#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>
#include <atom_msgs/GetSensorInteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#include <QHBoxLayout>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QCheckBox>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::initEstimateGetSensorsCurrentPose(std::string sensor) {
      tf2_ros::TransformListener tfListener(tfBuffer);
      geometry_msgs::TransformStamped transformStamped;

      std::string parent_link, child_link;
      std::string parent_param = "/sensors/" + sensor + "/parent_link";
      std::string child_param = "/sensors/" + sensor + "/child_link";

      this->nh.getParam(parent_param,parent_link);
      this->nh.getParam(child_param,child_link);

      transformStamped = tfBuffer.lookupTransform(parent_link, child_link, ros::Time(0));

      double x = transformStamped.transform.translation.x;
      double y = transformStamped.transform.translation.y;
      double z = transformStamped.transform.translation.z;
      double roll = transformStamped.transform.rotation.x;
      double pitch = transformStamped.transform.rotation.y;
      double yaw = transformStamped.transform.rotation.z;

      ui_->initEstimatePoseXSlider->setValue(x*100);
      ui_->initEstimatePoseYSlider->setValue(y*100);
      ui_->initEstimatePoseZSlider->setValue(z*100);
      ui_->initEstimatePoseRollSlider->setValue(roll*100);
      ui_->initEstimatePosePitchSlider->setValue(pitch*100);
      ui_->initEstimatePoseYawSlider->setValue(yaw*100);

      initEstimatePubPoseMsg();
    }

    void CalibrationPanel::initEstimateSetTable(){

      QTableWidgetItem *item0(ui_->initEstimateTableWidget->item(0,0));
      if (item0 != 0) {
        return;
      }

      std::vector <QString> sensors_for_table = getSensors();

      // TableWidget for sensors in the initial estimate tab
      ui_->initEstimateTableWidget->verticalHeader()->setVisible(false);
      ui_->initEstimateTableWidget->setShowGrid(false);
      ui_->initEstimateTableWidget->setColumnCount(3);
      ui_->initEstimateTableWidget->setColumnWidth(0, 125);
      ui_->initEstimateTableWidget->setColumnWidth(1, 85);
      ui_->initEstimateTableWidget->horizontalHeader()->setSectionResizeMode( 2, QHeaderView::Stretch );

      for (size_t i = 0; i < sensors_for_table.size(); i++) {
        QString sensor = sensors_for_table[i];
        std::string sensor_str = sensor.toUtf8().constData();

        // Get State of sensor marker (visibility and scale)
        std::string get_sensor_service_name = "/set_initial_estimate/" + sensor_str + "/get_sensor_interactive_marker";
        ros::ServiceClient get_sensor_client = nh.serviceClient<atom_msgs::GetSensorInteractiveMarker>(get_sensor_service_name);

        atom_msgs::GetSensorInteractiveMarker get_sensor_srv;
        //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

        get_sensor_client.waitForExistence(); // Wait for the service to be available before calling
        if (get_sensor_client.call(get_sensor_srv)){
         ROS_INFO("Service %s called successfully.", get_sensor_service_name.c_str() );
        }
        else{
        ROS_ERROR("Failed to call service %s", get_sensor_service_name.c_str());
        return; // perhaps here we should do a shutdown or an exit?
        }
//       cout <<  << srv.response.visible << endl;
        ROS_INFO("get_sensor_srv.response.visible=%d", get_sensor_srv.response.visible); // just for testing, return visible=1 so it should be fine

        //Add row
        ui_->initEstimateTableWidget->insertRow( ui_->initEstimateTableWidget->rowCount() );

        // Table Headers
        ui_->initEstimateTableWidget->setHorizontalHeaderItem(0,new QTableWidgetItem("Sensors"));
        ui_->initEstimateTableWidget->setHorizontalHeaderItem(1,new QTableWidgetItem("Show/Hide"));
        ui_->initEstimateTableWidget->setHorizontalHeaderItem(2,new QTableWidgetItem("Scale"));

        // First column: sensor names
        QTableWidgetItem *sensor_item = new QTableWidgetItem(sensor);
        sensor_item->setFlags(sensor_item->flags() ^ Qt::ItemIsEditable);
        ui_->initEstimateTableWidget->setItem(ui_->initEstimateTableWidget->rowCount()-1, 0, sensor_item);

        // Second column of the table (checkboxes for the sensors visibility)
        QWidget *pWidget = new QWidget();
        QCheckBox *pCheckBox = new QCheckBox();
        QHBoxLayout *pLayout = new QHBoxLayout(pWidget);
        pLayout->addWidget(pCheckBox);
        pLayout->setAlignment(Qt::AlignCenter);
        pLayout->setContentsMargins(0,0,0,0);
        pWidget->setLayout(pLayout);
        pCheckBox->setCheckState(get_sensor_srv.response.visible ? Qt::Checked : Qt::Unchecked  );
        ui_->initEstimateTableWidget->setCellWidget(ui_->initEstimateTableWidget->rowCount()-1,1,pWidget);
        connect(pCheckBox,SIGNAL(clicked()),this,SLOT(initEstimateCheckboxSpinBoxChanged()));


        // Third column of the table  (spinbox for the scale of the marker)
        QWidget *spinBoxWidget = new QWidget();
        QDoubleSpinBox *pSpinBox = new QDoubleSpinBox();
        QHBoxLayout *spinBoxLayout = new QHBoxLayout(pWidget);
        spinBoxLayout->addWidget(pSpinBox);
        spinBoxLayout->setContentsMargins(0,0,0,0);
        spinBoxWidget->setLayout(spinBoxLayout);
        pSpinBox->setDecimals(2);
        pSpinBox->setSingleStep(0.05);
        pSpinBox->setMinimum(0.05);
        pSpinBox->setValue(get_sensor_srv.response.scale);
        ui_->initEstimateTableWidget->setCellWidget(ui_->initEstimateTableWidget->rowCount()-1,2,spinBoxWidget);
        connect(pSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateCheckboxSpinBoxChanged()));
      }
    } //function initEstimateSetTable()


    void CalibrationPanel::initEstimateCheckboxSpinBoxChanged() {

      std::string set_sensor_service_name;

      for (int i = 0; i < ui_->initEstimateTableWidget->rowCount(); i++) {
        // Get sensor
        QTableWidgetItem *temp = ui_->initEstimateTableWidget->item(i, 0);
        QString str = temp->text();
        std::string sensor_name = str.toUtf8().constData();

        // Get Checkbox state
        QWidget *pWidget = ui_->initEstimateTableWidget->cellWidget(i, 1);
        QCheckBox *checkbox = pWidget->findChild<QCheckBox *>();

        // Get SpinBox value
        QWidget *pWidget2 = ui_->initEstimateTableWidget->cellWidget(i, 2);
        QDoubleSpinBox *spinbox = pWidget2->findChild<QDoubleSpinBox *>();

        set_sensor_service_name = "/set_initial_estimate/" + sensor_name + "/set_sensor_interactive_marker";

        ros::ServiceClient set_sensor_client = nh.serviceClient<atom_msgs::SetSensorInteractiveMarker>(set_sensor_service_name);

        atom_msgs::SetSensorInteractiveMarker set_sensor_srv;
        //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

        set_sensor_srv.request.visible = checkbox->isChecked();
        set_sensor_srv.request.scale = spinbox->value();

        set_sensor_client.waitForExistence(); // Wait for the service to be available before calling
        if (set_sensor_client.call(set_sensor_srv)){
          ROS_INFO("Service %s called successfully.", set_sensor_service_name.c_str() );
        }
        else{
          ROS_ERROR("Failed to call service %s", set_sensor_service_name.c_str());
          return; // perhaps here we should do a shutdown or an exit?
        }
      }
    } // function initEstimateCheckboxSpinBoxChanged()


    void CalibrationPanel::initEstimateSensorsCellClicked(int row,int col) {
      if (col != 0) { return; }

      // Get sensor
      QTableWidgetItem *temp = ui_->initEstimateTableWidget->item(row, col);
      QString sensor_str = temp->text();

      ui_->initEstimateSensorLabel2->setText(sensor_str);

      initEstimateGetSensorsCurrentPose(sensor_str.toUtf8().constData());

    } // function initEstimateSensorsCellClicked(int row,int col)

    void CalibrationPanel::initEstimateSaveButtonClicked() {
      initEstimatePubSaveResetMsg(1, "menu");
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      std::string sensor = ui_->initEstimateSensorLabel2->text().toUtf8().constData();
      initEstimatePubSaveResetMsg(1, sensor + "_menu");
    } // function initEstimateResetButtonClicked()


    void CalibrationPanel::initEstimateResetAllButtonClicked() {
      initEstimatePubSaveResetMsg(2, "menu");
    } // function initEstimateResetAllButtonClicked()


    void CalibrationPanel::initEstimatePubSaveResetMsg(int menu_entry, const std::string& marker_event) {
      visualization_msgs::InteractiveMarkerFeedback marker;

      marker.client_id = "/rviz/MoveSensors-InteractiveMarker";
      marker.marker_name = marker_event;
      marker.event_type = 2;
      marker.menu_entry_id = menu_entry;

      initial_estimate_pub.publish(marker);
    } //  function initEstimatePubSaveResetMsg(int menu_entry, const std::string& marker_event)


    void CalibrationPanel::initEstimateSliderToSpin(int val) {
      ui_->initEstimatePoseXDoubleSpinBox->setValue(ui_->initEstimatePoseXSlider->value()/100.0);
      ui_->initEstimatePoseYDoubleSpinBox->setValue(ui_->initEstimatePoseYSlider->value()/100.0);
      ui_->initEstimatePoseZDoubleSpinBox->setValue(ui_->initEstimatePoseZSlider->value()/100.0);
      ui_->initEstimatePoseRollDoubleSpinBox->setValue(ui_->initEstimatePoseRollSlider->value()/100.0);
      ui_->initEstimatePosePitchDoubleSpinBox->setValue(ui_->initEstimatePosePitchSlider->value()/100.0);
      ui_->initEstimatePoseYawDoubleSpinBox->setValue(ui_->initEstimatePoseYawSlider->value()/100.0);

      initEstimatePubPoseMsg();
    } // function initEstimateSliderToSpin(int)

    void CalibrationPanel::initEstimateSpinToSlider(double val) {
      ui_->initEstimatePoseXSlider->setValue(ui_->initEstimatePoseXDoubleSpinBox->value()*100);
      ui_->initEstimatePoseYSlider->setValue(ui_->initEstimatePoseYDoubleSpinBox->value()*100);
      ui_->initEstimatePoseZSlider->setValue(ui_->initEstimatePoseZDoubleSpinBox->value()*100);
      ui_->initEstimatePoseRollSlider->setValue(ui_->initEstimatePoseRollDoubleSpinBox->value()*100);
      ui_->initEstimatePosePitchSlider->setValue(ui_->initEstimatePosePitchDoubleSpinBox->value()*100);
      ui_->initEstimatePoseYawSlider->setValue(ui_->initEstimatePoseYawDoubleSpinBox->value()*100);

      initEstimatePubPoseMsg();
    } // function initEstimateSpinToSlider(double)


    void CalibrationPanel::initEstimatePubPoseMsg() {

      std::string sensor = ui_->initEstimateSensorLabel2->text().toUtf8().constData();
      if (sensor == "(none)") {
        return;
      }

      visualization_msgs::InteractiveMarkerFeedback marker;

      marker.client_id = "/rviz/MoveSensors-InteractiveMarker";
      marker.marker_name = sensor;
      marker.event_type = 1;
      marker.pose.position.x = ui_->initEstimatePoseXDoubleSpinBox->value();
      marker.pose.position.y = ui_->initEstimatePoseYDoubleSpinBox->value();
      marker.pose.position.z = ui_->initEstimatePoseZDoubleSpinBox->value();
      marker.pose.orientation.x = ui_->initEstimatePoseRollDoubleSpinBox->value();
      marker.pose.orientation.y = ui_->initEstimatePosePitchDoubleSpinBox->value();
      marker.pose.orientation.z = ui_->initEstimatePoseYawDoubleSpinBox->value();
      marker.menu_entry_id = 0;

      initial_estimate_pub.publish(marker);
    } //  function initEstimatePubPoseMsg()
}  // namespace atom_rviz_plugin
