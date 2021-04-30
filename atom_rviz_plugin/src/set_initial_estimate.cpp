#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/SetSensorInteractiveMarker.h>
#include <atom_msgs/GetSensorInteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include <QHBoxLayout>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QCheckBox>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::setTable(){

      QTableWidgetItem *item0(ui_->tableWidget->item(0,0));
      if (item0 != 0) {
        return;
      }

      std::vector <QString> sensors_for_table = getSensors();

      // TableWidget for sensors in the initial estimate tab
      ui_->tableWidget->verticalHeader()->setVisible(false);
      ui_->tableWidget->setShowGrid(false);
      ui_->tableWidget->setColumnCount(3);
      ui_->tableWidget->setColumnWidth(0, 125);
      ui_->tableWidget->setColumnWidth(1, 85);
      ui_->tableWidget->horizontalHeader()->setSectionResizeMode( 2, QHeaderView::Stretch );

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
        ui_->tableWidget->insertRow( ui_->tableWidget->rowCount() );

        // Table Headers
        ui_->tableWidget->setHorizontalHeaderItem(0,new QTableWidgetItem("Sensors"));
        ui_->tableWidget->setHorizontalHeaderItem(1,new QTableWidgetItem("Show/Hide"));
        ui_->tableWidget->setHorizontalHeaderItem(2,new QTableWidgetItem("Scale"));

        // First column: sensor names
        QTableWidgetItem *sensor_item = new QTableWidgetItem(sensor);
        sensor_item->setFlags(sensor_item->flags() ^ Qt::ItemIsEditable);
        ui_->tableWidget->setItem(ui_->tableWidget->rowCount()-1, 0, sensor_item);

        // Second column of the table (checkboxes for the sensors visibility)
        QWidget *pWidget = new QWidget();
        QCheckBox *pCheckBox = new QCheckBox();
        QHBoxLayout *pLayout = new QHBoxLayout(pWidget);
        pLayout->addWidget(pCheckBox);
        pLayout->setAlignment(Qt::AlignCenter);
        pLayout->setContentsMargins(0,0,0,0);
        pWidget->setLayout(pLayout);
        pCheckBox->setCheckState(get_sensor_srv.response.visible ? Qt::Checked : Qt::Unchecked  );
        ui_->tableWidget->setCellWidget(ui_->tableWidget->rowCount()-1,1,pWidget);
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
        ui_->tableWidget->setCellWidget(ui_->tableWidget->rowCount()-1,2,spinBoxWidget);
        connect(pSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateCheckboxSpinBoxChanged()));
      }
    } //function setTable()


    void CalibrationPanel::initEstimateCheckboxSpinBoxChanged() {

      std::string set_sensor_service_name;

      for (int i = 0; i < ui_->tableWidget->rowCount(); i++) {
        // Get sensor
        QTableWidgetItem *temp = ui_->tableWidget->item(i, 0);
        QString str = temp->text();
        std::string sensor_name = str.toUtf8().constData();

        // Get Checkbox state
        QWidget *pWidget = ui_->tableWidget->cellWidget(i, 1);
        QCheckBox *checkbox = pWidget->findChild<QCheckBox *>();

        // Get SpinBox value
        QWidget *pWidget2 = ui_->tableWidget->cellWidget(i, 2);
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


    void CalibrationPanel::sensorsCellClicked(int row,int col) {
      if (col != 0) { return; }

      // Get sensor
      QTableWidgetItem *temp = ui_->tableWidget->item(row, col);
      QString sensor_str = temp->text();

      ui_->initEstimateSensorLabel2->setText(sensor_str);

    } // function sensorsCellClicked(int row,int col)

    void CalibrationPanel::initEstimateSaveButtonClicked() {
      pubSaveResetMsg(1, "menu");
    } // function initEstimateSaveButtonClicked()

    void CalibrationPanel::initEstimateResetButtonClicked() {
      std::string sensor = ui_->initEstimateSensorLabel2->text().toUtf8().constData();
      pubSaveResetMsg(1, sensor + "_menu");
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
