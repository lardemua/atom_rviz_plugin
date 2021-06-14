#include <atom_rviz_plugin/calibration_panel.h>

#include <iostream>
#include "json.hpp"
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <atom_msgs/GetDataset.h>
#include <atom_msgs/SaveCollection.h>
#include <atom_msgs/DeleteCollection.h>
#include <cstdlib>
#include <QMessageBox>

#include "ui_calibration_panel.h"

using json = nlohmann::json;

namespace atom_rviz_plugin
{
    void CalibrationPanel::positionCallback(){
///*  // Function to get the Interactive Marker's position of the sensor in the combobox
      try {
        std::string sensor_in_cb = ui_->collectDataSensorsComboBox->currentText().toUtf8().constData();
      } catch(...) {
        return;
      }
    } //  function positionCallback()

    void CalibrationPanel::collectDataSaveButtonClicked(){
///*  // Save the collection (using the SaveCollection service)
      std::string save_collection_service_name = "/collect_data/save_collection";
      ros::ServiceClient save_collection_client = nh.serviceClient<atom_msgs::SaveCollection>(save_collection_service_name);

      // Call getDataset service
      atom_msgs::SaveCollection save_collection_srv;
      save_collection_client.waitForExistence(); // Wait for the service to be available before calling
      if (save_collection_client.call(save_collection_srv)){
        ROS_INFO("Service %s called successfully.", save_collection_service_name.c_str() );
      }
      else{
        ROS_ERROR("Failed to call service %s", save_collection_service_name.c_str());
      }

      collectDataParseJson();
    } //  function collectDataSaveButtonClicked()


    void CalibrationPanel::collectDataDeleteButtonClicked(){

      QString text_from_label = ui_->collectDataSensorLabel2->text();
      std::string text_from_label_str = text_from_label.toUtf8().constData();

      if (text_from_label_str.compare("(none)") != 0) {
        ui_->collectDataDeleteCollectionLabel->setVisible(false);

        std::string msgDialog_str = "Are you sure you want to delete " + text_from_label_str + "?";
        const char *msgDialog = msgDialog_str.c_str();

        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this,tr("Delete Collection"),tr(msgDialog),QMessageBox::Yes|QMessageBox::No);

        if (reply == QMessageBox::Yes) {
          std::string delete_collection_service_name = "/collect_data/delete_collection";

          ros::ServiceClient delete_collection_client = nh.serviceClient<atom_msgs::DeleteCollection>(delete_collection_service_name);

          atom_msgs::DeleteCollection delete_collection_srv;
          //  Check http://wiki.ros.org/ROS/Tutorials/WritingServiceClient%28c%2B%2B%29

          delete_collection_srv.request.collection_name = text_from_label_str.substr(11);

          delete_collection_client.waitForExistence(); // Wait for the service to be available before calling
          if (delete_collection_client.call(delete_collection_srv)){
            ROS_INFO("Service %s called successfully.", delete_collection_service_name.c_str() );
          }
          else{
            ROS_ERROR("Failed to call service %s", delete_collection_service_name.c_str());
          }

          collectDataParseJson();
        } else {
          return;
        }
      } else {
        ui_->collectDataDeleteCollectionLabel->setVisible(true);
      }


    } //  function collectDataDeleteButtonClicked()


    void CalibrationPanel::collectDataParseJson() {
///*  // Get the collection info (using the GetDataset service)
      std::string get_dataset_service_name = "/collect_data/get_dataset";
      ros::ServiceClient get_dataset_client = nh.serviceClient<atom_msgs::GetDataset>(get_dataset_service_name);

      // Call getDataset service
      atom_msgs::GetDataset get_dataset_srv;
      get_dataset_client.waitForExistence(); // Wait for the service to be available before calling
      if (get_dataset_client.call(get_dataset_srv)){
        ROS_INFO("Service %s called successfully.", get_dataset_service_name.c_str() );
      }
      else{
        ROS_ERROR("Failed to call service %s", get_dataset_service_name.c_str());
      }
//      ROS_INFO("get_dataset_srv.response.dataset_json=%s", get_dataset_srv.response.dataset_json.c_str());

///*  // Putting the collection in a tree widget
//    Parse response to call, in order to get json
      json j = json::parse(get_dataset_srv.response.dataset_json);

      // Get number of collections and change the header accordingly
      QString number_of_collections = QString::fromUtf8((std::to_string((j["collections"]).size()).c_str()));
      ui_->collectDataTreeWidget->setHeaderLabel("Collections: " + number_of_collections);

      // Main item of the tree Widget ("collections")
      QTreeWidgetItem *topTreeItem = ui_->collectDataTreeWidget->topLevelItem(0);

      // Cycle to delete all collections and call function to add them all again
      for( int i = topTreeItem->childCount() - 1; i >= 0; i-- ) {
        QTreeWidgetItem *itemLevel2 = topTreeItem->child(i);
        QString current_item = itemLevel2->text(0);
        std::string current_item_str = current_item.toUtf8().constData();
        topTreeItem->removeChild(itemLevel2);
      }

      std::vector <int> json_key_int;
      for (auto& collection: j["collections"].items()) // iterates over all collections
      {
        json_key_int.push_back(std::stoi(collection.key()));
      }
      std::sort(std::begin(json_key_int), std::end(json_key_int));
      std::string json_key;

      for (size_t i = 0; i < json_key_int.size(); i++) {
        json_key = std::to_string(json_key_int[i]);

        // First level of the tree ("collection 0", "collection 1", ..., "collection X")
        QTreeWidgetItem *childLevel1TreeItem = new QTreeWidgetItem();
        childLevel1TreeItem->setText(0, "collection " + QString::fromUtf8((json_key.c_str())));
        topTreeItem->addChild(childLevel1TreeItem);

        // Second level of the tree ("labels")
        QTreeWidgetItem *childLevel2TreeItem = new QTreeWidgetItem();
        childLevel2TreeItem->setText(0, "labels");
        childLevel1TreeItem->addChild(childLevel2TreeItem);

        for (auto& sensor: j["collections"].at(json_key).at("labels").items()) { // iterates over all sensors in this label
          // Level three of the tree (sensor)
          QTreeWidgetItem *childLevel3TreeItem = new QTreeWidgetItem();
          childLevel3TreeItem->setText(0, QString::fromUtf8((sensor.key().c_str())));
          childLevel2TreeItem->addChild(childLevel3TreeItem);

          // Level four of the tree (label detected)
          QString labels_detected = sensor.value().at("detected") ? "detected: true" : "detected: false";

          QTreeWidgetItem *childLevel4TreeItem = new QTreeWidgetItem();
          childLevel4TreeItem->setText(0, labels_detected);
          childLevel3TreeItem->addChild(childLevel4TreeItem);
        }
      }
      ui_->collectDataSensorLabel2->setText("(none)");



    } //  function collectDataParseJson()

    void CalibrationPanel::collectDataCheckItem(QTreeWidgetItem *item, int column) {
      QString current_item = item->text(column);
      std::string current_item_str = current_item.toUtf8().constData();

      // Checks if string starts with 'collection ' and if it has any digits on it
      if(current_item_str.find("collection ",0) != std::string::npos &&
         std::string::npos != current_item_str.find_first_of("0123456789"))
      {
        ui_->collectDataSensorLabel2->setText(current_item);
      } else {
        ui_->collectDataSensorLabel2->setText("(none)");
      }
    } // function collectDataCheckItem(QTreeWidgetItem *item, int column)

    void CalibrationPanel::collectDataSliderToSpin(int val) {
      ui_->collectDataPoseXDoubleSpinBox->setValue(ui_->collectDataPoseXSlider->value()/100.0);
      ui_->collectDataPoseYDoubleSpinBox->setValue(ui_->collectDataPoseYSlider->value()/100.0);
      ui_->collectDataPoseZDoubleSpinBox->setValue(ui_->collectDataPoseZSlider->value()/100.0);

      dataCollectPubPoseMsg();
    } // function collectDataSliderToSpin(int)

    void CalibrationPanel::collectDataSpinToSlider(double val) {
      ui_->collectDataPoseXSlider->setValue(ui_->collectDataPoseXDoubleSpinBox->value()*100);
      ui_->collectDataPoseYSlider->setValue(ui_->collectDataPoseYDoubleSpinBox->value()*100);
      ui_->collectDataPoseZSlider->setValue(ui_->collectDataPoseZDoubleSpinBox->value()*100);

      dataCollectPubPoseMsg();
    } // function collectDataSpinToSlider(double val)

    void CalibrationPanel::dataCollectPubPoseMsg() {

      visualization_msgs::InteractiveMarkerFeedback marker2;

      marker2.header.frame_id = "3dlidar"; // TODO COLOCAR AQUI O SENSOR LIDO DA COMBOBOX
      marker2.client_id = "/rviz/ManualDataLabeler-InteractiveMarkers";
      marker2.marker_name = "3dlidar";    // TODO COLOCAR AQUI O SENSOR LIDO DA COMBOBOX
      marker2.event_type = 1;
      marker2.control_name = "";
      marker2.pose.position.x = ui_->collectDataPoseXDoubleSpinBox->value();
      marker2.pose.position.y = ui_->collectDataPoseYDoubleSpinBox->value();
      marker2.pose.position.z = ui_->collectDataPoseZDoubleSpinBox->value();
      marker2.pose.orientation.x = 0.0;
      marker2.pose.orientation.y = 0.0;
      marker2.pose.orientation.z = 0.0;
      marker2.pose.orientation.w = 1.0;
      marker2.menu_entry_id = 0;

      data_collect_pub.publish(marker2);
    } //  function initEstimatePubPoseMsg()
}  // namespace atom_rviz_plugin
