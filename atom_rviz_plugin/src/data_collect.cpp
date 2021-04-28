#include <atom_rviz_plugin/calibration_panel.h>

#include <iostream>
#include "json.hpp"
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <atom_msgs/GetDataset.h>
#include <atom_msgs/SaveCollection.h>
#include <cstdlib>

using json = nlohmann::json;

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::collectDataSaveButtonClicked(){

///*  // Save the collection (using the SaveCollection service)
      std::string service_name = "/collect_data/save_collection";
      ros::ServiceClient client = nh.serviceClient<atom_msgs::SaveCollection>(service_name);

      // Call getDataset service
      atom_msgs::SaveCollection srv;
      client.waitForExistence(); // Wait for the service to be available before calling
      if (client.call(srv)){
        ROS_INFO("Service %s called successfully.", service_name.c_str() );
      }
      else{
        ROS_ERROR("Failed to call service %s", service_name.c_str());
      }

///*  // Get the collection info (using the GetDataset service)
      std::string service_name2 = "/collect_data/get_dataset";
      ros::ServiceClient client2 = nh.serviceClient<atom_msgs::GetDataset>(service_name2);

      // Call getDataset service
      atom_msgs::GetDataset srv2;
      client2.waitForExistence(); // Wait for the service to be available before calling
      if (client2.call(srv2)){
        ROS_INFO("Service %s called successfully.", service_name2.c_str() );
      }
      else{
        ROS_ERROR("Failed to call service %s", service_name2.c_str());
      }
      ROS_INFO("srv2.response.dataset_json=%s", srv2.response.dataset_json.c_str());

///*  // Putting the collection in a tree widget
//    Parse response to call, in order to get json
      json j = json::parse(srv2.response.dataset_json);

      // Get number of collections and change the header accordingly
      QString number_of_collections = QString::fromUtf8((std::to_string((j["collections"]).size()).c_str()));
      ui_->treeWidget->setHeaderLabel("Collections: " + number_of_collections);

      // Main item of the tree Widget ("collections")
      QTreeWidgetItem *topTreeItem = ui_->treeWidget->topLevelItem(0);

      // First level of the tree ("collection 1", "collection 2", ..., "collection X")
      QString coll_n = QString::fromUtf8((std::to_string((j["collections"]).size() - 1).c_str()));
      QTreeWidgetItem *childLevel1TreeItem = new QTreeWidgetItem();
//      childLevel1TreeItem->setText(0, "collection " + number_of_collections);
      childLevel1TreeItem->setText(0, "collection " + coll_n);
      topTreeItem->addChild(childLevel1TreeItem);

      // Second level of the tree ("labels")
      QTreeWidgetItem *childLevel2TreeItem = new QTreeWidgetItem();
      childLevel2TreeItem->setText(0, "labels");
      childLevel1TreeItem->addChild(childLevel2TreeItem);

      // Third and fourth levels of the tree (sensors and if labels were detected or not)
      std::string last_collection_added = std::to_string((j["collections"]).size() - 1);
      for (auto& collection: j["collections"].items()) // iterates over all collections
      {
        if (collection.key() == last_collection_added)
        {
          for (auto& sensor: collection.value().at("labels").items()) { // iterates over all sensors in this label
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
      }
    } //  function collectDataSaveButtonClicked()


    void CalibrationPanel::collectDataDeleteButtonClicked(){
      QString text_from_label = ui_->collectDataSensorLabel2->text();
      std::string text_from_label_str = text_from_label.toUtf8().constData();

      if (text_from_label_str.compare("(none)") != 0) {
        ui_->collectDataDeleteCollectionLabel->setVisible(false);

        QTreeWidgetItem *topLevel = ui_->treeWidget->topLevelItem(0);

        for( int i = 0; i < topLevel->childCount(); ++i ) {
          QTreeWidgetItem *itemLevel2 = topLevel->child(i);
          QString current_item = itemLevel2->text(0);
          std::string current_item_str = current_item.toUtf8().constData();

          if(text_from_label_str == current_item_str) {
            ROS_INFO_STREAM("Delete " + current_item_str);
            topLevel->removeChild(itemLevel2);

            CheckItem(topLevel, 0);
            ReadjustItemsNames();
          }
        }
      } else {
        ui_->collectDataDeleteCollectionLabel->setVisible(true);
      }
    } //  function collectDataDeleteButtonClicked()


    void CalibrationPanel::CheckItem(QTreeWidgetItem *item, int column) {
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
    } // function CheckItem(QTreeWidgetItem *item, int column)

    void CalibrationPanel::ReadjustItemsNames(){

    } // function ReadjustItemsNames()
}  // namespace atom_rviz_plugin
