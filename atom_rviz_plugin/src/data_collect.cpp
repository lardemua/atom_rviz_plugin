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

//    Parse response to call, in order to get json
      json j = json::parse(srv2.response.dataset_json);

      // print entire json content
      std::cout << std::setw(4) << j << std::endl;

      // Print some fields for testing
      std::cout << std::endl << "TESTING PARSER" << std::endl;

      std::string bagfile = j["calibration_config"]["bag_file"];
      std::cout << "bagfile is " << bagfile << std::endl;

      std::cout << "Number of collections = " << (j["collections"]).size() << std::endl;

      // Check on which sensors the pattern was detected, for each collection
      for (auto& collection: j["collections"].items()) // iterates over all collections
      {
//        std::cout << collection.key() << std::endl; // this prints the key
//        std::cout << collection.value() << std::endl; // this prints the value
        for (auto& sensor: collection.value().at("labels").items()){ // iterates over all sensors in this label
          std::cout << "Collection " << collection.key() <<", sensor " << sensor.key() << " detected=" << sensor.value().at("detected") << std::endl;
        }
      }

///*  // Putting the collection in a tree widget
      QString number_of_collections = QString::fromUtf8((std::to_string((j["collections"]).size()).c_str()));
      ui_->treeWidget->setHeaderLabel("Collections: " + number_of_collections);

      // Main item of the tree Widget
      QTreeWidgetItem *topTreeItem = ui_->treeWidget->topLevelItem(0);

      // First level of the tree
      QTreeWidgetItem *childLevel1TreeItem = new QTreeWidgetItem();
      childLevel1TreeItem->setText(0, "collection " + number_of_collections);
      topTreeItem->addChild(childLevel1TreeItem);


      // Second level of the tree
      for (auto& collection: j["collections"].items()) // iterates over all collections
      {
        for (auto& sensor: collection.value().at("labels").items()) { // iterates over all sensors in this label
          QTreeWidgetItem *childLevel2TreeItem = new QTreeWidgetItem();
          childLevel2TreeItem->setText(0, "sensor ");
          childLevel1TreeItem->addChild(childLevel2TreeItem);
        }
      }



    } //  function collectDataSaveButtonClicked()


    void CalibrationPanel::collectDataDeleteButtonClicked(){

    } //  function collectDataDeleteButtonClicked()



    void CalibrationPanel::CheckItem(QTreeWidgetItem *item, int column) {
      ROS_INFO_STREAM("Ola");
    } // function CheckItem(QTreeWidgetItem *item, int column)
}  // namespace atom_rviz_plugin
