#include <atom_rviz_plugin/calibration_panel.h>

#include <iostream>
#include "json.hpp"
#include <iomanip>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <atom_msgs/GetDataset.h>
#include <cstdlib>

using json = nlohmann::json;

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::collectDataSaveButtonClicked(){
      // Save the collection
      visualization_msgs::InteractiveMarkerFeedback marker;

      marker.client_id = "/rviz/ManualDataLabeler-InteractiveMarker";
      marker.marker_name = "menu";
      marker.event_type = 2;
      marker.menu_entry_id = 1;

      data_collect_pub.publish(marker);


/*      // Put the collection in a tree widget
      std::string service_name = "/collect_data/get_dataset";
      ros::ServiceClient client = nh.serviceClient<atom_msgs::GetDataset>(service_name);

      // Call getDataset service
      atom_msgs::GetDataset srv;
      client.waitForExistence(); // Wait for the service to be available before calling
      if (client.call(srv)){
        ROS_INFO("Service %s called successfully.", service_name.c_str() );
      }
      else{
        ROS_ERROR("Failed to call service %s", service_name.c_str());
      }
      ROS_INFO("srv.response.dataset_json=%s", srv.response.dataset_json.c_str());*/

      // Parse response to call, in order to get json
//      json j = json::parse(srv.response.dataset_json);

//      QString treeHeader = ;
//      ui_->treeWidget->setHeaderLabel("Collections: ");

    } //  function collectDataSaveButtonClicked()

    void CalibrationPanel::collectDataGetCollectionButtonClicked(){
      // Put the collection in a tree widget

      std::string service_name = "/collect_data/get_dataset";
      ros::ServiceClient client = nh.serviceClient<atom_msgs::GetDataset>(service_name);

      // Call getDataset service
      atom_msgs::GetDataset srv;
      client.waitForExistence(); // Wait for the service to be available before calling
      if (client.call(srv)){
        ROS_INFO("Service %s called successfully.", service_name.c_str() );
      }
      else{
        ROS_ERROR("Failed to call service %s", service_name.c_str());
      }
      ROS_INFO("srv.response.dataset_json=%s", srv.response.dataset_json.c_str());

      // Parse response to call, in order to get json
      json j = json::parse(srv.response.dataset_json);

/*      QString treeHeader = ;
      ui_->treeWidget->setHeaderLabel("Collections: ");*/

    } //  function collectDataSaveButtonClicked()
}  // namespace atom_rviz_plugin
