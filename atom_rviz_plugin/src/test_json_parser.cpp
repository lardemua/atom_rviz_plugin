#include <iostream>
#include "json.hpp"
#include <iomanip>
#include "ros/ros.h"
#include <atom_msgs/GetDataset.h>
#include <cstdlib>

using json = nlohmann::json;
using namespace std;

int main(int argc, char **argv)
{
    // Setup ros stuff
    ros::init(argc, argv, "test_json_parser");
    ros::NodeHandle n;
    std::string service_name = "/collect_data/get_dataset";
    ros::ServiceClient client = n.serviceClient<atom_msgs::GetDataset>(service_name);

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
//
//    // print entire json content
//    cout << setw(4) << j << endl;
//
//    // Print some fields for testing
//    cout << endl << "TESTING PARSER" << endl;
//
//    string bagfile = j["calibration_config"]["bag_file"];
//    cout << "bagfile is " << bagfile << endl;
//
//    cout << "Number of collections = " << (j["collections"]).size() << endl;
//
//    // Check on which sensors the pattern was detected, for each collection
//    for (auto& collection: j["collections"].items()) // iterates over all collections
//    {
////      cout << collection.key() << endl; // this prints the key
////      cout << collection.value() << endl; // this prints the value
//      for (auto& sensor: collection.value().at("labels").items()){ // iterates over all sensors in this label
//        cout << "Collection " << collection.key() <<", sensor " << sensor.key() << " detected=" << sensor.value().at("detected") << endl;
//      }
//    }

}