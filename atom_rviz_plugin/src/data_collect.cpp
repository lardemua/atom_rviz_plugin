#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/InteractiveMarkerFeedback.h>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::collectDataSaveButtonClicked(){
      visualization_msgs::InteractiveMarkerFeedback marker;

      marker.client_id = "/rviz/ManualDataLabeler-InteractiveMarker";
      marker.marker_name = "menu";
      marker.event_type = 2;
      marker.menu_entry_id = 1;

      data_collect_pub.publish(marker);
    } //  function collectDataSaveButtonClicked()
}  // namespace atom_rviz_plugin
