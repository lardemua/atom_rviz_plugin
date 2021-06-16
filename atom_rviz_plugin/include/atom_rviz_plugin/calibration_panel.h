#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <QTreeWidgetItem>

#include <tf2_ros/transform_listener.h>


namespace Ui {
class CalibUI;
}

namespace atom_rviz_plugin
{
class CalibrationPanel: public rviz::Panel
{
  Q_OBJECT
 public:
  ros::NodeHandle nh;
  ros::Publisher initial_estimate_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/set_initial_estimate/feedback", 1);
  ros::Publisher data_collect_pub = nh.advertise<visualization_msgs::InteractiveMarkerFeedback>("/data_labeler/feedback", 1);
  tf2_ros::Buffer tfBuffer;

   CalibrationPanel(QWidget* parent = nullptr);
  ~CalibrationPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  std::vector <QString> getSensors();
  void handleTabs();
  void getLidarSensorPosition();

  void configSensorsComboBoxChange();
  void configBorderSizeSetComboBox(QString combobox_str);
  void configLoadParameters(bool clicked = true, bool comboBoxChanged = false);
  void configWriteButtonClicked();


  void initEstimateSetTable();
  void initEstimateGetSensorsCurrentPose(std::string sensor);
  void initEstimateCheckboxSpinBoxChanged();
  void initEstimateSensorsCellClicked(int row,int col);
  void initEstimateSaveButtonClicked();
  void initEstimateResetButtonClicked();
  void initEstimateResetAllButtonClicked();
  void initEstimatePubSaveResetMsg(int menu_entry, const std::string& marker_event);
  void initEstimateSliderToSpin(int);
  void initEstimateSpinToSlider(double);
  void initEstimatePubPoseMsg();

  void collectDataSaveButtonClicked();
  void collectDataDeleteButtonClicked();
  void collectDataParseJson();
  void collectDataCheckItem(QTreeWidgetItem *item, int column);
  void collectDataSliderToSpin(int);
  void collectDataSpinToSlider(double);
  void dataCollectPubPoseMsg();

  void calibSetTable();
  void calibHelpButtonClicked();
  void calibCalibrateButtonClicked();
  void calibCopyButtonClicked();
  void calibTableChanged();

protected:
  Ui::CalibUI* ui_;

};


} // end namespace atom_rviz_plugin
