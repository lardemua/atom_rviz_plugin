#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <QTreeWidgetItem>

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
  CalibrationPanel(QWidget* parent = nullptr);
  ~CalibrationPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  std::vector <QString> getSensors();
  void handleTabs();
  void setTable();
  void ComboBoxChange();
  void configLoadParameters(bool clicked = true, bool comboBoxChanged = false);
  void configReadButtonClicked(); //TEST
  void configWriteButtonClicked();
  void initEstimateCheckboxSpinBoxChanged();
  void sensorsCellClicked(int row,int col);
  void initEstimateSaveButtonClicked();
  void initEstimateResetButtonClicked();
  void initEstimateResetAllButtonClicked();
  void pubSaveResetMsg(int menu_entry, const std::string& marker_event);
  void collectDataSaveButtonClicked();
  void collectDataDeleteButtonClicked();
  void collectDataParseJson();
  void collectDataCheckItem(QTreeWidgetItem *item, int column);

protected:
  Ui::CalibUI* ui_;

};
} // end namespace atom_rviz_plugin
