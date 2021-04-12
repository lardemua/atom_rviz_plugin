#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

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
  CalibrationPanel(QWidget* parent = nullptr);
  ~CalibrationPanel() override;

  void onInitialize() override;

private Q_SLOTS:
  void getSensors();
  void configReadButtonClicked();
  void configWriteButtonClicked();
  void initEstimateCheckboxChanged();
  void initEstimateComboBoxTextChanged();
  void initEstimateSpinBoxInputChanged();
  void initEstimateSaveButtonClicked();
  void initEstimateResetButtonClicked();


protected:
  Ui::CalibUI* ui_;

};
} // end namespace atom_rviz_plugin
