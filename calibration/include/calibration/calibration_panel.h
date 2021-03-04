#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/panel.h>

namespace Ui {
class CalibUI;
}

namespace calibration
{
class CalibrationPanel: public rviz::Panel
{
  Q_OBJECT
 public:
  CalibrationPanel(QWidget* parent = nullptr);
  ~CalibrationPanel() override;

  void onInitialize() override;
  void onEnable();
  void onDisable();

private Q_SLOTS:
  void buttonClicked();

protected:
  Ui::CalibUI* ui_;
  int value_{0};
  std::string topic_name_{"calibration"};

  ros::NodeHandle nh_;
  ros::Publisher pub_;
};
} // end namespace calibration
