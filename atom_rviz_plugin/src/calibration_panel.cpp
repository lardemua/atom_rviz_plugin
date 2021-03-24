#include <atom_rviz_plugin/calibration_panel.h>
#include <pluginlib/class_list_macros.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ui_calibration_panel.h"

#define PFLN ROS_INFO("file %s line %d\n",__FILE__,__LINE__);

namespace atom_rviz_plugin
{
CalibrationPanel::CalibrationPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::CalibUI())
{
  ui_->setupUi(this);
  int argc = 0;
  char** argv;
  ros::init(argc, NULL, "talker");

}

CalibrationPanel::~CalibrationPanel() = default;

void CalibrationPanel::onInitialize()
{
  connect(ui_->readButton, SIGNAL(clicked()), this, SLOT(readButtonClicked()));
  connect(ui_->writeButton, SIGNAL(clicked()), this, SLOT(writeButtonClicked()));

  parentWidget()->setVisible(true);
}


void CalibrationPanel::readButtonClicked()
{
// *************************************
// * Config Parameters (Miscellaneous) *
// *************************************

// String parameters
  std::vector<std::string> misc_params;
  misc_params.push_back("/description_file");
  misc_params.push_back("/bag_file");
  misc_params.push_back("/world_link");
  misc_params.push_back("/anchored_sensor");

  std::vector<std::string> misc_params_content;

  for (size_t i = 0; i < misc_params.size(); i++) {
    std::string param_i = misc_params[i];
    std::string param_content;
    this->nh.getParam(param_i, param_content);
    misc_params_content.push_back(param_content);
/*    ROS_INFO_STREAM("Reading parameter " << param_i);
    ROS_INFO_STREAM(param_content);*/
  }

// Int parameters
  int max_duration_between_msgs;
  this->nh.getParam("/max_duration_between_msgs", max_duration_between_msgs);
  std::string max_duration_between_msgs_str = std::to_string(max_duration_between_msgs);
/*  ROS_INFO_STREAM(max_duration_between_msgs);*/


// *********************************
// * Calibration Pattern Parameter *
// *********************************

//String parameters
  std::vector<std::string> calib_patt_params;
  calib_patt_params.push_back("/calibration_pattern/link");
  calib_patt_params.push_back("/calibration_pattern/parent_link");
  calib_patt_params.push_back("/calibration_pattern/pattern_link");
  calib_patt_params.push_back("/calibration_pattern/dictionary");
  calib_patt_params.push_back("/calibration_pattern/mesh_file");

  std::vector<std::string> calib_patt_params_content;

  for (size_t i = 0; i < calib_patt_params.size(); i++) {
    std::string param_i = calib_patt_params[i];
    std::string param_content;
    this->nh.getParam(param_i, param_content);
    calib_patt_params_content.push_back(param_content);
  }

// Int parameters
  std::vector<std::string> calib_patt_params_int;
  calib_patt_params_int.push_back("/calibration_pattern/size");
  calib_patt_params_int.push_back("/calibration_pattern/inner_size");

  for (size_t i = 0; i < calib_patt_params_int.size(); i++) {
    float param_content;
    this->nh.getParam(calib_patt_params_int[i], param_content);
/*    std::string aux_str = std::to_string(param_content);*/
    calib_patt_params_int[i] = std::to_string(param_content);
  }

// Boolean parameter
  bool calib_fixed;
  this->nh.getParam("/calibration_pattern/fixed", calib_fixed);
  std::string calib_patt_params_bool = calib_fixed ? "true":"false";

// Dictionary Parameters ?????
/*
  XmlRpc::XmlRpcValue my_dict;
  nh.getParam("/calibration_pattern/fixed", my_dict);
  ROS_ASSERT(my_dict.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  for (int32_t i = 0; i < my_dict.size(); ++i)
  {
    ROS_ASSERT(my_dict[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    sum += static_cast<double>(my_list[i]);
  }
*/



// Show read parameters on respective TextEdit boxes
  ui_->paramDescriptionFileTextEdit->setText(QString::fromUtf8(misc_params_content[0].c_str()));
  ui_->paramBagFileTextEdit->setText(QString::fromUtf8(misc_params_content[1].c_str()));
  ui_->paramWorldLinkTextEdit->setText(QString::fromUtf8(misc_params_content[2].c_str()));
  ui_->paramAnchoredSensorTextEdit->setText(QString::fromUtf8(misc_params_content[3].c_str()));
  ui_->paramMaxDurationTextEdit->setText(QString::fromUtf8(max_duration_between_msgs_str.c_str()));

  ui_->paramCalibPatLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[0].c_str()));
  ui_->paramCalibPatParentLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[1].c_str()));
  ui_->paramCalibPatPatternTypeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[2].c_str()));
  ui_->paramCalibPatDictionaryTextEdit->setText(QString::fromUtf8(calib_patt_params_content[3].c_str()));
  ui_->paramCalibPatMeshFileTextEdit->setText(QString::fromUtf8(calib_patt_params_content[4].c_str()));
  ui_->paramCalibPatSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_int[0].c_str()));
  ui_->paramCalibPatInnerSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_int[1].c_str()));
  ui_->paramCalibPatFixedTextEdit->setText(QString::fromUtf8(calib_patt_params_bool.c_str()));

// *********************
// * Sensors Parameter *
// *********************
  std::vector<std::string> parameters;
  nh.getParamNames(parameters);
/*  PFLN*/
  for (size_t i = 0; i < parameters.size(); i++) {
    std::string parameters_i = parameters[i];
    size_t idx = parameters_i.find("/sensors/");
    if (idx!=std::string::npos) {
      size_t idx_2 = parameters_i.find("/topic_name");
      if (idx_2!=std::string::npos) {
        std::string sensor_name = parameters_i.substr(std::string("/sensors/").length(),3);
/*        ROS_INFO_STREAM("Reading parameter " << parameters_i);
        ROS_INFO_STREAM("Sensor " << sensor_name);*/
      }
    }
  }
}

void CalibrationPanel::writeButtonClicked()
    {
      std::vector<std::string> misc_params;
      misc_params.push_back("/description_file");
      misc_params.push_back("/bag_file");
      misc_params.push_back("/world_link");
      misc_params.push_back("/anchored_sensor");

      std::vector<std::string> new_param_str;
      new_param_str.push_back(ui_->paramDescriptionFileTextEdit->toPlainText().toUtf8().constData());
      new_param_str.push_back(ui_->paramBagFileTextEdit->toPlainText().toUtf8().constData());
      new_param_str.push_back(ui_->paramWorldLinkTextEdit->toPlainText().toUtf8().constData());
      new_param_str.push_back(ui_->paramAnchoredSensorTextEdit->toPlainText().toUtf8().constData());


      for (size_t i = 0; i < new_param_str.size(); i++) {
/*        QString q_str = ui_->paramWorldLinkTextEdit->toPlainText();
        std::string str = q_str.toUtf8().constData();*/
        this->nh.setParam(misc_params[i], new_param_str[i]);
      }
    }

}  //namespace atom_rviz_plugin

PLUGINLIB_EXPORT_CLASS(atom_rviz_plugin::CalibrationPanel, rviz::Panel )
