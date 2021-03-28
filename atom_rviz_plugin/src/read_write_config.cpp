#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::readButtonClicked()
    {
// *************************************
// * Config Parameters (Miscellaneous) *
// *************************************

// String parameters
      std::vector<std::string> misc_params = {"/description_file",
                                              "/bag_file",
                                              "/world_link",
                                              "/anchored_sensor"};
      std::vector<std::string> misc_params_content;

      for (size_t i = 0; i < misc_params.size(); i++) {
        std::string param_i = misc_params[i];
        std::string param_content;
        this->nh.getParam(param_i, param_content);
        misc_params_content.push_back(param_content);
/*    ROS_INFO_STREAM("Reading parameter " << param_i);
    ROS_INFO_STREAM(param_content);*/
      }

// Int parameter
      int max_duration_between_msgs;
      this->nh.getParam("/max_duration_between_msgs", max_duration_between_msgs);
      std::string max_duration_between_msgs_str = std::to_string(max_duration_between_msgs);
      misc_params_content.push_back(max_duration_between_msgs_str);



// *********************************
// * Calibration Pattern Parameter *
// *********************************

//String parameters
      std::vector<std::string> calib_patt_params = {"/calibration_pattern/link",
                                                    "/calibration_pattern/parent_link",
                                                    "/calibration_pattern/pattern_link",
                                                    "/calibration_pattern/dictionary",
                                                    "/calibration_pattern/mesh_file"};
      std::vector<std::string> calib_patt_params_content;

      for (size_t i = 0; i < calib_patt_params.size(); i++) {
        std::string param_i = calib_patt_params[i];
        std::string param_content;
        this->nh.getParam(param_i, param_content);
        calib_patt_params_content.push_back(param_content);
      }

// Int parameters
      std::vector<std::string> calib_patt_params_int = {"/calibration_pattern/size",
                                                        "/calibration_pattern/inner_size"};
      for (size_t i = 0; i < calib_patt_params_int.size(); i++) {
        double param_content;
        this->nh.getParam(calib_patt_params_int[i], param_content);
        calib_patt_params_content.push_back(std::to_string(param_content));
      }

// Boolean parameter
      bool calib_fixed;
      this->nh.getParam("/calibration_pattern/fixed", calib_fixed);
      std::string calib_patt_params_bool = calib_fixed ? "true":"false";
      calib_patt_params_content.push_back(calib_patt_params_bool);



// Dictionary Parameters
      std::vector<std::string> calib_patt_dict = {"/calibration_pattern/dimension/x",
                                                  "/calibration_pattern/dimension/y"};
      std::vector<std::string> calib_patt_dimension_content;

      for (size_t i = 0; i < calib_patt_dict.size(); i++) {
        std::string param_i = calib_patt_dict[i];
        int param_content;
        this->nh.getParam(param_i, param_content);
        calib_patt_dimension_content.push_back(std::to_string(param_content));
      }
      calib_patt_params_content.push_back("{x:" + calib_patt_dimension_content[0] + ",   " +
                                          "y:" + calib_patt_dimension_content[1] + "}");

// *********************
// * Sensors Parameter *
// *********************
      std::string sensor_to_read = ui_->sensorsComboBox->currentText().toUtf8().constData();
//  ROS_INFO_STREAM(sensor_to_read);

      //String parameters
      std::vector<std::string> sensors_params = {"/sensors/" + sensor_to_read + "/link",
                                                 "/sensors/" + sensor_to_read + "/parent_link",
                                                 "/sensors/" + sensor_to_read + "/child_link",
                                                 "/sensors/" + sensor_to_read + "/topic_name"};
      std::vector<std::string> sensors_params_content;

      for (size_t i = 0; i < sensors_params.size(); i++) {
        std::string param_i = sensors_params[i];
        std::string param_content;
        this->nh.getParam(param_i, param_content);
        sensors_params_content.push_back(param_content);
/*    ROS_INFO_STREAM("Reading parameter " << param_i);
    ROS_INFO_STREAM(param_content);*/
      }


// Show read parameters on respective TextEdit boxes
      ui_->paramDescriptionFileTextEdit->setText(QString::fromUtf8(misc_params_content[0].c_str()));
      ui_->paramBagFileTextEdit->setText(QString::fromUtf8(misc_params_content[1].c_str()));
      ui_->paramWorldLinkTextEdit->setText(QString::fromUtf8(misc_params_content[2].c_str()));
      ui_->paramAnchoredSensorTextEdit->setText(QString::fromUtf8(misc_params_content[3].c_str()));
      ui_->paramMaxDurationTextEdit->setText(QString::fromUtf8(misc_params_content[4].c_str()));

      ui_->paramCalibPatLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[0].c_str()));
      ui_->paramCalibPatParentLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[1].c_str()));
      ui_->paramCalibPatPatternTypeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[2].c_str()));
      ui_->paramCalibPatDictionaryTextEdit->setText(QString::fromUtf8(calib_patt_params_content[3].c_str()));
      ui_->paramCalibPatMeshFileTextEdit->setText(QString::fromUtf8(calib_patt_params_content[4].c_str()));
      ui_->paramCalibPatSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[5].c_str()));
      ui_->paramCalibPatInnerSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[6].c_str()));
      ui_->paramCalibPatFixedTextEdit->setText(QString::fromUtf8(calib_patt_params_content[7].c_str()));
      ui_->paramCalibPatDimensionTextEdit->setText(QString::fromUtf8(calib_patt_params_content[8].c_str()));

      ui_->paramSensorsLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[0].c_str()));
      ui_->paramSensorsParentLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[1].c_str()));
      ui_->paramSensorsChildLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[2].c_str()));
      ui_->paramSensorsTopicNameTextEdit->setText(QString::fromUtf8(sensors_params_content[3].c_str()));
    } // function readButtonClicked

    void CalibrationPanel::writeButtonClicked() {
// *************************************
// * Config Parameters (Miscellaneous) *
// *************************************

// String parameters
      std::vector<std::string> misc_params = {"/description_file",
                                              "/bag_file",
                                              "/world_link",
                                              "/anchored_sensor"};
      std::vector <std::string> misc_new_param_str = {ui_->paramDescriptionFileTextEdit->toPlainText().toUtf8().constData(),
                                                      ui_->paramBagFileTextEdit->toPlainText().toUtf8().constData(),
                                                      ui_->paramWorldLinkTextEdit->toPlainText().toUtf8().constData(),
                                                      ui_->paramAnchoredSensorTextEdit->toPlainText().toUtf8().constData()};

      for (size_t i = 0; i < misc_new_param_str.size(); i++) {
        this->nh.setParam(misc_params[i], misc_new_param_str[i]);
      }

// Int parameter
      std::string misc_new_param_int_str = ui_->paramMaxDurationTextEdit->toPlainText().toUtf8().constData();
      int misc_new_param_int = std::stoi(misc_new_param_int_str);
      this->nh.setParam("/max_duration_between_msgs", misc_new_param_int);


// *********************************
// * Calibration Pattern Parameter *
// *********************************

//String parameters
      std::vector<std::string> calib_patt_params = {"/calibration_pattern/link",
                                                    "/calibration_pattern/parent_link",
                                                    "/calibration_pattern/pattern_type",
                                                    "/calibration_pattern/dictionary",
                                                    "/calibration_pattern/mesh_file"};
      std::vector<std::string> calib_patt_new_param_str = {ui_->paramCalibPatLinkTextEdit->toPlainText().toUtf8().constData(),
                                                           ui_->paramCalibPatParentLinkTextEdit->toPlainText().toUtf8().constData(),
                                                           ui_->paramCalibPatPatternTypeTextEdit->toPlainText().toUtf8().constData(),
                                                           ui_->paramCalibPatDictionaryTextEdit->toPlainText().toUtf8().constData(),
                                                           ui_->paramCalibPatMeshFileTextEdit->toPlainText().toUtf8().constData()};
      for (size_t i = 0; i < calib_patt_new_param_str.size(); i++) {
        this->nh.setParam(calib_patt_params[i], calib_patt_new_param_str[i]);
      }

      // Int parameter
      std::vector<std::string> calib_patt_int_params = {"/calibration_pattern/size",
                                                        "/calibration_pattern/inner_size"};
      std::vector<std::string> calib_patt_new_int_param_str = {ui_->paramCalibPatSizeTextEdit->toPlainText().toUtf8().constData(),
                                                               ui_->paramCalibPatInnerSizeTextEdit->toPlainText().toUtf8().constData()};
      for (size_t i = 0; i < calib_patt_new_int_param_str.size(); i++) {
        double calib_patt_new_param_int = std::stod(calib_patt_new_int_param_str[i]);
        this->nh.setParam(calib_patt_int_params[i], calib_patt_new_param_int);
      }

      // Boolean parameter
      std::string calib_patt_new_param_bool_str = ui_->paramCalibPatFixedTextEdit->toPlainText().toUtf8().constData();
      bool misc_new_param_bool = calib_patt_new_param_bool_str.compare("true") ? true : false;
      this->nh.setParam("/calibration_pattern/fixed", calib_patt_new_param_bool_str);

      // Dictionary parameters
      // /calibration_pattern/dimension parameter:
      std::string calib_patt_new_dict_str = ui_->paramCalibPatDimensionTextEdit->toPlainText().toUtf8().constData();
      int dimension_x_parameter = stoi(calib_patt_new_dict_str.substr(calib_patt_new_dict_str.find("x:")+2,
                                                                      calib_patt_new_dict_str.find(",") - (calib_patt_new_dict_str.find("x:") +2)));
      int dimension_y_parameter = stoi(calib_patt_new_dict_str.substr(calib_patt_new_dict_str.find("y:")+2,
                                                                      calib_patt_new_dict_str.find("}") - (calib_patt_new_dict_str.find("y:") +2)));
      this->nh.setParam("/calibration_pattern/dimension/x", dimension_x_parameter);
      this->nh.setParam("/calibration_pattern/dimension/y", dimension_y_parameter);
      /*  ROS_INFO_STREAM(calib_patt_new_dict_str);
      ROS_INFO_STREAM(dimension_x_parameter);
      ROS_INFO_STREAM(dimension_y_parameter);*/


// *********************
// * Sensors Parameter *
// *********************
      std::string sensor_to_write = ui_->sensorsComboBox->currentText().toUtf8().constData();

      //String parameters
      std::vector<std::string> sensors_params = {"/sensors/" + sensor_to_write + "/link",
                                                 "/sensors/" + sensor_to_write + "/parent_link",
                                                 "/sensors/" + sensor_to_write + "/child_link",
                                                 "/sensors/" + sensor_to_write + "/topic_name"};
      std::vector<std::string> sensors_new_param_str = {ui_->paramSensorsLinkTextEdit->toPlainText().toUtf8().constData(),
                                                        ui_->paramSensorsParentLinkTextEdit->toPlainText().toUtf8().constData(),
                                                        ui_->paramSensorsChildLinkTextEdit->toPlainText().toUtf8().constData(),
                                                        ui_->paramSensorsTopicNameTextEdit->toPlainText().toUtf8().constData()};
      for (size_t i = 0; i < sensors_new_param_str.size(); i++) {
        this->nh.setParam(sensors_params[i], sensors_new_param_str[i]);
      }

    }  // function writeButtonClicked
}  //namespace atom_rviz_plugin
