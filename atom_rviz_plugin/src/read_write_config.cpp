#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include "ui_calibration_panel.h"

#include <fstream>
#include "yaml-cpp/yaml.h"

#define PFLN ROS_INFO("file %s line %d\n",__FILE__,__LINE__);

namespace atom_rviz_plugin {
    void CalibrationPanel::configComboBoxChange() {

      bool any_text = ui_->paramDescriptionFileTextEdit->toPlainText().isEmpty();
      configLoadParameters(false, !any_text);
    }

    void CalibrationPanel::configLoadParameters(bool clicked /*= true*/, bool comboBoxChanged /*= false*/) {

//      std::string config_file_path = ros::package::getPath("mmtbot_calibration") + "/calibration/config.yml";

      try {
        std::string config_ros_package = ui_->configPackageLineEdit->toPlainText().toUtf8().constData();
        std::string config_file_path = ros::package::getPath(config_ros_package) + "/calibration/config.yml";

        YAML::Node config_params = YAML::LoadFile(config_file_path);
  //      YAML::Node config_params = YAML::LoadFile("/home/miguel/catkin_ws/src/calibration/mmtbot/mmtbot_calibration/calibration/config.yml");
  //      YAML::Node config_params = YAML::LoadFile("/home/mike/catkin_ws/src/calibration/mmtbot/mmtbot_calibration/calibration/config.yml");

        YAML::Node calibration_pattern_node = config_params["calibration_pattern"];
        YAML::Node sensors_node = config_params["sensors"];

        std::vector <std::string> misc_params_content, calib_patt_params_content, sensors_params_content;

        // String parameters for misc and within the calibration_pattern parameter
        std::vector <std::string> str_misc_params = {"description_file",
                                                "bag_file",
                                                "world_link",
                                                "anchored_sensor"};
        std::vector <std::string> cal_pat_str_param = {"link",
                                                       "parent_link",
                                                       "pattern_type",
                                                       "dictionary",
                                                       "mesh_file"};

        // Iterate through all parameters in config.yaml file
        for (YAML::const_iterator it = config_params.begin(); it != config_params.end(); ++it) {

          if (std::find(str_misc_params.begin(), str_misc_params.end(), it->first.as<std::string>()) != str_misc_params.end()) {

            nh.setParam("/" + it->first.as<std::string>(), it->second.as<std::string>());
            misc_params_content.push_back(it->second.as<std::string>());

          } else if (it->first.as<std::string>() == "max_duration_between_msgs") {

            nh.setParam("/" + it->first.as<std::string>(), it->second.as<int>());
            misc_params_content.push_back(std::to_string(it->second.as<int>()));

          } else if (it->first.as<std::string>() == "calibration_pattern") {

            for (YAML::const_iterator it2 = calibration_pattern_node.begin(); it2 != calibration_pattern_node.end(); ++it2) {

              if (std::find(cal_pat_str_param.begin(), cal_pat_str_param.end(), it2->first.as<std::string>()) != cal_pat_str_param.end()) {

                nh.setParam("/" + it->first.as<std::string>() + "/" + it2->first.as<std::string>(), it2->second.as<std::string>());
                calib_patt_params_content.push_back(it2->second.as<std::string>());

              } else if (it2->first.as<std::string>() == "fixed") {

                nh.setParam("/" + it->first.as<std::string>() + "/" + it2->first.as<std::string>(), it2->second.as<bool>());
                calib_patt_params_content.push_back(it2->second.as<bool>() ? "true" : "false");

              } else if (it2->first.as<std::string>() == "size" || it2->first.as<std::string>() == "inner_size") {

                nh.setParam("/" + it->first.as<std::string>() + "/" + it2->first.as<std::string>(), it2->second.as<double>());
                calib_patt_params_content.push_back(std::to_string(it2->second.as<double>()));

              } else if (it2->first.as<std::string>() == "dimension") {
                for (YAML::const_iterator it5 = calibration_pattern_node["dimension"].begin(); it5 != calibration_pattern_node["dimension"].end(); ++it5) {

                  nh.setParam("/" + it->first.as<std::string>() + "/" + it2->first.as<std::string>() + "/" + it5->first.as<std::string>(), it5->second.as<double>());
                  calib_patt_params_content.push_back(std::to_string(it5->second.as<double>()));
                }
              } else if (it2->first.as<std::string>() == "border_size") {
                try {
  //                ROS_INFO_STREAM(it2->first.as<std::string>());
                  ROS_INFO_STREAM(it2->second.as<double>());

                  ui_->configBorderSizeComboBox->addItem("Scalar");
                  ui_->configBorderSizeComboBox->addItem("Dict.");
                  ui_->paramBorderSizeXLabel->setVisible(false);
                  ui_->paramBorderSizeYLabel->setVisible(false);
                  ui_->paramBorderSizeXTextEdit->setVisible(false);
                  ui_->paramBorderSizeYTextEdit->setVisible(false);
                  ui_->paramBorderSizeScalarTextEdit->setVisible(true);
                }
                catch (...) {
                  ui_->configBorderSizeComboBox->addItem("Dict.");
                  ui_->configBorderSizeComboBox->addItem("Scalar");
                  ui_->paramBorderSizeXLabel->setVisible(true);
                  ui_->paramBorderSizeYLabel->setVisible(true);
                  ui_->paramBorderSizeXTextEdit->setVisible(true);
                  ui_->paramBorderSizeYTextEdit->setVisible(true);
                  ui_->paramBorderSizeScalarTextEdit->setVisible(false);

                  for (YAML::const_iterator it6 = calibration_pattern_node["border_size"].begin(); it6 != calibration_pattern_node["border_size"].end(); ++it6) {
  //                  ROS_INFO_STREAM(it6->first.as<std::string>());
  //                  ROS_INFO_STREAM(it6->second.as<std::string>());
                  }
  //                ROS_INFO_STREAM("adeus");
  //                ROS_INFO_STREAM(calibration_pattern_node["border_size"]["x"]);
  //                ROS_INFO_STREAM(calibration_pattern_node["border_size"]["y"]);
                }
              }
            }
          } else if (it->first.as<std::string>() == "sensors") {

            std::string sensor_to_read;
            bool something_in_combobox = false;

            for (YAML::const_iterator it3 = sensors_node.begin(); it3 != sensors_node.end(); ++it3) {

              if (it3 == sensors_node.begin() && ui_->sensorsComboBox->count() == 0)
              {
                something_in_combobox = true;
              }
              if (something_in_combobox){
                QString str_to_combo_box = QString::fromUtf8(it3->first.as<std::string>().c_str());
                ui_->sensorsComboBox->addItem(str_to_combo_box);
              }
              sensor_to_read = ui_->sensorsComboBox->currentText().toUtf8().constData();

              YAML::Node sensor_read_node = sensors_node[it3->first.as<std::string>()];

              for (YAML::const_iterator it4 = sensor_read_node.begin(); it4 != sensor_read_node.end(); ++it4) {

                nh.setParam("/" + it->first.as<std::string>() + "/" + it3->first.as<std::string>() + "/" + it4->first.as<std::string>(), it4->second.as<std::string>());

                if (it3->first.as<std::string>() == sensor_to_read ){
                  sensors_params_content.push_back(it4->second.as<std::string>());
                }

              }
            }
          }
        }

  // Show read parameters on all respective TextEdit boxes
        if (clicked || comboBoxChanged) {
          ui_->paramDescriptionFileTextEdit->setText(QString::fromUtf8(misc_params_content[0].c_str()));
          ui_->paramBagFileTextEdit->setText(QString::fromUtf8(misc_params_content[1].c_str()));
          ui_->paramWorldLinkTextEdit->setText(QString::fromUtf8(misc_params_content[2].c_str()));
          ui_->paramAnchoredSensorTextEdit->setText(QString::fromUtf8(misc_params_content[3].c_str()));
          ui_->paramMaxDurationTextEdit->setText(QString::fromUtf8(misc_params_content[4].c_str()));

          ui_->paramCalibPatLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[0].c_str()));
          ui_->paramCalibPatParentLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[1].c_str()));
          ui_->paramCalibPatFixedTextEdit->setText(QString::fromUtf8(calib_patt_params_content[2].c_str()));
          ui_->paramCalibPatPatternTypeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[3].c_str()));
          ui_->paramCalibPatDictionaryTextEdit->setText(QString::fromUtf8(calib_patt_params_content[4].c_str()));
          ui_->paramCalibPatMeshFileTextEdit->setText(QString::fromUtf8(calib_patt_params_content[5].c_str()));
          ui_->paramCalibPatDimensionXTextEdit->setText(QString::fromUtf8(calib_patt_params_content[6].c_str()));
          ui_->paramCalibPatDimensionYTextEdit->setText(QString::fromUtf8(calib_patt_params_content[7].c_str()));
          ui_->paramCalibPatSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[8].c_str()));
          ui_->paramCalibPatInnerSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[9].c_str()));
          ui_->paramBorderSizeScalarTextEdit->setText("Ola");

          ui_->paramSensorsLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[0].c_str()));
          ui_->paramSensorsParentLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[1].c_str()));
          ui_->paramSensorsChildLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[2].c_str()));
          ui_->paramSensorsTopicNameTextEdit->setText(QString::fromUtf8(sensors_params_content[3].c_str()));
        }


      } catch (...) {
        return;
      }
    } // function configLoadButtonClicked()


// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################
// ####################################################################################################################



  void CalibrationPanel::configWriteButtonClicked() {
  // *************************************
  // * Config Parameters (Miscellaneous) *
  // *************************************

  // String parameters
    std::vector <std::string> misc_params = {"/description_file",
                                             "/bag_file",
                                             "/world_link",
                                             "/anchored_sensor"};
    std::vector <std::string> misc_new_param_str = {
        ui_->paramDescriptionFileTextEdit->toPlainText().toUtf8().constData(),
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
    std::vector <std::string> calib_patt_params = {"/calibration_pattern/link",
                                                   "/calibration_pattern/parent_link",
                                                   "/calibration_pattern/pattern_type",
                                                   "/calibration_pattern/dictionary",
                                                   "/calibration_pattern/mesh_file"};
    std::vector <std::string> calib_patt_new_param_str = {
        ui_->paramCalibPatLinkTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramCalibPatParentLinkTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramCalibPatPatternTypeTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramCalibPatDictionaryTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramCalibPatMeshFileTextEdit->toPlainText().toUtf8().constData()};
    for (size_t i = 0; i < calib_patt_new_param_str.size(); i++) {
      this->nh.setParam(calib_patt_params[i], calib_patt_new_param_str[i]);
    }

    // Int parameter
    std::vector <std::string> calib_patt_int_params = {"/calibration_pattern/size",
                                                       "/calibration_pattern/inner_size"};
    std::vector <std::string> calib_patt_new_int_param_str = {
        ui_->paramCalibPatSizeTextEdit->toPlainText().toUtf8().constData(),
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
/*    std::string calib_patt_new_dict_str = ui_->paramCalibPatDimensionTextEdit->toPlainText().toUtf8().constData();
    int dimension_x_parameter = stoi(calib_patt_new_dict_str.substr(calib_patt_new_dict_str.find("x:") + 2,
                                                                    calib_patt_new_dict_str.find(",") -
                                                                    (calib_patt_new_dict_str.find("x:") + 2)));
    int dimension_y_parameter = stoi(calib_patt_new_dict_str.substr(calib_patt_new_dict_str.find("y:") + 2,
                                                                    calib_patt_new_dict_str.find("}") -
                                                                    (calib_patt_new_dict_str.find("y:") + 2)));
    this->nh.setParam("/calibration_pattern/dimension/x", dimension_x_parameter);
    this->nh.setParam("/calibration_pattern/dimension/y", dimension_y_parameter);
      ROS_INFO_STREAM(calib_patt_new_dict_str);
    ROS_INFO_STREAM(dimension_x_parameter);
    ROS_INFO_STREAM(dimension_y_parameter);*/


  // *********************
  // * Sensors Parameter *
  // *********************
    std::string sensor_to_write = ui_->sensorsComboBox->currentText().toUtf8().constData();

    //String parameters
    std::vector <std::string> sensors_params = {"/sensors/" + sensor_to_write + "/link",
                                                "/sensors/" + sensor_to_write + "/parent_link",
                                                "/sensors/" + sensor_to_write + "/child_link",
                                                "/sensors/" + sensor_to_write + "/topic_name"};
    std::vector <std::string> sensors_new_param_str = {
        ui_->paramSensorsLinkTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramSensorsParentLinkTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramSensorsChildLinkTextEdit->toPlainText().toUtf8().constData(),
        ui_->paramSensorsTopicNameTextEdit->toPlainText().toUtf8().constData()};
    for (size_t i = 0; i < sensors_new_param_str.size(); i++) {
      this->nh.setParam(sensors_params[i], sensors_new_param_str[i]);
    }


    YAML::Node baseNode = YAML::LoadFile("/home/miguel/catkin_ws/src/calibration/mmtbot/mmtbot_calibration/calibration/config.yml");
    baseNode["calibration_pattern"]["link"] = "pattern_link2"; // edit one of the nodes
    std::ofstream fout("/home/miguel/catkin_ws/src/calibration/mmtbot/mmtbot_calibration/calibration/config.yml");
    fout << baseNode; // dump it back into the file

  }  // function writeButtonClicked
}