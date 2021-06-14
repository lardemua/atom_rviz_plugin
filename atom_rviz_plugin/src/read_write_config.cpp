#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"

#include "ui_calibration_panel.h"

#include <fstream>
#include "yaml-cpp/yaml.h"

#define PFLN ROS_INFO("file %s line %d\n",__FILE__,__LINE__);

namespace atom_rviz_plugin {

    void CalibrationPanel::configSensorsComboBoxChange() {
      /// Function configComboBoxChange()
      /* This function handles the action of alternating between the combobox of the sensors.
       * It checks if the file has read something already and it calls a function with the
       * right arguments in order to adjust the content of the text boxes to the sensor that
       * is currently on the combobox. */

      bool any_text = ui_->paramDescriptionFileTextEdit->toPlainText().isEmpty();
      configLoadParameters(false, !any_text);
    }


    void CalibrationPanel::configLoadParameters(bool clicked /*= true*/, bool comboBoxChanged /*= false*/) {
      /// Function configLoadParameters(bool clicked /*= true*/, bool comboBoxChanged /*= false*/)
      /* This function loads the parameters of the config.yml file into the panel to the respective
       * text boxes. It takes two arguments that have a default value. The argument clicked is true
       * by default, and it is passed as false if there is a need to load the parameters without the
       * button being clicked. The comboboxChanged is false by default, and is called as true when the
       * sensors combobox is changed. */

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
        bool calib_patt_fixed_param_cb;

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
                calib_patt_fixed_param_cb = it2->second.as<bool>();
//                calib_patt_params_content.push_back(it2->second.as<bool>() ? "true" : "false");

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
                  nh.setParam("/" + it->first.as<std::string>() + "/" + it2->first.as<std::string>(), it2->second.as<double>());
                  calib_patt_params_content.push_back(std::to_string(it2->second.as<double>()));

                  ui_->configBorderSizeComboBox->addItem("Scalar");
                  ui_->configBorderSizeComboBox->addItem("Dict.");
                  configBorderSizeSetComboBox("Scalar");
                }
                catch (...) {
                  ui_->configBorderSizeComboBox->addItem("Dict.");
                  ui_->configBorderSizeComboBox->addItem("Scalar");
                  configBorderSizeSetComboBox("Dict.");

                  for (YAML::const_iterator it6 = calibration_pattern_node["border_size"].begin(); it6 != calibration_pattern_node["border_size"].end(); ++it6) {
                    nh.setParam("/" + it->first.as<std::string>() + "/" + it2->first.as<std::string>() + "/" + it6->first.as<std::string>(), it6->second.as<double>());
                    calib_patt_params_content.push_back(std::to_string(it6->second.as<double>()));
                  }
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
        if (comboBoxChanged) {

          ui_->paramSensorsLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[0].c_str()));
          ui_->paramSensorsParentLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[1].c_str()));
          ui_->paramSensorsChildLinkTextEdit->setText(QString::fromUtf8(sensors_params_content[2].c_str()));
          ui_->paramSensorsTopicNameTextEdit->setText(QString::fromUtf8(sensors_params_content[3].c_str()));

        } else {
          ui_->paramDescriptionFileTextEdit->setText(QString::fromUtf8(misc_params_content[0].c_str()));
          ui_->paramBagFileTextEdit->setText(QString::fromUtf8(misc_params_content[1].c_str()));
          ui_->paramWorldLinkTextEdit->setText(QString::fromUtf8(misc_params_content[2].c_str()));
          ui_->paramAnchoredSensorTextEdit->setText(QString::fromUtf8(misc_params_content[3].c_str()));
          ui_->paramMaxDurationTextEdit->setText(QString::fromUtf8(misc_params_content[4].c_str()));

          ui_->paramCalibPatLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[0].c_str()));
          ui_->paramCalibPatParentLinkTextEdit->setText(QString::fromUtf8(calib_patt_params_content[1].c_str()));

          if (calib_patt_fixed_param_cb) {
            ui_->paramCalibPatFixedComboBox->addItem("True");
            ui_->paramCalibPatFixedComboBox->addItem("False");
          } else {
            ui_->paramCalibPatFixedComboBox->addItem("False");
            ui_->paramCalibPatFixedComboBox->addItem("True");
          }

          ui_->paramCalibPatPatternTypeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[2].c_str()));
          ui_->paramCalibPatDictionaryTextEdit->setText(QString::fromUtf8(calib_patt_params_content[3].c_str()));
          ui_->paramCalibPatMeshFileTextEdit->setText(QString::fromUtf8(calib_patt_params_content[4].c_str()));
          ui_->paramCalibPatDimensionXTextEdit->setText(QString::fromUtf8(calib_patt_params_content[5].c_str()));
          ui_->paramCalibPatDimensionYTextEdit->setText(QString::fromUtf8(calib_patt_params_content[6].c_str()));
          ui_->paramCalibPatSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[7].c_str()));
          ui_->paramCalibPatInnerSizeTextEdit->setText(QString::fromUtf8(calib_patt_params_content[8].c_str()));

          QString border_size_scalar_dict = ui_->configBorderSizeComboBox->currentText();
          if (border_size_scalar_dict == "Scalar") {
            ui_->paramBorderSizeScalarTextEdit->setText(QString::fromUtf8(calib_patt_params_content[9].c_str()));
          } else {
            ui_->paramBorderSizeXTextEdit->setText(QString::fromUtf8(calib_patt_params_content[9].c_str()));
            ui_->paramBorderSizeYTextEdit->setText(QString::fromUtf8(calib_patt_params_content[10].c_str()));
          }

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

      // Double parameters
      std::vector <std::string> calib_patt_int_params = {"/calibration_pattern/size",
                                                         "/calibration_pattern/inner_size"};

      double calib_patt_size_param = std::stod(ui_->paramCalibPatSizeTextEdit->toPlainText().toUtf8().constData());
      double calib_patt_inner_size_param = std::stod(ui_->paramCalibPatInnerSizeTextEdit->toPlainText().toUtf8().constData());

      this->nh.setParam(calib_patt_int_params[0], calib_patt_size_param);
      this->nh.setParam(calib_patt_int_params[1], calib_patt_inner_size_param);



      // Boolean parameter
      std::string calib_patt_new_param_bool_str = ui_->paramCalibPatFixedComboBox->currentText().toUtf8().constData();
      bool calib_patt_new_param_bool;
      if (calib_patt_new_param_bool_str == "True") {
        calib_patt_new_param_bool = true;
      } else {
        calib_patt_new_param_bool = false;
      }
      this->nh.setParam("/calibration_pattern/fixed", calib_patt_new_param_bool);


      // Dictionary parameters
      // /calibration_pattern/dimension parameter:
      std::string calib_patt_dimension_x_parameter_str = ui_->paramCalibPatDimensionXTextEdit->toPlainText().toUtf8().constData();
      std::string calib_patt_dimension_y_parameter_str = ui_->paramCalibPatDimensionYTextEdit->toPlainText().toUtf8().constData();

      double calib_patt_dimension_x_parameter = std::stod(calib_patt_dimension_x_parameter_str);
      double calib_patt_dimension_y_parameter = std::stod(calib_patt_dimension_y_parameter_str);

      this->nh.setParam("/calibration_pattern/dimension/x", calib_patt_dimension_x_parameter);
      this->nh.setParam("/calibration_pattern/dimension/y", calib_patt_dimension_y_parameter);

      // /calibration_pattern/border_size parameter:
      double calib_patt_border_size_param, calib_patt_border_size_x_param, calib_patt_border_size_y_param;

      if (ui_->configBorderSizeComboBox->currentText() == "Scalar"){
        calib_patt_border_size_param = std::stod(ui_->paramBorderSizeScalarTextEdit->toPlainText().toUtf8().constData());
        this->nh.setParam("/calibration_pattern/border_size", calib_patt_border_size_param);
      } else {
        calib_patt_border_size_x_param = std::stod(ui_->paramBorderSizeXTextEdit->toPlainText().toUtf8().constData());
        calib_patt_border_size_y_param = std::stod(ui_->paramBorderSizeYTextEdit->toPlainText().toUtf8().constData());

        this->nh.setParam("/calibration_pattern/border_size/x", calib_patt_border_size_x_param);
        this->nh.setParam("/calibration_pattern/border_size/y", calib_patt_border_size_y_param);
      }


      // *********************
      // * Sensors Parameter *
      // *********************
      std::string sensor_to_write = ui_->sensorsComboBox->currentText().toUtf8().constData();

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

      // =======
      // EMITTER
      // =======
      std::string config_ros_package = ui_->configPackageLineEdit->toPlainText().toUtf8().constData();
      std::string config_file_path = ros::package::getPath(config_ros_package) + "/calibration/config.yml";

      YAML::Emitter emitter;

      emitter << YAML::BeginMap;
        emitter << YAML::Key << "description_file" << YAML::DoubleQuoted << YAML::Value << misc_new_param_str[0];
        emitter << YAML::Key << "bag_file" << YAML::DoubleQuoted << YAML::Value << misc_new_param_str[1];
        emitter << YAML::Key << "world_link" << YAML::DoubleQuoted << YAML::Value << misc_new_param_str[2];

        std::vector <QString> all_sensors = getSensors();
        std::string sensor_param_link_path, sensor_param_parent_link_path, sensor_param_child_link_path, sensor_param_topic_name_path;
        std::string sensor_param_link, sensor_param_parent_link, sensor_param_child_link, sensor_param_topic_name;

        emitter << YAML::Key << "sensors" << YAML::Value << YAML::BeginMap;
        for (size_t i = 0; i < all_sensors.size(); i++) {
          std::string sensor_str = all_sensors[i].toUtf8().constData();

          sensor_param_link_path = "/sensors/" + sensor_str + "/link";
          sensor_param_parent_link_path = "/sensors/" + sensor_str + "/parent_link";
          sensor_param_child_link_path = "/sensors/" + sensor_str + "/child_link";
          sensor_param_topic_name_path = "/sensors/" + sensor_str + "/topic_name";

          this->nh.getParam(sensor_param_link_path, sensor_param_link);
          this->nh.getParam(sensor_param_parent_link_path, sensor_param_parent_link);
          this->nh.getParam(sensor_param_child_link_path, sensor_param_child_link);
          this->nh.getParam(sensor_param_topic_name_path, sensor_param_topic_name);

          emitter << YAML::Key << sensor_str << YAML::DoubleQuoted << YAML::Value << YAML::BeginMap;
            emitter << YAML::Key << "link" << YAML::DoubleQuoted << YAML::Value << sensor_param_link;
            emitter << YAML::Key << "parent_link" << YAML::DoubleQuoted << YAML::Value << sensor_param_parent_link;
            emitter << YAML::Key << "child_link" << YAML::DoubleQuoted << YAML::Value << sensor_param_child_link;
            emitter << YAML::Key << "topic_name" << YAML::DoubleQuoted << YAML::Value << sensor_param_topic_name;
          emitter << YAML::EndMap;
        }
        emitter << YAML::EndMap;

        emitter << YAML::Key << "calibration_pattern" << YAML::Value << YAML::BeginMap;
          emitter << YAML::Key << "link" << YAML::DoubleQuoted << YAML::Value << calib_patt_new_param_str[0];
          emitter << YAML::Key << "parent_link" << YAML::DoubleQuoted << YAML::Value << calib_patt_new_param_str[1];
          emitter << YAML::Key << "fixed" << YAML::Value << calib_patt_new_param_bool;
          emitter << YAML::Key << "pattern_type" << YAML::DoubleQuoted << YAML::Value << calib_patt_new_param_str[2];
          emitter << YAML::Key << "dictionary" << YAML::DoubleQuoted << YAML::Value << calib_patt_new_param_str[3];
          emitter << YAML::Key << "mesh_file" << YAML::DoubleQuoted << YAML::Value << calib_patt_new_param_str[4];
          if (ui_->configBorderSizeComboBox->currentText() == "Scalar"){
            emitter << YAML::Key << "border_size" << YAML::Value << calib_patt_border_size_param;
          } else {
            emitter << YAML::Key << "border_size" << YAML::Value << YAML::BeginMap;
              emitter << YAML::Key << "x" << YAML::Value << calib_patt_border_size_x_param;
              emitter << YAML::Key << "y" << YAML::Value << calib_patt_border_size_y_param;
            emitter << YAML::EndMap;
          }
          emitter << YAML::Key << "dimension" << YAML::Value << YAML::BeginMap;
            emitter << YAML::Key << "x" << YAML::Value << calib_patt_dimension_x_parameter;
            emitter << YAML::Key << "y" << YAML::Value << calib_patt_dimension_y_parameter;
          emitter << YAML::EndMap;
          emitter << YAML::Key << "size" << YAML::Value << calib_patt_size_param;
          emitter << YAML::Key << "inner_size" << YAML::Value << calib_patt_inner_size_param;

        emitter << YAML::EndMap;

        emitter << YAML::Key << "anchored_sensor" << YAML::DoubleQuoted << YAML::Value << misc_new_param_str[3];
        emitter << YAML::Key << "max_duration_between_msgs" << YAML::Value << misc_new_param_int;

      emitter << YAML::EndMap;

      std::ofstream fout(config_file_path);
      fout << emitter.c_str(); // dump it back into the file

      std::string python_script_path = ros::package::getPath("atom_rviz_plugin") + "/scripts/copy_content_to_yaml.py";
      std::string yaml_format_path = ros::package::getPath("atom_rviz_plugin") + "/scripts/test_config_format.yml";

      std::string command_str = "python3 " + python_script_path + " -yc " + config_file_path + " -yf " + yaml_format_path + " -yo " + config_file_path;
      const char *command = command_str.c_str();
      system(command);
//      std::cout << "Running command " << std::endl << command << std::endl;

    }  // function writeButtonClicked

//##########################################################

    void CalibrationPanel::configBorderSizeSetComboBox(QString combobox_str) {

      if (combobox_str == "Scalar") {
        ui_->paramBorderSizeXLabel->setVisible(false);
        ui_->paramBorderSizeYLabel->setVisible(false);
        ui_->paramBorderSizeXTextEdit->setVisible(false);
        ui_->paramBorderSizeYTextEdit->setVisible(false);
        ui_->paramBorderSizeScalarTextEdit->setVisible(true);
      } else {
        ui_->paramBorderSizeXLabel->setVisible(true);
        ui_->paramBorderSizeYLabel->setVisible(true);
        ui_->paramBorderSizeXTextEdit->setVisible(true);
        ui_->paramBorderSizeYTextEdit->setVisible(true);
        ui_->paramBorderSizeScalarTextEdit->setVisible(false);
      }
    }

}