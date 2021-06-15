#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"

#include <QHBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QMessageBox>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::calibSetTable(){
      try {
        QTableWidgetItem *item0(ui_->calibTableWidget->item(0, 0));
        if (item0 != 0) {
          return;
        }

        for (int i = 0; i < ui_->calibTableWidget->rowCount(); i++) {
//          ROS_INFO_STREAM("Adding checkbox");

          QWidget *pWidget = new QWidget();
          QCheckBox *pCheckBox = new QCheckBox();
          QHBoxLayout *pLayout = new QHBoxLayout(pWidget);
          pLayout->addWidget(pCheckBox);
          pLayout->setAlignment(Qt::AlignCenter);
          pLayout->setContentsMargins(0, 0, 0, 0);
          pWidget->setLayout(pLayout);
          pCheckBox->setCheckState(Qt::Unchecked);
          ui_->calibTableWidget->setCellWidget(i, 0, pWidget);
        }

      } catch (...) {
        return;
      }
    } //function calibSetTable()

    void CalibrationPanel::calibHelpButtonClicked(){

      if (ui_->calibTextBrowser->isVisible()) {
        ui_->calibTextBrowser->setVisible(false);
      } else {
        ui_->calibTextBrowser->setVisible(true);
        ui_->calibTextBrowser->raise();
      }

      QMessageBox messageBox;
      messageBox.information(0,"Help","usage: calibrate [-h] [-sv SKIP_VERTICES] [-z Z_INCONSISTENCY_THRESHOLD]\n"
                                    "                 [-vpv] [-vo] -json JSON_FILE [-v] [-rv] [-si] [-oi] [-pof]\n"
                                    "                 [-sr SAMPLE_RESIDUALS] [-ss SAMPLE_SEED] [-od] [-fec] [-uic]\n"
                                    "                 [-rpd] [-ssf SENSOR_SELECTION_FUNCTION]\n"
                                    "                 [-csf COLLECTION_SELECTION_FUNCTION]\n"
                                    "\n"
                                    "optional arguments:\n"
                                    "  -h, --help            show this help message and exit\n"
                                    "  -json JSON_FILE, --json_file JSON_FILE\n"
                                    "                        Json file containing input dataset.\n"
                                    "  -vo, --view_optimization\n"
                                    "                        Displays generic total error and residuals graphs.\n"
                                    "  -v, --verbose         Be verbose\n"
                                    "  -rv, --ros_visualization\n"
                                    "                        Publish ros visualization markers.\n"
                                    "  -si, --show_images    shows images for each camera\n"
                                    "  -oi, --optimize_intrinsics\n"
                                    "                        Adds camera instrinsics and distortion to the optimization\n"
                                    "  -pof, --profile_objective_function\n"
                                    "                        Runs and prints a profile of the objective function,\n"
                                    "                        then exits.\n"
                                    "  -sr SAMPLE_RESIDUALS, --sample_residuals SAMPLE_RESIDUALS\n"
                                    "                        Samples residuals\n"
                                    "  -ss SAMPLE_SEED, --sample_seed SAMPLE_SEED\n"
                                    "                        Sampling seed\n"
                                    "  -uic, --use_incomplete_collections\n"
                                    "                        Remove any collection which does not have a detection\n"
                                    "                        for all sensors.\n"
                                    "  -rpd, --remove_partial_detections\n"
                                    "                        Remove detected labels which are only partial. Used or\n"
                                    "                        the Charuco.\n"
                                    "  -ssf SENSOR_SELECTION_FUNCTION, --sensor_selection_function SENSOR_SELECTION_FUNCTION\n"
                                    "                        A string to be evaluated into a lambda function that\n"
                                    "                        receives a sensor name as input and returns True or\n"
                                    "                        False to indicate if the sensor should be loaded (and\n"
                                    "                        used in the optimization). The Syntax is lambda name:\n"
                                    "                        f(x), where f(x) is the function in python language.\n"
                                    "                        Example: lambda name: name in [\"left_laser\",\n"
                                    "                        \"frontal_camera\"] , to load only sensors left_laser\n"
                                    "                        and frontal_camera\n"
                                    "  -csf COLLECTION_SELECTION_FUNCTION, --collection_selection_function COLLECTION_SELECTION_FUNCTION\n"
                                    "                        A string to be evaluated into a lambda function that\n"
                                    "                        receives a collection name as input and returns True\n"
                                    "                        or False to indicate if the collection should be\n"
                                    "                        loaded (and used in the optimization). The Syntax is\n"
                                    "                        lambda name: f(x), where f(x) is the function in\n"
                                    "                        python language. Example: lambda name: int(name) > 5 ,\n"
                                    "                        to load only collections 6, 7, and onward.");

    } //function calibHelpButtonClicked()

    void CalibrationPanel::calibCalibrateButtonClicked(){

      try {
        std::string robotic_system = ui_->configPackageLineEdit->toPlainText().toUtf8().constData();

        // Read values from table

        std::string command_line = "rosrun " + robotic_system + "_calibration";
        const char *command = command_line.c_str();

//      system(command);

        ROS_INFO_STREAM(command_line);

      } catch (...) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","Error trying to run the calibration!");
      }

    } //function calibCalibrateButtonClicked()

}  // namespace atom_rviz_plugin
