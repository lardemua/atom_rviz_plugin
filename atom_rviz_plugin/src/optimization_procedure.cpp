#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"

#include <QHBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QMessageBox>
#include <QClipboard>
#include <QSpacerItem>
#include <QGridLayout>

#include "ui_calibration_panel.h"

namespace atom_rviz_plugin
{
    void CalibrationPanel::calibSetTable(){
      try {
        QTableWidgetItem *item0(ui_->calibTableWidget->item(0, 0));
        if (item0 != 0) {
          return;
        }

        QTableWidgetItem *item = new QTableWidgetItem;
        item->setFlags(item->flags() | Qt::ItemIsEditable);
        ui_->calibTableWidget->setItem(0,0,item);

        for (int i = 1; i < ui_->calibTableWidget->rowCount()-2; i++) {
          QWidget *pWidget = new QWidget();
          QCheckBox *pCheckBox = new QCheckBox();
          QHBoxLayout *pLayout = new QHBoxLayout(pWidget);
          pLayout->addWidget(pCheckBox);
          pLayout->setAlignment(Qt::AlignCenter);
          pLayout->setContentsMargins(0, 0, 0, 0);
          pWidget->setLayout(pLayout);
          pCheckBox->setCheckState(Qt::Unchecked);
          ui_->calibTableWidget->setCellWidget(i, 0, pWidget);
          connect(pCheckBox, SIGNAL(clicked()), this, SLOT(calibTableChanged()));
        }

      } catch (...) {
        return;
      }
    } //function calibSetTable()

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


    void CalibrationPanel::calibCopyButtonClicked(){
      try {
        QClipboard *clip = QApplication::clipboard();
        QString input = ui_->calibCommandLineEdit->text();
        clip->setText(input);
      } catch (...) {
        return;
      }
    } //function calibCalibrateButtonClicked()

    void CalibrationPanel::calibTableChanged(){
      ROS_INFO_STREAM("Ola");
    } //function calibTableCHanged()

    void CalibrationPanel::calibHelpButtonClicked(){

      QMessageBox messageBox;
      QSpacerItem* horizontalSpacer = new QSpacerItem(5500, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);
      messageBox.setIcon(QMessageBox::Information);
      messageBox.setWindowTitle("Help");
      messageBox.setText("                                                               Command-line arguments description            \n"
                         "                                                               ---------------------------------------------------------             \n"
                         "  - JSON File: Json file containing input dataset.\n\n"
                         "  - View Optimization: Displays generic total error and residuals graphs.\n\n"
                         "  - Verbose: Be verbose\n\n"
                         "  - ROS Visualization: Publish ros visualization markers.\n\n"
                         "  - Show Images: Shows images for each camera.\n\n"
                         "  - Optimize Intrinsics: Adds camera intrinsics and distortion to the optimization.\n\n"
                         "  - Profile Objective function: Runs and prints a profile of the objective function then exits.\n\n"
                         "  - Sample Residuals: Samples residuals.\n\n"
                         "  - Sample seed: Samples seed.\n\n"
                         "  - Use Incomplete Collections: Remove any collection which does not have a detection for all sensors.\n\n"
                         "  - Remove partial detections: Remove detected labels which are only partial. Used or the Charuco.\n\n"
                         "  - Sensor select function: A string to be evaluated into a lambda function that "
                         "receives a sensor name as input and returns Tru or False"
                         " to indicate if the sensor should be loaded (and used in the optimization)."
                         " The Syntax is lambda name: f(x), where f(x) is the "
                         "function in python language. Example: lambda name: "
                         "name in [\"left_laser\",\"frontal_camera\"], to load "
                         "sensors left_laser and frontal_camera\n\n"
                         "  - Collection Selection Function: A string to be evaluated into a lambda function that "
                         "receives a collection name as input and returns True or False "
                         "to indicate if the collection should be loaded (and used in the optimization)."
                         " The Syntax is lambda name: f(x), where f(x) is the "
                         "function in python language. Example: lambda name: "
                         "int(name) > 5, to load only collections 6, 7, and onward.");
      QGridLayout* layout = (QGridLayout*)messageBox.layout();
      layout->addItem(horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
      messageBox.exec();
    } //function calibHelpButtonClicked()
}  // namespace atom_rviz_plugin