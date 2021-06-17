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
        for (int i = 1; i < ui_->calibTableWidget->rowCount()-4; i++) {
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

      std::string command_line_text_box = ui_->calibCommandLineEdit->text().toUtf8().constData();
      std::string argument;
      size_t pos;

      // JSON File argument
/*      QTableWidgetItem *json_file_cell = ui_->calibTableWidget->item(0, 0);
      QString str = json_file_cell->text();
      std::string json_file = json_file_cell->text().toUtf8().constData();
      if (not json_file.empty()) {

//       ui_->calibCommandLineEdit->insert(QString::fromUtf8((" -json dataset_file:=" + json_file).c_str()));
      }*/

      // View Optimization argument
      argument = "-vo ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget1 = ui_->calibTableWidget->cellWidget(1, 0);
      QCheckBox *checkbox1 = pWidget1->findChild<QCheckBox *>();

      if (checkbox1->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }

      // Verbose argument
      argument = "-v ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget2 = ui_->calibTableWidget->cellWidget(2, 0);
      QCheckBox *checkbox2 = pWidget2->findChild<QCheckBox *>();
      if (checkbox2->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }

      // ROS Visualization argument
      argument = "-rv ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget3 = ui_->calibTableWidget->cellWidget(3, 0);
      QCheckBox *checkbox3 = pWidget3->findChild<QCheckBox *>();
      if (checkbox3->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }

      // Show Images argument
      argument = "-si ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget4 = ui_->calibTableWidget->cellWidget(4, 0);
      QCheckBox *checkbox4 = pWidget4->findChild<QCheckBox *>();
      if (checkbox4->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }


      // Optimize Intrinsics argument
      argument = "-oi ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget5 = ui_->calibTableWidget->cellWidget(5, 0);
      QCheckBox *checkbox5 = pWidget5->findChild<QCheckBox *>();
      if (checkbox5->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }


      // Profile Objective Function argument
      argument = "-pof ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget6 = ui_->calibTableWidget->cellWidget(6, 0);
      QCheckBox *checkbox6 = pWidget6->findChild<QCheckBox *>();
      if (checkbox6->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }


      // Use Incomplete Collections argument
      argument = "-uic ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget7 = ui_->calibTableWidget->cellWidget(7, 0);
      QCheckBox *checkbox7 = pWidget7->findChild<QCheckBox *>();
      if (checkbox7->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }

      // Remove Partial Detections argument
      argument = "-rpd ";
      pos = command_line_text_box.find(argument);
      QWidget *pWidget8 = ui_->calibTableWidget->cellWidget(8, 0);
      QCheckBox *checkbox8 = pWidget8->findChild<QCheckBox *>();
      if (checkbox8->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }

      ui_->calibCommandLineEdit->setText(QString::fromUtf8(command_line_text_box.c_str()));
    } //function calibTableCHanged()

    void CalibrationPanel::calibHelpButtonClicked(){

      QMessageBox messageBox;
      QSpacerItem* horizontalSpacer = new QSpacerItem(5500, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);
      messageBox.setIcon(QMessageBox::Information);
      messageBox.setWindowTitle("Help");
      messageBox.setText("                                                               Command-line arguments description            \n"
                         "                                                               ---------------------------------------------------------             \n"
                         "  - JSON File: Json file path containing input dataset.\n\n"
                         "  - View Optimization: Displays generic total error and residuals graphs.\n\n"
                         "  - Verbose: Be verbose\n\n"
                         "  - ROS Visualization: Publish ros visualization markers.\n\n"
                         "  - Show Images: Shows images for each camera.\n\n"
                         "  - Optimize Intrinsics: Adds camera intrinsics and distortion to the optimization.\n\n"
                         "  - Profile Objective Function: Runs and prints a profile of the objective function then exits.\n\n"
                         "  - Use Incomplete Collections: Remove any collection which does not have a detection for all sensors.\n\n"
                         "  - Remove Partial Detections: Remove detected labels which are only partial. Used or the Charuco.\n\n"
                         "  - Sample Residuals: Samples residuals.\n\n"
                         "  - Sample seed: Samples seed.\n\n"
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