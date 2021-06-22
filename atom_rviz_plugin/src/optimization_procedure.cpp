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
        for (int i = 3; i < ui_->calibTableWidget->rowCount()-7; i++) {
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

        system(command);

//        ROS_INFO_STREAM(command_line);

      } catch (...) {
        QMessageBox messageBox;
        messageBox.critical(0,"Error","Error trying to run the calibration!");
      }

    } //function calibCalibrateButtonClicked()


    void CalibrationPanel::calibCopyButtonClicked(){
      try {
        QClipboard *clip = QApplication::clipboard();
//        QString input = ui_->calibCommandTextEdit->text();
        QString input = ui_->calibCommandTextEdit->toPlainText();

        clip->setText(input);
      } catch (...) {
        return;
      }
    } //function calibCalibrateButtonClicked()

    void CalibrationPanel::calibTableChanged(){

      std::string command_line_text_box = ui_->calibCommandTextEdit->toPlainText().toUtf8().constData();
      std::string argument;
      size_t pos;
      QWidget *pWidget;
      QCheckBox *checkbox;
      std::vector <std::string> checkbox_arguments = {"-vpv ","-vo ","-v ","-phased ","-rv ","-si ",
                                                      "-oi ","-pof ","-ajf ","-uic ","-rpd ","-ipg "};
      std::string argument_to_erase;
      QTableWidgetItem *argument_cell;
      size_t string_argument_pos;
      std::vector <std::string> string_arguments = {"-json ", "-sv ", "-z ","-nig ","-sr ",
                                                    "-ss ", "-ssf ", "-csf ", "-oj ", "-ox "};
      std::vector <int> string_arguments_idx = {0, 1, 2, 15, 16, 17, 18, 19, 20, 21};
      std::string argument_content;

//"""""""""""""""""""""""""""""""""""""""""
      for (int i = 0; i < string_arguments.size(); i++) {
        argument_to_erase = command_line_text_box;
        argument = string_arguments[i];
        pos = argument_to_erase.find(argument);

        if (pos != std::string::npos) { //there is a "-... " argument in the command-line edit box
          argument_to_erase.erase(0, pos);
          string_argument_pos = argument_to_erase.find("-",1);
          if (string_argument_pos != std::string::npos) { //there is a second "-" in the line edit
            argument_to_erase.erase(string_argument_pos, argument_to_erase.length());
          }
          string_argument_pos = command_line_text_box.find(argument_to_erase);
          command_line_text_box.erase(string_argument_pos,argument_to_erase.length()); //erases the argument
        }

        argument_cell = ui_->calibTableWidget->item(string_arguments_idx[i], 0);
        if (argument_cell){ //argument cell is not empty
          argument_content = argument_cell->text().toUtf8().constData();
          if (not argument_content.empty()) {  //empty string in the cell
            command_line_text_box = command_line_text_box + argument + argument_content + " ";
          }
        }
      }


    // Checkbox arguments
    for (int i = 0; i < checkbox_arguments.size(); i++) {
      argument = checkbox_arguments[i];
      pos = command_line_text_box.find(argument);
      pWidget = ui_->calibTableWidget->cellWidget(i+3, 0);
      checkbox = pWidget->findChild<QCheckBox *>();

      if (checkbox->isChecked()) {
        if (pos == std::string::npos) {
          command_line_text_box = command_line_text_box + argument;
        }
      } else {
        if (pos != std::string::npos) {
          command_line_text_box.erase(pos, argument.length());
        }
      }
    }
      ui_->calibCommandTextEdit->setText(QString::fromUtf8(command_line_text_box.c_str()));
    } //function calibTableCHanged()

    void CalibrationPanel::calibHelpButtonClicked(){

      QMessageBox messageBox;
      QSpacerItem* horizontalSpacer = new QSpacerItem(20000, 0, QSizePolicy::Minimum, QSizePolicy::Expanding);
      messageBox.setIcon(QMessageBox::Information);
      messageBox.setWindowTitle("Help");
      messageBox.setText("                                                               Command-line arguments description            \n"
                         "                                                               ---------------------------------------------------------             \n"
                         "  - JSON File: Json file path containing input dataset.\n"
                         "  - Skip Vertices: Useful for fast testing.\n"
                         "  - Inconsistency Threshold: Threshold for max z inconsistency value.\n"
                         "  - View Projected Vertices: Visualize Projections of vertices onto images.\n"
                         "  - View Optimization: Displays generic total error and residuals graphs.\n"
                         "  - Verbose: Be verbose\n"
                         "  - Phased: Stay in a loop before calling optimization, and in another after calling the optimization. Good for debugging\n"
                         "  - ROS Visualization: Publish ros visualization markers.\n"
                         "  - Show Images: Shows images for each camera.\n"
                         "  - Optimize Intrinsics: Adds camera intrinsics and distortion to the optimization.\n"
                         "  - Profile Objective Function: Runs and prints a profile of the objective function then exits.\n"
                         "  - All Joints Fixed: Assume all joints are fixed and because of that draw a single robot mesh. Overrides automatic detection of static robot.\n"
                         "  - Use Incomplete Collections: Remove any collection which does not have a detection for all sensors.\n"
                         "  - Remove Partial Detections: Remove detected labels which are only partial. Used or the Charuco.\n"
                         "  - Initial Pose Ghost: Draw a ghost mesh with the systems initial pose. Good for debugging.\n"
                         "  - Noisy Initial Guess: Percentage of noise to add to the initial guess atomic transformations set before.\n"
                         "  - Sample Residuals: Samples residuals.\n"
                         "  - Sample seed: Samples seed.\n"
                         "  - Sensor select function: A string to be evaluated into a lambda function that "
                         "receives a sensor name as input and returns Tru or False"
                         " to indicate if the sensor should be loaded (and used in the optimization)."
                         " The Syntax is lambda name: f(x), where f(x) is the "
                         "function in python language. Example: lambda name: "
                         "name in [\"left_laser\",\"frontal_camera\"], to load "
                         "sensors left_laser and frontal_camera\n"
                         "  - Collection Selection Function: A string to be evaluated into a lambda function that "
                         "receives a collection name as input and returns True or False "
                         "to indicate if the collection should be loaded (and used in the optimization)."
                         " The Syntax is lambda name: f(x), where f(x) is the "
                         "function in python language. Example: lambda name: "
                         "int(name) > 5, to load only collections 6, 7, and onward.\n"
                         "  - Output Json: Full path to output json file.\n"
                         "  - Output Xacro: Full path to output xacro file.");
      QGridLayout* layout = (QGridLayout*)messageBox.layout();
      layout->addItem(horizontalSpacer, layout->rowCount(), 0, 1, layout->columnCount());
      messageBox.exec();
    } //function calibHelpButtonClicked()
}  // namespace atom_rviz_plugin