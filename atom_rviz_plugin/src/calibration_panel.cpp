#include <atom_rviz_plugin/calibration_panel.h>

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
      ros::init(argc, NULL, "rviz_panel");
    }

    CalibrationPanel::~CalibrationPanel() = default;

    void CalibrationPanel::onInitialize()
    {
      // Functions to run when rviz opens
      getSensors();
      initEstimateComboBoxTextChanged();

      // Qt events for buttons, checkboxes, labels, combobox,...
      connect(ui_->configReadButton, SIGNAL(clicked()), this, SLOT(configReadButtonClicked()));
      connect(ui_->configWriteButton, SIGNAL(clicked()), this, SLOT(configWriteButtonClicked()));

      connect(ui_->initEstimateSensorsComboBox, SIGNAL(currentTextChanged(QString)), this, SLOT(initEstimateComboBoxTextChanged()));
      connect(ui_->initialEstimateCheckBox, SIGNAL(clicked(bool)), this, SLOT(initEstimateCheckboxOrSpinBoxInputChanged()));
      connect(ui_->initialEstimateSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateCheckboxOrSpinBoxInputChanged()));
      connect(ui_->initEstimateSaveButton, SIGNAL(clicked()), this, SLOT(initEstimateSaveButtonClicked()));
      connect(ui_->initEstimateResetSensorButton, SIGNAL(clicked()), this, SLOT(initEstimateResetButtonClicked()));
      connect(ui_->initEstimateResetAllButton, SIGNAL(clicked()), this, SLOT(initEstimateResetAllButtonClicked()));

      parentWidget()->setVisible(true);

    } //function onInitialize()

    void CalibrationPanel::getSensors()
    {
      // Get number of sensors to put on ComboBox for the configuration file
      std::vector<std::string> parameters;
      nh.getParamNames(parameters);

      std::vector <QString> sensors;

      ui_->initEstimateSensorsComboBox->addItem("");
      for (size_t i = 0; i < parameters.size(); i++) {
        std::string parameters_i = parameters[i];
        size_t idx = parameters_i.find("/sensors/");
        if (idx!=std::string::npos) {
          size_t idx_2 = parameters_i.find("/topic_name");
          if (idx_2!=std::string::npos) {
            int sensor_length = parameters_i.size() - std::string("/sensors/").size() - std::string("/topic_name").size();
            std::string sensor_name = parameters_i.substr(std::string("/sensors/").length(),sensor_length);
            QString str_to_combo_box = QString::fromUtf8(sensor_name.c_str());
            ui_->sensorsComboBox->addItem(str_to_combo_box);
            ui_->initEstimateSensorsComboBox->addItem(str_to_combo_box);
            sensors.push_back(str_to_combo_box);


            // ListWidget for sensors in the initial estimate tab
            QListWidgetItem* item = new QListWidgetItem("             " + str_to_combo_box, ui_->initEstimateSensorListWidget);
//            ui_->initEstimateSensorListWidget->addItem(item);
//            ui_->initEstimateSensorListWidget->setItemWidget(item,new QCheckBox());
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
            item->setCheckState(Qt::Unchecked); // AND initialize check state
          }
        }
      }
      setTable(sensors);
    } //function getSensors()

    void CalibrationPanel::setTable(std::vector <QString> sensors_for_table){
      ui_->tableWidget->verticalHeader()->setVisible(false);
//      ui_->tableWidget->horizontalHeader()->setVisible(false);
//      ui_->tableWidget->setFrameStyle(QFrame::NoFrame);
      ui_->tableWidget->setColumnCount(2);
      ui_->tableWidget->setColumnWidth(0, 150);
      ui_->tableWidget->setColumnWidth(1, 20);

      for (size_t i = 0; i < sensors_for_table.size(); i++) {
        // TableWidget for sensors in the initial estimate tab
        ui_->tableWidget->insertRow( ui_->tableWidget->rowCount() );

        QTableWidgetItem *sensors_header = new QTableWidgetItem();
        sensors_header->setText("Sensors");
        ui_->tableWidget->setHorizontalHeaderItem(0,sensors_header);

        QTableWidgetItem *scale_header = new QTableWidgetItem();
        scale_header->setText("Scale");
        ui_->tableWidget->setHorizontalHeaderItem(1,scale_header);

        QString sensor = sensors_for_table[i];
        QTableWidgetItem *sensor_item = new QTableWidgetItem(sensor);

        sensor_item->setCheckState(Qt::Checked);
        sensor_item->setFlags(sensor_item->flags() ^ Qt::ItemIsEditable);

        ui_->tableWidget->setItem(ui_->tableWidget->rowCount()-1, 0, sensor_item);
        ui_->tableWidget->setCellWidget(ui_->tableWidget->rowCount()-1, 1, new QSpinBox);
      }
    } //function setTable()
}  //namespace atom_rviz_plugin