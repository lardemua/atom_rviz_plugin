#include <atom_rviz_plugin/calibration_panel.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <atom_msgs/GetSensorInteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>

#include "ui_calibration_panel.h"


#define PFLN ROS_INFO("file %s line %d\n",__FILE__,__LINE__);

namespace atom_rviz_plugin
{
    CalibrationPanel::CalibrationPanel(QWidget* parent) : Panel(parent),  ui_(new Ui::CalibUI())
    {
      ui_->setupUi(this);
      int argc = 0;
      char** argv;
      ros::init(argc, NULL, "calibration_panel");
    }

    CalibrationPanel::~CalibrationPanel() = default;

    void CalibrationPanel::onInitialize()
    {
// Functions to run when rviz opens
      handleTabs(-1);
      getSensors();
      calibSetTable();

      ui_->collectDataDeleteCollectionLabel->setVisible(false);

      ui_->paramBorderSizeXLabel->setVisible(false);
      ui_->paramBorderSizeYLabel->setVisible(false);
      ui_->paramBorderSizeXTextEdit->setVisible(false);
      ui_->paramBorderSizeYTextEdit->setVisible(false);
      ui_->paramBorderSizeScalarTextEdit->setVisible(true);

      // Qt events for buttons, checkboxes, labels, combobox,...
      connect(ui_->mainTabs, SIGNAL(currentChanged(int)), this, SLOT(handleTabs(int)));

      // Configuration Tab
      connect(ui_->configWriteButton, SIGNAL(clicked()), this, SLOT(configWriteButtonClicked()));
      connect(ui_->configLoadButton, SIGNAL(clicked(bool)), this, SLOT(configLoadParameters()));
      connect(ui_->sensorsComboBox, SIGNAL(currentTextChanged(QString)), this, SLOT(configSensorsComboBoxChange()));
      connect(ui_->configBorderSizeComboBox, SIGNAL(currentTextChanged(QString)), this, SLOT(configBorderSizeSetComboBox(QString)));

      // Initial Estimate Tab
      connect(ui_->initEstimateSaveButton, SIGNAL(clicked()), this, SLOT(initEstimateSaveButtonClicked()));
      connect(ui_->initEstimateResetSensorButton, SIGNAL(clicked()), this, SLOT(initEstimateResetButtonClicked()));
      connect(ui_->initEstimateResetAllButton, SIGNAL(clicked()), this, SLOT(initEstimateResetAllButtonClicked()));
      connect(ui_->initEstimateTableWidget, SIGNAL(cellClicked(int,int)), this, SLOT(initEstimateSensorsCellClicked(int,int)));

      connect(ui_->initEstimatePoseXSlider, SIGNAL(valueChanged(int)), this, SLOT(initEstimateSliderToSpin(int)));
      connect(ui_->initEstimatePoseYSlider, SIGNAL(valueChanged(int)), this, SLOT(initEstimateSliderToSpin(int)));
      connect(ui_->initEstimatePoseZSlider, SIGNAL(valueChanged(int)), this, SLOT(initEstimateSliderToSpin(int)));
      connect(ui_->initEstimatePoseRollSlider, SIGNAL(valueChanged(int)), this, SLOT(initEstimateSliderToSpin(int)));
      connect(ui_->initEstimatePosePitchSlider, SIGNAL(valueChanged(int)), this, SLOT(initEstimateSliderToSpin(int)));
      connect(ui_->initEstimatePoseYawSlider, SIGNAL(valueChanged(int)), this, SLOT(initEstimateSliderToSpin(int)));

      connect(ui_->initEstimatePoseXDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinToSlider(double)));
      connect(ui_->initEstimatePoseYDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinToSlider(double)));
      connect(ui_->initEstimatePoseZDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinToSlider(double)));
      connect(ui_->initEstimatePoseRollDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinToSlider(double)));
      connect(ui_->initEstimatePosePitchDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinToSlider(double)));
      connect(ui_->initEstimatePoseYawDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(initEstimateSpinToSlider(double)));

      // Collect Data Tab
      connect(ui_->collectDataSaveButton, SIGNAL(clicked()), this, SLOT(collectDataSaveButtonClicked()));
      connect(ui_->collectDataDeleteButton, SIGNAL(clicked()), this, SLOT(collectDataDeleteButtonClicked()));
      connect(ui_->collectDataTreeWidget, SIGNAL(itemClicked(QTreeWidgetItem *, int)), this, SLOT(collectDataCheckItem(QTreeWidgetItem*, int)));
      connect(ui_->collectDataSensorsComboBox, SIGNAL(currentTextChanged(QString)), this, SLOT(getLidarSensorPosition()));

      connect(ui_->collectDataPoseXSlider, SIGNAL(valueChanged(int)), this, SLOT(collectDataSliderToSpin(int)));
      connect(ui_->collectDataPoseYSlider, SIGNAL(valueChanged(int)), this, SLOT(collectDataSliderToSpin(int)));
      connect(ui_->collectDataPoseZSlider, SIGNAL(valueChanged(int)), this, SLOT(collectDataSliderToSpin(int)));

      connect(ui_->collectDataPoseXDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(collectDataSpinToSlider(double)));
      connect(ui_->collectDataPoseYDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(collectDataSpinToSlider(double)));
      connect(ui_->collectDataPoseZDoubleSpinBox, SIGNAL(valueChanged(double)), this, SLOT(collectDataSpinToSlider(double)));

      connect(ui_->calibHelpButton, SIGNAL(clicked()), this, SLOT(calibHelpButtonClicked()));
      connect(ui_->calibCalibrateButton, SIGNAL(clicked()), this, SLOT(calibCalibrateButtonClicked()));
      connect(ui_->calibCopyPushButton, SIGNAL(clicked()), this, SLOT(calibCopyButtonClicked()));
      connect(ui_->calibTableWidget, SIGNAL(cellChanged(int,int)), this, SLOT(calibTableChanged()));

      parentWidget()->setVisible(true);

    } //function onInitialize()

    // Function to control what happens every time each tab of the panel is opened
    void CalibrationPanel::handleTabs(int i) {
      if (ui_->mainTabs->currentWidget() == ui_->configTab){

        ui_->tabDescriptionLabel->setText("Configuration of the calibration parameters");

      } else if (ui_->mainTabs->currentWidget() == ui_->initEstimateTab){

        ui_->tabDescriptionLabel->setText("Set initial estimate of the sensors' pose");
        initEstimateSetTable();

      } else if (ui_->mainTabs->currentWidget() == ui_->dataCollectTab){

        ui_->tabDescriptionLabel->setText("Collect data from the sensors");
        if (i<0){
          setDataCollectComboBox();
        }

      } else if (ui_->mainTabs->currentWidget() == ui_->calibrationTab){

        ui_->tabDescriptionLabel->setText("Optimization procedure");
      }
      return;
    } // function handleTabs()

    // Function to get the sensors of the robotic system
    std::vector <QString> CalibrationPanel::getSensors()
    {
      // Get number of sensors to put on ComboBox for the configuration file
      std::vector<std::string> parameters;
      nh.getParamNames(parameters);

      std::vector <QString> sensors;

//      ui_->initEstimateSensorsComboBox->addItem("");
      for (size_t i = 0; i < parameters.size(); i++) {
        std::string parameters_i = parameters[i];
        size_t idx = parameters_i.find("/sensors/");
        if (idx != std::string::npos) {
          size_t idx_2 = parameters_i.find("/topic_name");
          if (idx_2 != std::string::npos) {
            int sensor_length =
                parameters_i.size() - std::string("/sensors/").size() - std::string("/topic_name").size();
            std::string sensor_name = parameters_i.substr(std::string("/sensors/").length(), sensor_length);
            QString str_to_combo_box = QString::fromUtf8(sensor_name.c_str());
            sensors.push_back(str_to_combo_box);
          }
        }
      }
      return sensors;
    } //function getSensors()

    void CalibrationPanel::getLidarSensorPosition(){
///*  // Function to get the Interactive Marker's position of the sensor in the combobox
      std::string sensor_in_cb = ui_->collectDataSensorsComboBox->currentText().toUtf8().constData();

      try {
        boost::shared_ptr<visualization_msgs::InteractiveMarkerUpdate const> msg;
        msg = ros::topic::waitForMessage<visualization_msgs::InteractiveMarkerUpdate>("/data_labeler/update", ros::Duration(100));

        std::string test;
        visualization_msgs::InteractiveMarkerUpdate update_msg;

        if (msg != NULL) {
          update_msg = *msg;

          for (size_t i=0; i<update_msg.markers.size(); i++) {
            if (update_msg.markers[i].name == sensor_in_cb) {
              ui_->collectDataPoseXSlider->setValue(update_msg.markers[i].pose.position.x*100);
              ui_->collectDataPoseYSlider->setValue(update_msg.markers[i].pose.position.y*100);
              ui_->collectDataPoseZSlider->setValue(update_msg.markers[i].pose.position.z*100);
            }
          }
        }
      } catch(...) {
        return;
      }
    } //  function getLidarSensorPosition()


    void CalibrationPanel::setDataCollectComboBox(){
    /// Set Combo Box of Data collect Tab
      try {
        ros::master::V_TopicInfo master_topics;
        ros::master::getTopics(master_topics);

        int repeated_sensor = 0;
        std::string topic;
        std::vector <std::string> lidar_topics;
        for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
          const ros::master::TopicInfo& info = *it;
          if (info.datatype == "sensor_msgs/PointCloud2") {
            topic = info.name;
            topic = topic.substr(1);
            std::size_t pos = topic.find('/');
            if (pos != std::string::npos)
            {
              topic = topic.substr(0, pos);
            }

            // Avoid repeated topics
            if (lidar_topics.size()!=0){
              for (size_t i = 0; i < lidar_topics.size(); i++) {
                if(lidar_topics[i]==topic){
                  repeated_sensor = repeated_sensor + 1;
                }
              }
            }
            if (repeated_sensor == 0) {
              lidar_topics.push_back(topic);
              ui_->collectDataSensorsComboBox->addItem(QString::fromUtf8(topic.c_str()));
            }
          }
        }
      } catch(...) {
        return;
      }
    } //  function getLidarSensorPosition()
}  //namespace atom_rviz_plugin