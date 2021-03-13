#include <new_display/imu_display.h>
#include <pluginlib/class_list_macros.h>
#include <rviz/display.h>

#include "ui_calibration_panel.h"

namespace new_display
{

DisplayTemplate::DisplayTemplate(QWidget* parent)
    : Display(parent), ui_(new Ui::CalibUI())
{
}

DisplayTemplate::~DisplayTemplate() = default;

void DisplayTemplate::onInitialize()
{
    connect(ui_->pushButton, SIGNAL(clicked()), this, SLOT(buttonClicked()));

    parentWidget()->setVisible(true);
}

void DisplayTemplate::buttonClicked()
    {
        ROS_INFO("Button pushed");
    }

}

PLUGINLIB_EXPORT_CLASS(new_display::DisplayTemplate, rviz::Display)
