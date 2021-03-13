#pragma once

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif

#include <rviz/display.h>

namespace Ui {
    class CalibUI;
}

namespace new_display
{

class DisplayTemplate : public rviz::Display
{
    Q_OBJECT
public:
    DisplayTemplate(QWidget* parent = nullptr);

    ~DisplayTemplate() override;

    void onInitialize() override;
    void onEnable();
    void onDisable();

private Q_SLOTS:
    void buttonClicked();

protected:
    Ui::CalibUI* ui_;

};

}


