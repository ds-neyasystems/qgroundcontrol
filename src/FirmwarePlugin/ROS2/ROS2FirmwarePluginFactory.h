#pragma once
#include "FirmwarePlugin.h"

class ROS2FirmwarePlugin;

class ROS2FirmwarePluginFactory : public FirmwarePluginFactory
{
    Q_OBJECT

public:
    ROS2FirmwarePluginFactory(void);

    QList<MAV_AUTOPILOT>    supportedFirmwareTypes      (void) const final;
    FirmwarePlugin*         firmwarePluginForAutopilot  (MAV_AUTOPILOT autopilotType, MAV_TYPE vehicleType) final;

private:
    ROS2FirmwarePlugin*  _pluginInstance;
};
