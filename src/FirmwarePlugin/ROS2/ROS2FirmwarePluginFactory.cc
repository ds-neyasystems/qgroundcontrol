#include "ROS2FirmwarePluginFactory.h"
#include "ROS2/ROS2FirmwarePlugin.h"

ROS2FirmwarePluginFactory ROS2FirmwarePluginFactory;

ROS2FirmwarePluginFactory::ROS2FirmwarePluginFactory(void) :
	_pluginInstance(NULL)
{
}

QList<MAV_AUTOPILOT> ROS2FirmwarePluginFactory::supportedFirmwareTypes(void) const
{
	QList<MAV_AUTOPILOT> list;

	list.append(MAV_AUTOPILOT_RESERVED);
	return list;
}

FirmwarePlugin* ROS2FirmwarePluginFactory::firmwarePluginForAutopilot(MAV_AUTOPILOT autopilotType, MAV_TYPE vehicleType)
{
	if( !_pluginInstance )
		_pluginInstance = new ROS2FirmwarePlugin();
	return _pluginInstance;
}
