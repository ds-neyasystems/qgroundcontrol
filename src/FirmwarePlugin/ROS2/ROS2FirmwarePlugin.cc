#include "ROS2FirmwarePlugin.h";
#include <iostream>

ROS2FirmwarePlugin::ROS2FirmwarePlugin()
{
}

ROS2FirmwarePlugin::~ROS2FirmwarePlugin()
{
}

bool ROS2FirmwarePlugin::isCapable(const Vehicle *vehicle, FirmwareCapabilities capabilities)
{
	int ros2_capabilities;

	ros2_capabilities |= SetFlightModeCapability;
	ros2_capabilities |= PauseVehicleCapability;
	ros2_capabilities |= GuidedModeCapability;
	ros2_capabilities |= TakeoffVehicleCapability;
	ros2_capabilities |= OrbitModeCapability;

	return ( ros2_capabilities & capabilities ) == capabilities;
}

void ROS2FirmwarePlugin::guidedModeRTL (Vehicle* vehicle)
{
	FirmwarePlugin::guidedModeRTL(vehicle);
}

void ROS2FirmwarePlugin::guidedModeLand (Vehicle* vehicle)
{
	FirmwarePlugin::guidedModeLand(vehicle);
}

void ROS2FirmwarePlugin::guidedModeTakeoff (Vehicle* vehicle, double takeoffAltRel)
{
}

void ROS2FirmwarePlugin::guidedModeGotoLocation(Vehicle* vehicle, const QGeoCoordinate& gotoCoord)
{
	vehicle->sendROS2GlobalWaypointCommand( gotoCoord.latitude(), gotoCoord.longitude(), gotoCoord.altitude() );
}

void ROS2FirmwarePlugin::guidedModeChangeAltitude (Vehicle* vehicle, double altitudeRel)
{
	FirmwarePlugin::guidedModeChangeAltitude(vehicle, altitudeRel);
}

QList<MAV_CMD> ROS2FirmwarePlugin::supportedMissionCommands(void)
{
    QList<MAV_CMD> list;

    list << MAV_CMD_NAV_WAYPOINT
         << MAV_CMD_NAV_LOITER_UNLIM << MAV_CMD_NAV_LOITER_TIME << MAV_CMD_NAV_LOITER_TO_ALT
         << MAV_CMD_NAV_LAND << MAV_CMD_NAV_TAKEOFF << MAV_CMD_NAV_RETURN_TO_LAUNCH
         << MAV_CMD_DO_JUMP
         << MAV_CMD_DO_VTOL_TRANSITION << MAV_CMD_NAV_VTOL_TAKEOFF << MAV_CMD_NAV_VTOL_LAND
         << MAV_CMD_DO_DIGICAM_CONTROL
         << MAV_CMD_DO_SET_CAM_TRIGG_DIST
         << MAV_CMD_DO_SET_SERVO
         << MAV_CMD_DO_CHANGE_SPEED
         << MAV_CMD_DO_LAND_START
         << MAV_CMD_DO_SET_ROI_LOCATION << MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET << MAV_CMD_DO_SET_ROI_NONE
         << MAV_CMD_DO_MOUNT_CONFIGURE
         << MAV_CMD_DO_MOUNT_CONTROL
         << MAV_CMD_SET_CAMERA_MODE
         << MAV_CMD_IMAGE_START_CAPTURE << MAV_CMD_IMAGE_STOP_CAPTURE << MAV_CMD_VIDEO_START_CAPTURE << MAV_CMD_VIDEO_STOP_CAPTURE
         << MAV_CMD_NAV_DELAY
         << MAV_CMD_CONDITION_YAW;

    return list;
}
