#pragma once

#include "FirmwarePlugin.h"

class ROS2FirmwarePlugin : public FirmwarePlugin
{
public:
	ROS2FirmwarePlugin();
	~ROS2FirmwarePlugin();

	bool isCapable(const Vehicle *vehicle, FirmwareCapabilities capabilities);

	void guidedModeRTL (Vehicle* vehicle) override;
    void guidedModeLand (Vehicle* vehicle) override;
    void guidedModeTakeoff (Vehicle* vehicle, double takeoffAltRel) override;
    void guidedModeGotoLocation (Vehicle* vehicle, const QGeoCoordinate& gotoCoord) override;
    void guidedModeChangeAltitude (Vehicle* vehicle, double altitudeRel) override;

	QList<MAV_CMD> supportedMissionCommands(void);

protected:

};
