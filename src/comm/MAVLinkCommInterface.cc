#include "MAVLinkCommInterface.h"
#include "CommManager.h"
#include "SettingsManager.h"
#include "QGCApplication.h"
#include "MultiVehicleManager.h"


/* ******************** *
 * MAVLinkCommInterface *
 * ******************** */
MAVLinkCommInterface::MAVLinkCommInterface(CommInterfaceConfiguration::SharedPointer config, bool isPX4Flow) :
	CommInterface( config, isPX4Flow ),
	_mavlinkChannelSet(false)																											   
{
	connect( this, &MAVLinkCommInterface::protocolStatusMessage, qgcApp(), &QGCApplication::criticalMessageBoxOnMainThread);
	connect( this, &CommInterface::vehicleHeartbeatInfo, qgcApp()->toolbox()->multiVehicleManager(), &MultiVehicleManager::vehicleHeartbeatInfo);

	int channel = MAVLinkProtocol::getInstance()->reserveMAVLinkChannel();
	if( channel != 0 )
		_setMavlinkChannel(channel);
}

MAVLinkCommInterface::~MAVLinkCommInterface()
{
	if( _mavlinkChannelSet )
		MAVLinkProtocol::getInstance()->freeMAVLinkChannel(_mavlinkChannel);
}

uint8_t MAVLinkCommInterface::mavlinkChannel() const
{
    if (!_mavlinkChannelSet) {
        qWarning() << "Call to CommInterface::mavlinkChannel with _mavlinkChannelSet == false";
    }
    return _mavlinkChannel;
}

void MAVLinkCommInterface::resetDecodedFirstMAVLinkPacket()
{
	_decodedFirstMavlinkPacket = false;
}

void MAVLinkCommInterface::resetMetaData()
{
	MAVLinkProtocol::getInstance()->resetMetadataForInterface(this);
}

void MAVLinkCommInterface::_parseBytes( QByteArray bytes )
{
    static int  nonmavlinkCount = 0;
    static bool checkedUserNonMavlink = false;
    static bool warnedUserNonMavlink  = false;	

	if( !_mavlinkChannelSet )
		qWarning() << "MAVLinkCommInterface::_parseBytes - mavlink channel not set";
	
    for (int position = 0; position < bytes.size(); position++)
	{
        if (mavlink_parse_char(_mavlinkChannel, static_cast<uint8_t>(bytes[position]), &_message, &_status))
		{
            // Got a valid message
            if (!_decodedFirstMavlinkPacket)
			{
                _decodedFirstMavlinkPacket = true;
                mavlink_status_t* mavlink_status = mavlink_get_channel_status(_mavlinkChannel);
                if (!(mavlink_status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) && (mavlink_status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1))
				{
                    qDebug() << "Switching outbound to mavlink 2.0 due to incoming mavlink 2.0 packet:" << mavlink_status << _mavlinkChannel << mavlink_status->flags;
                    mavlink_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
                    // Set all links to v2
					MAVLinkProtocol::getInstance()->setCurrentVersion(200);
                }
            }

            //-----------------------------------------------------------------
            // MAVLink Status
            uint8_t last_seq = MAVLinkProtocol::getInstance()->_lastIndex[_message.sysid][_message.compid];
            uint8_t expected_seq = last_seq + 1;
			
            // Increase receive counter
            MAVLinkProtocol::getInstance()->_totalReceiveCounter[_mavlinkChannel]++;
			
            // Determine what the next expected sequence number is, accounting for
            // never having seen a message for this system/component pair.
            if(MAVLinkProtocol::getInstance()->_firstMessage[_message.sysid][_message.compid])
			{
                MAVLinkProtocol::getInstance()->_firstMessage[_message.sysid][_message.compid] = 0;
                last_seq     = _message.seq;
                expected_seq = _message.seq;
            }
            // And if we didn't encounter that sequence number, record the error
            if (_message.seq != expected_seq)
            {
                int lost_messages = 0;
                //-- Account for overflow during packet loss
                if(_message.seq < expected_seq)
                    lost_messages = (_message.seq + 255) - expected_seq;
				else
                    lost_messages = _message.seq - expected_seq;

                // Log how many were lost
                MAVLinkProtocol::getInstance()->_totalLossCounter[_mavlinkChannel] += static_cast<uint64_t>(lost_messages);
            }

            // And update the last sequence number for this system/component pair
            MAVLinkProtocol::getInstance()->_lastIndex[_message.sysid][_message.compid] = _message.seq;
			
            // Calculate new loss ratio
            uint64_t total_sent = MAVLinkProtocol::getInstance()->_totalReceiveCounter[_mavlinkChannel] + MAVLinkProtocol::getInstance()->_totalLossCounter[_mavlinkChannel];
            float receive_loss_percent = static_cast<float>(static_cast<double>(MAVLinkProtocol::getInstance()->_totalLossCounter[_mavlinkChannel]) / static_cast<double>(total_sent));
            receive_loss_percent *= 100.0f;
            receive_loss_percent = (receive_loss_percent * 0.5f) + (MAVLinkProtocol::getInstance()->_runningLossPercent[_mavlinkChannel] * 0.5f);
            MAVLinkProtocol::getInstance()->_runningLossPercent[_mavlinkChannel] = receive_loss_percent;

            //-----------------------------------------------------------------
            // Log data
			MAVLinkProtocol::getInstance()->logMessage(&_message);

			//Special heartbeat signal to add vehicle to vehicle manager
            if (_message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
			{
                MAVLinkProtocol::getInstance()->_startLogging();
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&_message, &heartbeat);
                emit vehicleHeartbeatInfo(this, _message.sysid, _message.compid, heartbeat.autopilot, heartbeat.type);
            }

            if (_message.msgid == MAVLINK_MSG_ID_HIGH_LATENCY2)
			{
                MAVLinkProtocol::getInstance()->_startLogging();
                mavlink_high_latency2_t highLatency2;
                mavlink_msg_high_latency2_decode(&_message, &highLatency2);
                emit vehicleHeartbeatInfo(this, _message.sysid, _message.compid, highLatency2.autopilot, highLatency2.type);
            }

            // Detect if we are talking to an old radio not supporting v2
            mavlink_status_t* mavlink_status = mavlink_get_channel_status(_mavlinkChannel);
            if (_message.msgid == MAVLINK_MSG_ID_RADIO_STATUS && MAVLinkProtocol::getInstance()->_radioVersionMismatchCount != -1)
			{
                if ((mavlink_status->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)
                && !(mavlink_status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1))
				{
                    MAVLinkProtocol::getInstance()->_radioVersionMismatchCount++;
                }
            }

            if (MAVLinkProtocol::getInstance()->_radioVersionMismatchCount == 5)
			{
                // Warn the user if the radio continues to send v1 while the link uses v2
                emit protocolStatusMessage(tr("MAVLink Protocol"), tr("Detected radio still using MAVLink v1.0 on a link with MAVLink v2.0 enabled. Please upgrade the radio firmware."));
                // Set to flag warning already shown
                MAVLinkProtocol::getInstance()->_radioVersionMismatchCount = -1;
                // Flick link back to v1
                qDebug() << "Switching outbound to mavlink 1.0 due to incoming mavlink 1.0 packet:" << mavlink_status << _mavlinkChannel << mavlink_status->flags;
                mavlink_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            }

            // Update MAVLink status on every 32th packet
            if ((MAVLinkProtocol::getInstance()->_totalReceiveCounter[_mavlinkChannel] & 0x1F) == 0)
			{
                MAVLinkProtocol::getInstance()->statusMessage(_message.sysid, total_sent, MAVLinkProtocol::getInstance()->_totalReceiveCounter[_mavlinkChannel], MAVLinkProtocol::getInstance()->_totalLossCounter[_mavlinkChannel], receive_loss_percent);
            }

			//Handle completed message
			_handleMessage(_message);
			
            // Reset message parsing
            memset(&_status,  0, sizeof(_status));
            memset(&_message, 0, sizeof(_message));
        }
		else if (!_decodedFirstMavlinkPacket)
		{
            // No formed message yet
            nonmavlinkCount++;
            if (nonmavlinkCount > 1000 && !warnedUserNonMavlink)
			{
                // 1000 bytes with no mavlink message. Are we connected to a mavlink capable device?
                if (!checkedUserNonMavlink)
				{
                    requestReset();
                    checkedUserNonMavlink = true;
                }
				else
				{
                    warnedUserNonMavlink = true;
                    // Disconnect the link since it's some other device and
                    // QGC clinging on to it and feeding it data might have unintended
                    // side effects (e.g. if its a modem)
                    qDebug() << "disconnected link" << getName() << "as it contained no MAVLink data";
                    QMetaObject::invokeMethod(CommManager::getCommManager(), "disconnectLink", Q_ARG( CommInterface*, this ) );
                    return;
                }
            }
        }
    }
}

void MAVLinkCommInterface::_handleMessage( mavlink_message_t& message )
{
	Vehicle* vehicle;
	
	//Get vehicle from multi vehicle manager based on sysid of message
	vehicle = qgcApp()->toolbox()->multiVehicleManager()->getVehicleById( message.sysid );
	
	//Start/reset activity timer for this vehicle/link combination
	startActivityTimer(message.sysid);
	
	//If the vehicle has not yet been initialized, ignore it
	if( vehicle == NULL )
		return;

	//Give the vehicle the opportunity to add this interface to its list of links
	vehicle->checkLink(this);
	
	//Adjust mavlink message based on vehicle firmware
	if( vehicle != NULL && vehicle->firmwarePlugin() != NULL )
		vehicle->firmwarePlugin()->adjustIncomingMavlinkMessage(vehicle, &message);

	//Unpack message and us Qt signals to send relevent data to Vehicle class
	switch( message.msgid )
	{
    case MAVLINK_MSG_ID_ADSB_VEHICLE:
	{
		mavlink_adsb_vehicle_t adsb_vehicle;
		mavlink_msg_adsb_vehicle_decode(&message, &adsb_vehicle);
		emit receivedADSBVehicle( vehicle,
								  adsb_vehicle.ICAO_address,
			                      adsb_vehicle.lat,
			                      adsb_vehicle.lon,
								  adsb_vehicle.altitude,
								  adsb_vehicle.heading,
								  adsb_vehicle.flags,
								  adsb_vehicle.callsign );
		break;
	}
    case MAVLINK_MSG_ID_ALTITUDE:
	{
		mavlink_altitude_t altitude;
		mavlink_msg_altitude_decode(&message, &altitude);
		emit receivedAltitude( vehicle, altitude.altitude_amsl, altitude.altitude_relative );
		break;
	}
    case MAVLINK_MSG_ID_ATTITUDE:
	{
	    mavlink_attitude_t attitude;
		float roll, pitch, yaw;
		
		mavlink_msg_attitude_decode(&message, &attitude);
		
		roll = qRadiansToDegrees(attitude.roll);
		pitch = qRadiansToDegrees(attitude.pitch);
		yaw = qRadiansToDegrees(attitude.yaw);
		
		emit receivedAttitude( vehicle, roll, pitch, yaw );
		break;
	}
    case MAVLINK_MSG_ID_ATTITUDE_QUATERNION:
	{
		mavlink_attitude_quaternion_t attitude_quaternion;
		float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
		
		mavlink_msg_attitude_quaternion_decode(&message, &attitude_quaternion);
		float q[] = { attitude_quaternion.q1, attitude_quaternion.q2, attitude_quaternion.q3, attitude_quaternion.q4 };
		mavlink_quaternion_to_euler(q, &roll, &pitch, &yaw);
		
		roll  = qRadiansToDegrees(roll);
		pitch = qRadiansToDegrees(pitch);
		yaw   = qRadiansToDegrees(yaw);
		
		roll_rate  = qRadiansToDegrees(attitude_quaternion.rollspeed);
		pitch_rate = qRadiansToDegrees(attitude_quaternion.pitchspeed);
		yaw_rate   = qRadiansToDegrees(attitude_quaternion.yawspeed);
		
		emit receivedAttitudeQuaternion(vehicle, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate);
		break;
	}
    case MAVLINK_MSG_ID_ATTITUDE_TARGET:
	{
		mavlink_attitude_target_t attitude_target;
		float roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate;
		
		mavlink_msg_attitude_target_decode(&message, &attitude_target);
		mavlink_quaternion_to_euler(attitude_target.q, &roll, &pitch, &yaw);

		mavlink_quaternion_to_euler(attitude_target.q, &roll, &pitch, &yaw);
		
		roll_rate  = qRadiansToDegrees(attitude_target.body_roll_rate);
		pitch_rate = qRadiansToDegrees(attitude_target.body_pitch_rate);
		yaw_rate   = qRadiansToDegrees(attitude_target.body_yaw_rate);
		
		emit receivedAttitudeTarget( vehicle, roll, pitch, yaw, roll_rate, pitch_rate, yaw_rate );
		break;
	}
    case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
	{
		mavlink_autopilot_version_t autopilot_version;
		FIRMWARE_VERSION_TYPE firmware_version;
		int version_type, major_version, minor_version, patch_version, major_custom_version, minor_custom_version, patch_custom_version;		
		bool capability_mission, capability_command, capability_mavlink2, capability_fence, capability_rally;
		uint64_t flight_custom_version;		
		
		mavlink_msg_autopilot_version_decode(&message, &autopilot_version);

		memcpy( &flight_custom_version, &*autopilot_version.flight_custom_version, 8 * sizeof(uint8_t) );		
		
		emit receivedAutopilotVersion( vehicle,
									   autopilot_version.uid,
									   autopilot_version.flight_sw_version,
									   flight_custom_version,//autopilot_version.flight_custom_version,
									   autopilot_version.capabilities );
/*		
		version_type = (FIRMWARE_VERSION_TYPE)((autopilot_version.flight_sw_version >> (8*0)) & 0xFF);

		major_version = (autopilot_version.flight_sw_version >> (8*3)) & 0xFF;
        minor_version = (autopilot_version.flight_sw_version >> (8*2)) & 0xFF;
        patch_version = (autopilot_version.flight_sw_version >> (8*1)) & 0xFF;
		
		major_custom_version = autopilot_version.flight_custom_version[2];
        minor_custom_version = autopilot_version.flight_custom_version[1];
        patch_custom_version = autopilot_version.flight_custom_version[0];
		
		capability_mission  = autopilot_version.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_INT;
		capability_command  = autopilot_version.capabilities & MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		capability_mavlink2 = autopilot_version.capabilities & MAV_PROTOCOL_CAPABILITY_MAVLINK2;
		capability_fence    = autopilot_version.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
		capability_rally    = autopilot_version.capabilities & MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
		
		emit receivedAutopilotVersion( vehicle,
									   autopilot_version.uid,
									   version_type,
									   major_version,
									   minor_version,
									   patch_version,
									   major_custom_version,
									   minor_custom_version,
									   patch_custom_version,
									   capability_mission,
									   capability_command,
									   capability_mavlink2,
									   capability_fence,
									   capability_rally );
 */
		break;
	}
    case MAVLINK_MSG_ID_BATTERY_STATUS:
	{
		mavlink_battery_status_t battery_status;
		double temperature;
		int cell_count, charge_state;
		
		mavlink_msg_battery_status_decode(&message, &battery_status);
/*
		if( battery_status.temperature == INT16_MAX )
			temperature = -1;
		else
			temperature = (double)battery_status.temperature / 100.0;

		cell_count = 0;
		for(int i = 0; i < 10; i++)
			if( battery_status.voltages[i] != UINT16_MAX )
				cell_count++;
		if( cell_count == 0 )
			cell_count = -1;
*/
		if( battery_status.charge_state >= MAV_BATTERY_CHARGE_STATE_ENUM_END )
			charge_state = 0;
		else
			charge_state = battery_status.charge_state;
		
//		emit receivedBatteryStatus( vehicle, battery_status.id, temperature, battery_status.current_consumed, cell_count, battery_status.time_remaining, charge_state );
		emit receivedBatteryChargeState( vehicle, battery_status.id, charge_state );
		emit receivedBatteryCurrentConsumed( vehicle, battery_status.id, battery_status.current_consumed );
		break;
	}
	case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
	{
        mavlink_camera_capture_status_t capture_status;
        mavlink_msg_camera_capture_status_decode(&message, &capture_status);
		emit receivedCameraCaptureStatus( vehicle, capture_status.available_capacity, capture_status.image_status, capture_status.video_status );
		break;
	}
	case MAVLINK_MSG_ID_CAMERA_FEEDBACK:
	{
		mavlink_camera_feedback_t feedback;
		double latitude, longitude;
		
		mavlink_msg_camera_feedback_decode(&message, &feedback);

		latitude  = (double)feedback.lat / qPow(10.0, 7.0);
		longitude = (double)feedback.lng / qPow(10.0, 7.0);
		
		emit receivedCameraImageCaptured( vehicle, latitude, longitude, feedback.alt_msl );
		break;
	}
    case MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED:
	{
		mavlink_camera_image_captured_t feedback;
		double latitude, longitude;
		mavlink_msg_camera_image_captured_decode(&message, &feedback);

		if( !feedback.capture_result == 1 )
			break;
		
		latitude  = (double)feedback.lat / qPow(10.0, 7.0);
		longitude = (double)feedback.lon / qPow(10.0, 7.0);
		
		emit receivedCameraImageCaptured( vehicle, latitude, longitude, feedback.alt );
		break;
	}
	case MAVLINK_MSG_ID_CAMERA_INFORMATION:
	{
		//#TODO
//		mavlink_camera_information_t info;
//      mavlink_msg_camera_information_decode(&message, &info);
//		emit receivedCameraInfo(vehicle);
		break;
	}
	case MAVLINK_MSG_ID_CAMERA_SETTINGS:
	{
		mavlink_camera_settings_t settings;
        mavlink_msg_camera_settings_decode(&message, &settings);
		emit receivedCameraSettings( vehicle, settings.mode_id, settings.zoomLevel, settings.focusLevel );
		break;
	}
	case MAVLINK_MSG_ID_COMMAND_ACK:
	{
		mavlink_command_ack_t ack;
		mavlink_msg_command_ack_decode(&message, &ack);
		emit receivedCommandAck( vehicle, message.compid, ack.command, ack.result );
		break;
	}		
	case MAVLINK_MSG_ID_COMMAND_LONG:
	{
		mavlink_command_long_t cmd;
		mavlink_msg_command_long_decode(&message, &cmd);

		if( cmd.command == MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN )
			emit receivedPreflightRebootShutdown( vehicle, cmd.param6 );
		
		break;
	}
	case MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE:
	{
		mavlink_data_transmission_handshake_t handshake;
		mavlink_msg_data_transmission_handshake_decode(&message, &handshake);
		emit receivedDataTransmissionHandshake( vehicle,
												handshake.size,
												handshake.width,
												handshake.height,
												handshake.packets,
												handshake.type,
												handshake.payload,
												handshake.jpg_quality );
		break;
	}
    case MAVLINK_MSG_ID_DISTANCE_SENSOR:
	{
		mavlink_distance_sensor_t distance_sensor;
		mavlink_msg_distance_sensor_decode(&message, &distance_sensor);
		emit receivedDistanceSensor( vehicle, distance_sensor.id, distance_sensor.current_distance, distance_sensor.orientation );
		break;
	}
	case MAVLINK_MSG_ID_ENCAPSULATED_DATA:
	{
		//#TODO				
//		mavlink_encapsulated_data_t img;
//		mavlink_msg_encapsulated_data_decode(&message, &img);
//		emit receivedEncapsulatedData(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_ESTIMATOR_STATUS:
	{
	    mavlink_estimator_status_t estimator_status;
		mavlink_msg_estimator_status_decode(&message, &estimator_status);
		emit receivedEstimatorStatus( vehicle,
									  estimator_status.vel_ratio,
									  estimator_status.pos_horiz_ratio,
									  estimator_status.pos_vert_ratio,
									  estimator_status.mag_ratio,
									  estimator_status.hagl_ratio,
									  estimator_status.tas_ratio,
									  estimator_status.pos_horiz_accuracy,
									  estimator_status.pos_vert_accuracy,
									  !!(estimator_status.flags & ESTIMATOR_ATTITUDE),
									  !!(estimator_status.flags & ESTIMATOR_VELOCITY_HORIZ),
									  !!(estimator_status.flags & ESTIMATOR_VELOCITY_VERT),
									  !!(estimator_status.flags & ESTIMATOR_POS_HORIZ_REL),
									  !!(estimator_status.flags & ESTIMATOR_POS_HORIZ_ABS),
									  !!(estimator_status.flags & ESTIMATOR_POS_VERT_ABS),
									  !!(estimator_status.flags & ESTIMATOR_POS_VERT_AGL),
									  !!(estimator_status.flags & ESTIMATOR_CONST_POS_MODE),
									  !!(estimator_status.flags & ESTIMATOR_PRED_POS_HORIZ_REL),
									  !!(estimator_status.flags & ESTIMATOR_PRED_POS_HORIZ_ABS),
									  !!(estimator_status.flags & ESTIMATOR_GPS_GLITCH),
									  !!(estimator_status.flags & ESTIMATOR_ACCEL_ERROR) );
		break;
	}
    case MAVLINK_MSG_ID_EXTENDED_SYS_STATE:
	{
	    mavlink_extended_sys_state_t extended_state;
		mavlink_msg_extended_sys_state_decode(&message, &extended_state);
		emit receivedExtendedSysState( vehicle, extended_state.vtol_state, extended_state.landed_state );
		break;
	}
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
	{
	    mavlink_global_position_int_t global_position_int;
		mavlink_msg_global_position_int_decode(&message, &global_position_int);

		emit receivedAltitude( vehicle, global_position_int.alt / 1000.0, global_position_int.relative_alt / 1000.0 );

		//If latitude and longitude are 0, this message is being used to only convery altitude
		if( global_position_int.lat == 0 && global_position_int.lon == 0 )
			break;

		emit receivedGPS( vehicle,
						  global_position_int.lat / (double)1e7,
						  global_position_int.lon / (double)1e7,
						  global_position_int.alt / 1000.0	);
		emit receivedVelocity( vehicle,
							   global_position_int.vx / 100.0,
							   global_position_int.vy / 100.0,
							   global_position_int.vz / 100.0 );
		break;
	}
    case MAVLINK_MSG_ID_GPS_RAW_INT:
	{
		mavlink_gps_raw_int_t gps_raw_int;
		mavlink_msg_gps_raw_int_decode(&message, &gps_raw_int);
		emit receivedAltitude( vehicle, gps_raw_int.alt / 1000.0, NAN );
		if (gps_raw_int.fix_type >= GPS_FIX_TYPE_3D_FIX)
		{
			emit receivedGPS( vehicle,
							  gps_raw_int.lat / 1e7,
							  gps_raw_int.lon / 1e7,
							  gps_raw_int.alt / 1000.0 );
		}
		emit receivedGPSData( vehicle,
							  gps_raw_int.lat * 1e-7,
							  gps_raw_int.lon * 1e-7,
							  gps_raw_int.eph,
							  gps_raw_int.epv,
							  gps_raw_int.cog,
							  gps_raw_int.fix_type,
							  gps_raw_int.satellites_visible );
		break;
	}
    case MAVLINK_MSG_ID_HEARTBEAT:
	{
		mavlink_heartbeat_t heartbeat;		
		mavlink_msg_heartbeat_decode(&message, &heartbeat);

		emit receivedHeartbeat( vehicle, heartbeat.base_mode, heartbeat.custom_mode );
//			emit receivedHeartbeat( vehicle, this, message.sysid, message.compid, heartbeat.autopilot, heartbeat.type );
		break;
	}			
    case MAVLINK_MSG_ID_HIGH_LATENCY2:
	{
		mavlink_high_latency2_t high_latency;
		mavlink_msg_high_latency2_decode(&message, &high_latency);

		emit receivedGPS( vehicle,
						  high_latency.latitude  / (double)1e7,
						  high_latency.longitude / (double)1e7,
						  high_latency.altitude );
						
		emit receivedAltitude( vehicle, high_latency.altitude / 1000.0, NAN );

		emit receivedBatteryRemaining( vehicle, 0, high_latency.battery );
		emit receivedHeartbeat( vehicle, MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, high_latency.custom_mode );

//			emit receivedHeartbeat( vehicle, this, message.sysid, message.compid, high_latency.autopilot, high_latency.type );
/*
  //#TODO
 uint16_t failure_flags; //<  Bitmap of failure flags.
 uint8_t heading;        //< [deg/2] Heading
 uint8_t airspeed;       //< [m/s*5] Airspeed
 uint8_t groundspeed;    //< [m/s*5] Groundspeed
 uint8_t windspeed;      //< [m/s*5] Windspeed
 uint8_t wind_heading;   //< [deg/2] Wind heading
 uint8_t eph;            //< [dm] Maximum error horizontal position since last message
 uint8_t epv;            //< [dm] Maximum error vertical position since last message
 int8_t temperature_air; //< [degC] Air temperature from airspeed sensor
 int8_t climb_rate;      //< [dm/s] Maximum climb rate magnitude since last message
*/
		break;
	}
    case MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS:
	{
		mavlink_hil_actuator_controls_t hil;
		mavlink_msg_hil_actuator_controls_decode(&message, &hil);
//		emit receivedHILActuatorControls(vehicle);
		/*
		emit hilActuatorControlsChanged( hil.time_usec,
										 hil.flags,
										 hil.controls[0],
										 hil.controls[1],
										 hil.controls[2],
										 hil.controls[3],
										 hil.controls[4],
										 hil.controls[5],
										 hil.controls[6],
										 hil.controls[7],
										 hil.controls[8],
										 hil.controls[9],
										 hil.controls[10],
										 hil.controls[11],
										 hil.controls[12],
										 hil.controls[13],
										 hil.controls[14],
										 hil.controls[15],
										 hil.mode);
		*/
		break;
	}
    case MAVLINK_MSG_ID_HOME_POSITION:
	{
		mavlink_home_position_t home_position;
		mavlink_msg_home_position_decode(&message, &home_position);
		emit receivedHomePosition( vehicle,
								   home_position.latitude / (double)1e7,
								   home_position.longitude / (double)1e7,
								   home_position.altitude / 1000.0 );
		break;
	}
	case MAVLINK_MSG_ID_LOG_DATA:
	{
		//#TODO
//		mavlink_log_data_t log;
//		mavlink_msg_log_data_decode(&message, &log);
//		emit receivedLogData(vehicle);		
		break;
	}
	case MAVLINK_MSG_ID_LOG_ENTRY:
	{
		//#TODO
//		mavlink_log_entry_t log;
//		mavlink_msg_log_entry_decode(&message, &log);
//		emit receivedLogEntry(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_LOGGING_DATA:
	{
		//#TODO
//		mavlink_logging_data_t log;
//		mavlink_msg_logging_data_decode(&message, &log);
//		emit receivedLogData(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_LOGGING_DATA_ACKED:
	{
		//#TODO
//		mavlink_logging_data_acked_t log;
//		mavlink_msg_logging_data_acked_decode(&message, &log);
//		emit receivedLogDataAck();
		break;
	}
	case MAVLINK_MSG_ID_MAG_CAL_PROGRESS:
	{
		//#TODO
//		mavlink_mag_cal_progress_t progress;
//      mavlink_msg_mag_cal_progress_decode(&message, &progress);
//		emit receivedMagneticCalibrationProgress();
        break;
	}
    case MAVLINK_MSG_ID_MAG_CAL_REPORT:
	{
		mavlink_mag_cal_report_t report;
        mavlink_msg_mag_cal_report_decode(&message, &report);
		emit receivedMagneticCalibrationReport( vehicle,
												report.compass_id,
												report.cal_status,
												report.fitness );
		break;
	}
    case MAVLINK_MSG_ID_MISSION_ACK:
	{
		mavlink_mission_ack_t mission_ack;
		mavlink_msg_mission_ack_decode(&message, &mission_ack);
		emit receivedMissionAck( vehicle, mission_ack.type, mission_ack.mission_type );
		break;
	}
    case MAVLINK_MSG_ID_MISSION_COUNT:
	{
		mavlink_mission_count_t mission_count;
		mavlink_msg_mission_count_decode(&message, &mission_count);
		emit receivedMissionCount( vehicle, mission_count.mission_type, mission_count.count );
		break;
	}
	case MAVLINK_MSG_ID_MISSION_CURRENT:
	{
		mavlink_mission_current_t mission_current;
		mavlink_msg_mission_current_decode(&message, &mission_current);
		emit receivedMissionCurrent( vehicle, mission_current.seq );
		break;
	}
    case MAVLINK_MSG_ID_MISSION_ITEM:
	{
		mavlink_mission_item_t mission_item;
        mavlink_msg_mission_item_decode(&message, &mission_item);
		MissionItem item( mission_item.seq,
						  (MAV_CMD)mission_item.command,
						  (MAV_FRAME)mission_item.frame,
						  mission_item.param1,
						  mission_item.param2,
						  mission_item.param3,
						  mission_item.param4,
						  mission_item.x,
						  mission_item.y,
						  mission_item.z,
						  mission_item.autocontinue,
						  mission_item.current,
						  NULL );
		emit receivedMissionItem( vehicle, item );
		break;
	}
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
	{
		mavlink_mission_item_int_t mission_item;
        mavlink_msg_mission_item_int_decode(&message, &mission_item);
		MissionItem item( mission_item.seq,
						  (MAV_CMD)mission_item.command,
						  (MAV_FRAME)mission_item.frame,
						  mission_item.param1,
						  mission_item.param2,
						  mission_item.param3,
						  mission_item.param4,
						  mission_item.x / (double)1e7,
						  mission_item.y / (double)1e7,
						  mission_item.z,
						  mission_item.autocontinue,
						  mission_item.current,
						  NULL );
		emit receivedMissionItem( vehicle, item );
		break;
	}
    case MAVLINK_MSG_ID_MISSION_REQUEST:
	{
		mavlink_mission_request_t mission_request;
        mavlink_msg_mission_request_decode(&message, &mission_request);
		emit receivedMissionRequest( vehicle, mission_request.mission_type, mission_request.seq, false );
		break;
	}
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
	{
		mavlink_mission_request_t mission_request;    
		mavlink_msg_mission_request_decode(&message, &mission_request);
		emit receivedMissionRequest( vehicle, mission_request.mission_type, mission_request.seq, true );		
		break;
	}
    case MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS:
	{
		mavlink_orbit_execution_status_t orbit_status;
		mavlink_msg_orbit_execution_status_decode(&message, &orbit_status);
		emit receivedOrbitExecutionStatus( vehicle, orbit_status.x / (double)1e7, orbit_status.y / (double)1e7, orbit_status.radius );
		break;
	}
	case MAVLINK_MSG_ID_PARAM_EXT_ACK:
	{
		//#TODO
//		mavlink_param_ext_ack_t ack;
//      mavlink_msg_param_ext_ack_decode(&message, &ack);
//		emit receivedParamAck(vehicle);
		break;
	}	
	case MAVLINK_MSG_ID_PARAM_EXT_VALUE:
	{
		//#TODO
//		mavlink_param_ext_value_t value;
//		mavlink_msg_param_ext_value_decode(&message, &value);
//		emit receivedParamValue(vehicle);
		break;
	}
	case MAVLINK_MSG_ID_PARAM_VALUE:
	{
		mavlink_param_value_t value;
		mavlink_param_union_t value_union;
		QString param_name;
		QVariant param_value;
		mavlink_msg_param_value_decode(&message, &value);
		value_union.param_float = value.param_value;

		param_name = QString(value.param_id).chopped(MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
		switch( value.param_type )
		{
		case MAV_PARAM_TYPE_REAL32:
            param_value = QVariant(value_union.param_float);
            break;
        case MAV_PARAM_TYPE_UINT8:
            param_value = QVariant(value_union.param_uint8);
            break;
        case MAV_PARAM_TYPE_INT8:
            param_value = QVariant(value_union.param_int8);
            break;
        case MAV_PARAM_TYPE_UINT16:
            param_value = QVariant(value_union.param_uint16);
            break;
        case MAV_PARAM_TYPE_INT16:
            param_value = QVariant(value_union.param_int16);
            break;
        case MAV_PARAM_TYPE_UINT32:
            param_value = QVariant(value_union.param_uint32);
            break;
        case MAV_PARAM_TYPE_INT32:
            param_value = QVariant(value_union.param_int32);
            break;
        //-- Note: These are not handled above:
        //   MAV_PARAM_TYPE_UINT64
        //   MAV_PARAM_TYPE_INT64
        //   MAV_PARAM_TYPE_REAL64
        //   No space in message (the only storage allocation is a "float") and not present in mavlink_param_union_t
        default:
            qCritical() << "INVALID DATA TYPE USED AS PARAMETER VALUE: " << value.param_type;
		}
		
		emit receivedParamValue( vehicle, value.param_count, value.param_index, param_name, param_value );
		break;
	}
    case MAVLINK_MSG_ID_PING:
	{
		mavlink_ping_t ping;
		mavlink_msg_ping_decode(&message, &ping);
		emit receivedPing( vehicle, ping.time_usec, ping.seq );		
		break;
	}
    case MAVLINK_MSG_ID_PROTOCOL_VERSION:
	{
		//#TODO
		mavlink_protocol_version_t protocol_version;
		mavlink_msg_protocol_version_decode(&message, &protocol_version);
//		emit receivedProtocolVersion(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_RADIO_STATUS:
	{
		//#TODO
		mavlink_radio_status_t radio_status;
		mavlink_msg_radio_status_decode(&message, &radio_status);
//		emit receivedRadioStatus(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_RAW_IMU:
	{
		//#TODO
		mavlink_raw_imu_t raw_imu;
		mavlink_msg_raw_imu_decode(&message, &raw_imu);
//		emit receivedRawIMU(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_RC_CHANNELS:
	{
	    mavlink_rc_channels_t channels;
		QList<int> pwm_values;
		
		mavlink_msg_rc_channels_decode(&message, &channels);

		uint16_t* pwm_data[] =
		{
			&channels.chan1_raw,
			&channels.chan2_raw,
			&channels.chan3_raw,
			&channels.chan4_raw,
			&channels.chan5_raw,
			&channels.chan6_raw,
			&channels.chan7_raw,
			&channels.chan8_raw,
			&channels.chan9_raw,
			&channels.chan10_raw,
			&channels.chan11_raw,
			&channels.chan12_raw,
			&channels.chan13_raw,
			&channels.chan14_raw,
			&channels.chan15_raw,
			&channels.chan16_raw,
			&channels.chan17_raw,
			&channels.chan18_raw
		};

		for(int i = 0; i < channels.chancount && i < 18; i++)
		{
			if( *pwm_data[i] == UINT16_MAX )
				pwm_values.append( -1 );
			else
				pwm_values.append( *pwm_data[i] );
		}
	
		emit receivedRCChannels(vehicle, channels.rssi, pwm_values);
		break;
	}
    case MAVLINK_MSG_ID_RC_CHANNELS_RAW:
	{
	    mavlink_rc_channels_raw_t channels;
		QList<int>pwm_values;
		int channel_count = 0;
		
		mavlink_msg_rc_channels_raw_decode(&message, &channels);

		uint16_t* pwm_data[] =
		{
			&channels.chan1_raw,
			&channels.chan2_raw,
			&channels.chan3_raw,
			&channels.chan4_raw,
			&channels.chan5_raw,
			&channels.chan6_raw,
			&channels.chan7_raw,
			&channels.chan8_raw,
		};

		for(int i = 0; i < 8; i++)
			if( *pwm_data[i] != UINT16_MAX )
				channel_count = i + 1;

		for(int i = 0; i < channel_count; i++)
		{
			if( *pwm_data[i] == UINT16_MAX )
				pwm_values.append( -1 );
			else
				pwm_values.append( *pwm_data[i] );
		}
		
		emit receivedRCChannels(vehicle, channels.rssi, pwm_values);
		break;
	}
    case MAVLINK_MSG_ID_SCALED_IMU:
	{
		//unused
		break;
	}
    case MAVLINK_MSG_ID_SCALED_IMU2:
	{
		//#TODO
		mavlink_scaled_imu2_t scaled_imu;
		mavlink_msg_scaled_imu2_decode(&message, &scaled_imu);
//		emit receivedScaledIMU(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_SCALED_IMU3:
	{
		//#TODO
		mavlink_scaled_imu3_t scaled_imu;
		mavlink_msg_scaled_imu3_decode(&message, &scaled_imu);
//		emit receivedScaledIMU(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
	{
		mavlink_scaled_pressure_t pressure;
		mavlink_msg_scaled_pressure_decode(&message, &pressure);
		emit receivedTemperature( vehicle, 1, pressure.temperature / 100.0 );
		break;
	}
    case MAVLINK_MSG_ID_SCALED_PRESSURE2:
	{
	    mavlink_scaled_pressure2_t pressure;
		mavlink_msg_scaled_pressure2_decode(&message, &pressure);
		emit receivedTemperature( vehicle, 2, pressure.temperature / 100.0 );
		break;
	}
    case MAVLINK_MSG_ID_SCALED_PRESSURE3:
	{
	    mavlink_scaled_pressure3_t pressure;
		mavlink_msg_scaled_pressure3_decode(&message, &pressure);
		emit receivedTemperature( vehicle, 3, pressure.temperature / 100.0 );
		break;
	}
    case MAVLINK_MSG_ID_SERIAL_CONTROL:
	{
		//#TODO
		mavlink_serial_control_t serial_control;
        mavlink_msg_serial_control_decode(&message, &serial_control);

//		for console
		break;
	}
    case MAVLINK_MSG_ID_STATUSTEXT:
	{
//    QByteArray b;
//    b.resize(MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1);
//	  mavlink_msg_statustext_get_text(&message, b.data());
//    b[b.length()-1] = '\0';
		char status_text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] = {0};
		mavlink_msg_statustext_get_text(&message, status_text);
//		emit receivedStatusText(vehicle);
		break;
	}
	case MAVLINK_MSG_ID_STORAGE_INFORMATION:
	{
		mavlink_storage_information_t info;
        mavlink_msg_storage_information_decode(&message, &info);
//		emit receivedStorageInfo(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_SYS_STATUS:
	{
		mavlink_sys_status_t system_status;
		mavlink_msg_sys_status_decode(&message, &system_status);

		emit receivedBatteryRemaining(vehicle, 0, system_status.battery_remaining);
		emit receivedBatteryVoltage(vehicle, 0, (double)system_status.voltage_battery / 1000.0, (double)system_status.current_battery / 100.0);

//		if( apmFirmware() && _apmArmingNotRequired() )
//			emit receivedArmed(vehicle, system_status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS);
		
//		emit receivedSysStatus(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_VFR_HUD:
	{
		mavlink_vfr_hud_t vfr_hud;
		mavlink_msg_vfr_hud_decode(&message, &vfr_hud);
//		emit receivedVFRHUD(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_VIBRATION:
	{
		mavlink_vibration_t vibration;
		mavlink_msg_vibration_decode(&message, &vibration);
//		emit receivedVibration(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_WIND:
	{
		mavlink_wind_t wind;
		mavlink_msg_wind_decode(&message, &wind);
//		emit receivedWind(vehicle);
		break;
	}
    case MAVLINK_MSG_ID_WIND_COV:
	{
		mavlink_wind_cov_t wind;
		mavlink_msg_wind_cov_decode(&message, &wind);
//		emit receivedWindCovariance(vehicle);
		break;
	}
	default:
		if( !_unhandledMessageIDList.contains(message.msgid) )
			_unhandledMessageIDList.append(message.msgid);
//		qWarning() << "MAVLinkCommInterface::_handleMessage - unhandled message id(" << message.msgid << ")";
	}
}


void MAVLinkCommInterface::_setMavlinkChannel(uint8_t channel)
{
    if (_mavlinkChannelSet) {
        qWarning() << "Mavlink channel set multiple times";
    }
    _mavlinkChannelSet = true;
    _mavlinkChannel = channel;
}


void MAVLinkCommInterface::slotCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
	mavlink_message_t message;
	mavlink_msg_command_int_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									   MAVLinkProtocol::getInstance()->getComponentId(),
									   _mavlinkChannel,
									   &message,
									   target_system,
									   target_component,
									   frame,
									   command,
									   current,
									   autocontinue,
									   param1,
									   param2,
									   param3,
									   param4,
									   x,
									   y,
									   z );
	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 )
{
		mavlink_message_t message;
		mavlink_msg_command_long_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											MAVLinkProtocol::getInstance()->getComponentId(),
											_mavlinkChannel,
											&message,
											target_system,
											target_component,
											command,
											confirmation,
											param1,
											param2,
											param3,
											param4,
											param5,
											param6,
											param7 );
 	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component )
{
	mavlink_message_t message;

    mavlink_msg_command_ack_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									   MAVLinkProtocol::getInstance()->getComponentId(),
									   _mavlinkChannel,
									   &message,
									   0,    // command
									   1,    // result
									   0,    // progress
									   0,    // result_param2
									   0,    // target_system
									   0);   // target_component
	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotDataTransmissionHandshake( uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality) 
{
	mavlink_message_t message;
	mavlink_msg_data_transmission_handshake_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
													   MAVLinkProtocol::getInstance()->getComponentId(),
													   _mavlinkChannel,
													   &message,
													   type,
													   size,
													   width,
													   height,
													   packets,
													   payload,
													   jpg_quality );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities )
{
	mavlink_message_t message;
	
	mavlink_msg_follow_target_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										 MAVLinkProtocol::getInstance()->getComponentId(),
										 _mavlinkChannel,
										 &message,
										 CommManager::getCommManager()->runtime(),
										 est_capabilities,
										 latitude,
										 longitude,
										 altitude,
										 velocity,
										 acceleration,
										 attitude_q,
										 rates,
										 position_cov,
										 0 ); //custom_state
	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotFileTransfer( uint8_t target_system, QByteArray payload )
{
	mavlink_message_t message;
	mavlink_msg_file_transfer_protocol_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
												  MAVLinkProtocol::getInstance()->getComponentId(),
												  _mavlinkChannel,
												  &message,                   // Mavlink Message to pack into
												  0,                          // Target network
												  target_system,              // Target system
												  0,                          // Target component
												  (uint8_t*)payload.data() ); // Payload
    _sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id )
{
	mavlink_message_t message;
    mavlink_gps_rtcm_data_t mavlink_rtcm_data;
    memset(&mavlink_rtcm_data, 0, sizeof(mavlink_gps_rtcm_data_t));
	
    if (rtcm_data.size() < MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN)
	{
        mavlink_rtcm_data.len = rtcm_data.size();
        mavlink_rtcm_data.flags = (sequence_id & 0x1F) << 3;
        memcpy(&mavlink_rtcm_data.data, rtcm_data.data(), rtcm_data.size());
		mavlink_msg_gps_rtcm_data_encode_chan( MAVLinkProtocol::getInstance()->getSystemId(),
                                               MAVLinkProtocol::getInstance()->getComponentId(),
											   _mavlinkChannel,
											   &message,
											   &mavlink_rtcm_data );
        _sendMAVLinkMessage( message );
    }
	else
	{
        // We need to fragment

        uint8_t fragmentId = 0;         // Fragment id indicates the fragment within a set
        int start = 0;
        while (start < rtcm_data.size())
		{
            int length = std::min(rtcm_data.size() - start, MAVLINK_MSG_GPS_RTCM_DATA_FIELD_DATA_LEN);
            mavlink_rtcm_data.flags = 1;                          // LSB set indicates rtcm_data is fragmented
            mavlink_rtcm_data.flags |= fragmentId++ << 1;         // Next 2 bits are fragment id
            mavlink_rtcm_data.flags |= (sequence_id & 0x1F) << 3; // Next 5 bits are sequence id
            mavlink_rtcm_data.len = length;
            memcpy(&mavlink_rtcm_data.data, rtcm_data.data() + start, length);
			mavlink_msg_gps_rtcm_data_encode_chan( MAVLinkProtocol::getInstance()->getSystemId(),
												   MAVLinkProtocol::getInstance()->getComponentId(),
												   _mavlinkChannel,
												   &message,
												   &mavlink_rtcm_data );
			_sendMAVLinkMessage( message );
			start += length;
        }
    }
}

void MAVLinkCommInterface::slotHILGPS( uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible )
{
	mavlink_message_t msg;
	mavlink_msg_hil_gps_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
								   MAVLinkProtocol::getInstance()->getComponentId(),
								   _mavlinkChannel, 
								   &msg,
								   QGC::groundTimeUsecs(),
								   fix_type,
								   lat,
								   lon,
								   alt,
								   eph,
								   epv,
								   vel,
								   vn,
								   ve,
								   vd,
								   cog,
								   satellites_visible );
	_sendMAVLinkMessage( msg );	
}

void MAVLinkCommInterface::slotHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated )
{
    mavlink_message_t msg;
	mavlink_msg_hil_sensor_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									  MAVLinkProtocol::getInstance()->getComponentId(),
									  _mavlinkChannel,
									  &msg,
									  QGC::groundTimeUsecs(),
									  xacc, yacc, zacc,
									  xgyro, ygyro, zgyro,
									  xmag, ymag, zmag,
									  abs_pressure, diff_pressure, pressure_alt,
									  temperature,
									  fields_updated );
		_sendMAVLinkMessage( msg );	
}

void MAVLinkCommInterface::slotHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
	mavlink_message_t msg;
	mavlink_msg_hil_state_quaternion_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
												MAVLinkProtocol::getInstance()->getComponentId(),
												_mavlinkChannel,
												&msg,
												QGC::groundTimeUsecs(),
												attitude_quaternion,
												rollspeed,
												pitchspeed,
												yawspeed,
												lat,
												lon,
												alt,
												vx,
												vy,
												vz,
												ind_airspeed,
												true_airspeed,
												xacc,
												yacc,
												zacc );
	_sendMAVLinkMessage( msg );	
}

void MAVLinkCommInterface::slotHeartbeat()
{
	mavlink_message_t message;

	mavlink_msg_heartbeat_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									 MAVLinkProtocol::getInstance()->getComponentId(),
									 _mavlinkChannel,
									 &message,
									 MAV_TYPE_GCS,            // MAV_TYPE
									 MAV_AUTOPILOT_INVALID,   // MAV_AUTOPILOT
									 MAV_MODE_MANUAL_ARMED,   // MAV_MODE
									 0,                       // custom mode
									 MAV_STATE_ACTIVE);       // MAV_STATE
	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotLogErase( uint8_t target_system, uint8_t target_component )
{
	mavlink_message_t msg;
	mavlink_msg_log_erase_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									 MAVLinkProtocol::getInstance()->getComponentId(),
									 _mavlinkChannel,									 
									 &msg,
									 target_system,
									 target_component );
	_sendMAVLinkMessage( msg );
}

void MAVLinkCommInterface::slotLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count )
{
	mavlink_message_t msg;
	mavlink_msg_log_request_data_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											MAVLinkProtocol::getInstance()->getComponentId(),
											_mavlinkChannel,
											&msg,
											target_system,
											target_component,
											id,
											offset,
											count );
	_sendMAVLinkMessage( msg );	
}

void MAVLinkCommInterface::slotLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end)
{
    mavlink_message_t msg;
	mavlink_msg_log_request_list_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											MAVLinkProtocol::getInstance()->getComponentId(),
											_mavlinkChannel,
											&msg,
											target_system,
											target_component,
											start,
											end);
	_sendMAVLinkMessage( msg );	
}

void MAVLinkCommInterface::slotLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq )
{
    mavlink_message_t message;
	mavlink_msg_logging_ack_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									   MAVLinkProtocol::getInstance()->getComponentId(),
									   _mavlinkChannel,
									   &message, 
									   target_system,
									   target_component,
									   seq );
	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons )
{
	mavlink_message_t message;
	mavlink_msg_manual_control_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										  MAVLinkProtocol::getInstance()->getComponentId(),
										  _mavlinkChannel,
										  &message,
										  target_system,
										  pitch,
										  roll,
										  thrust,
										  yaw,
										  buttons );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type)
{
	mavlink_message_t message;
    mavlink_msg_mission_ack_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									   MAVLinkProtocol::getInstance()->getComponentId(),
									   _mavlinkChannel,
									   &message,
									   target_system,
									   target_component,
									   type,
									   mission_type );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type )
{
	mavlink_message_t message;
	mavlink_msg_mission_clear_all_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											 MAVLinkProtocol::getInstance()->getComponentId(),
											 _mavlinkChannel,
											 &message,
											 target_system,
											 target_component,
											 mission_type );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type )
{
	mavlink_message_t message;
	mavlink_msg_mission_count_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										 MAVLinkProtocol::getInstance()->getComponentId(),
										 _mavlinkChannel,
										 &message,
										 target_system,
										 target_component,
										 count,
										 mission_type );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
	mavlink_message_t message;
	mavlink_msg_mission_item_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										MAVLinkProtocol::getInstance()->getComponentId(),
										_mavlinkChannel,
										&message,
										target_system,
										target_component,
										seq,
										frame,
										command,
										current,
										autocontinue,
										param1,
										param2,
										param3,
										param4,
										x,
										y,
										z,
										mission_type );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type)
{
	mavlink_message_t message;
	mavlink_msg_mission_item_int_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											MAVLinkProtocol::getInstance()->getComponentId(),
											_mavlinkChannel,
											&message,
											target_system,
											target_component,
											seq,
											frame,
											command,
											current,
											autocontinue,
											param1,
											param2,
											param3,
											param4,
											x,
											y,
											z,
											mission_type );
		_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type)
{
	mavlink_message_t message;
	mavlink_msg_mission_request_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										   MAVLinkProtocol::getInstance()->getComponentId(),
										   _mavlinkChannel,
										   &message,
										   target_system,
										   target_component,
										   seq,
										   mission_type);
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type)
{
	mavlink_message_t message;
	mavlink_msg_mission_request_int_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											   MAVLinkProtocol::getInstance()->getComponentId(),
											   _mavlinkChannel,
											   &message,
											   target_system,
											   target_component,
											   seq,
											   mission_type);
		_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type)
{
	mavlink_message_t message;
	mavlink_msg_mission_request_list_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
												MAVLinkProtocol::getInstance()->getComponentId(),
												_mavlinkChannel,
												&message,
												target_system,
												target_component,
												mission_type);
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMissionSetCurrentSequence( uint16_t seq )
{
	mavlink_message_t message;
	mavlink_msg_mission_current_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										   MAVLinkProtocol::getInstance()->getComponentId(),
										   _mavlinkChannel,
										   &message,
										   seq );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode)
{
	mavlink_message_t message;
	mavlink_msg_set_mode_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									MAVLinkProtocol::getInstance()->getComponentId(),
									_mavlinkChannel,
									&message,
									target_system,
									base_mode,
									custom_mode );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotParameterExtRequestList(uint8_t target_system, uint8_t target_component)
{
	mavlink_message_t msg;
    mavlink_msg_param_ext_request_list_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
												  MAVLinkProtocol::getInstance()->getComponentId(),
												  _mavlinkChannel,
												  &msg,
												  target_system,
												  target_component );
	_sendMAVLinkMessage( msg );
}

void MAVLinkCommInterface::slotParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_name )
{
    mavlink_message_t msg;
	char param_id[MAVLINK_MSG_PARAM_EXT_REQUEST_READ_FIELD_PARAM_ID_LEN + 1];
	
    memset(param_id, 0, sizeof(param_id));
    strncpy(param_id, param_name.toStdString().c_str(), MAVLINK_MSG_PARAM_EXT_REQUEST_READ_FIELD_PARAM_ID_LEN);

    mavlink_msg_param_ext_request_read_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
												  MAVLinkProtocol::getInstance()->getComponentId(),
												  _mavlinkChannel,
												  &msg,
												  target_system,
												  target_component,
												  param_id,
												  -1 );
	_sendMAVLinkMessage( msg );
}

void MAVLinkCommInterface::slotParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_name, uint8_t param_value[128],  uint8_t param_type )
{
	mavlink_message_t message;
	mavlink_param_ext_set_t param_set;

	mavlink_msg_param_ext_set_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										 MAVLinkProtocol::getInstance()->getComponentId(),
										 _mavlinkChannel,
										 &message,
										 target_system,
										 target_component,
										 param_name.toStdString().c_str(),
										 (char*)param_value,
										 param_type );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max )
{
	mavlink_message_t msg;
	mavlink_msg_param_map_rc_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
										MAVLinkProtocol::getInstance()->getComponentId(),
										_mavlinkChannel, 
										&msg,
										target_system,
										target_component,
										param_id.toStdString().c_str(),
										param_index,
										parameter_rc_channel_index,
                                        param_value0,
										scale,
										param_value_min,
										param_value_max );
	_sendMAVLinkMessage( msg );	
}

void MAVLinkCommInterface::slotParameterRequestList( uint8_t target_system, uint8_t target_component )
{
	mavlink_message_t msg;
	mavlink_msg_param_request_list_pack_chan(MAVLinkProtocol::getInstance()->getSystemId(),
											 MAVLinkProtocol::getInstance()->getComponentId(),
											 _mavlinkChannel,	
                                             &msg,
											 target_system,
											 target_component );
	_sendMAVLinkMessage( msg );
}

void MAVLinkCommInterface::slotParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex )
{
    mavlink_message_t msg;
    char fixedParamName[MAVLINK_MSG_PARAM_REQUEST_READ_FIELD_PARAM_ID_LEN];

    strncpy(fixedParamName, paramName.toStdString().c_str(), sizeof(fixedParamName));
    mavlink_msg_param_request_read_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											  MAVLinkProtocol::getInstance()->getComponentId(),
											  _mavlinkChannel,
											  &msg,                           // Pack into this mavlink_message_t
											  target_system,
											  target_component,
											  fixedParamName,                 // Named parameter being requested
											  paramIndex );
	_sendMAVLinkMessage( msg );
}

void MAVLinkCommInterface::slotParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type )
{
	mavlink_message_t message;
	mavlink_param_union_t   union_value;

	//Get value from QVariant
	switch (type)
	{
    case FactMetaData::valueTypeUint8:
        union_value.param_uint8 = (uint8_t)parameter_value.toUInt();
        break;
		
    case FactMetaData::valueTypeInt8:
        union_value.param_int8 = (int8_t)parameter_value.toInt();
        break;
		
    case FactMetaData::valueTypeUint16:
        union_value.param_uint16 = (uint16_t)parameter_value.toUInt();
        break;

    case FactMetaData::valueTypeInt16:
        union_value.param_int16 = (int16_t)parameter_value.toInt();
        break;

    case FactMetaData::valueTypeUint32:
        union_value.param_uint32 = (uint32_t)parameter_value.toUInt();
        break;

    case FactMetaData::valueTypeFloat:
        union_value.param_float = parameter_value.toFloat();
        break;

    default:
        qCritical() << "Unsupported fact type" << type;
        // fall through

    case FactMetaData::valueTypeInt32:
        union_value.param_int32 = (int32_t)parameter_value.toInt();
        break;
    }

	
	mavlink_msg_param_set_pack_chan(  MAVLinkProtocol::getInstance()->getSystemId(),
									  MAVLinkProtocol::getInstance()->getComponentId(),
									  _mavlinkChannel,
									  &message,
									  target_system,
									  target_component,
									  parameter_id.toStdString().c_str(),
									  union_value.param_float,
									  type );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotROS2GlobalWaypointCommand( int target_system, int target_component, double latitude, double longitude, double altitude )
{
	qWarning() << "MAVLinkCommInterface::slotROS2GlobalWaypointCommand - command not supported";
}

void MAVLinkCommInterface::slotSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust)
{
	mavlink_message_t message;
    mavlink_msg_set_attitude_target_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
											   MAVLinkProtocol::getInstance()->getComponentId(),
											   _mavlinkChannel,
											   &message,
											   QGC::groundTimeUsecs(),
											   target_system,
											   target_component,
											   type_mask,
											   attitude_quaternion,
											   body_roll_rate,
											   body_pitch_rate,
											   body_yaw_rate,
											   thrust );
	_sendMAVLinkMessage( message );	
}

void MAVLinkCommInterface::slotSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame)
{
	mavlink_message_t message;
	mavlink_msg_set_position_target_local_ned_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
														 MAVLinkProtocol::getInstance()->getComponentId(),
														 _mavlinkChannel,
														 &message,
														 QGC::groundTimeUsecs(),
														 target_system,
														 target_component,
														 coordinate_frame,
														 type_mask,
														 position[0],
														 position[1],
														 position[2],
														 velocity[0],
														 velocity[1],
														 velocity[2],
														 acceleration[0],
														 acceleration[1],
														 acceleration[2],
														 yaw,
														 yaw_rate );
	_sendMAVLinkMessage( message );
}

void MAVLinkCommInterface::slotSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms)
{
	mavlink_message_t message;
	mavlink_msg_system_time_pack_chan( MAVLinkProtocol::getInstance()->getSystemId(),
									   MAVLinkProtocol::getInstance()->getComponentId(),
									   _mavlinkChannel,
									   &message,
									   time_unix_usec,
									   time_boot_ms );
	_sendMAVLinkMessage( message );
}


/* *************** *
 * MAVLinkProtocol *
 * *************** */
MAVLinkProtocol* MAVLinkProtocol::_instance = NULL;
const char*  MAVLinkProtocol::_tempLogFileTemplate = "FlightDataXXXXXX";
const char*  MAVLinkProtocol::_logFileExtension = "mavlink";

MAVLinkProtocol::MAVLinkProtocol()
	: _enableVersionCheck(true),
	  _versionMismatchIgnore(false),
	  _systemId(255),
	  _currentVersion(100),
	  _radioVersionMismatchCount(0),
	  _mavlinkChannelsUsedBitMask(1),
	  _logSuspendError(false),
	  _logSuspendReplay(false),
	  _vehicleWasArmed(false),
	  _tempLogFile(QString("%2.%3").arg(_tempLogFileTemplate).arg(_logFileExtension))
{
	QGCApplication* app;
	MultiVehicleManager* vehicle_manager;
	
	memset(_lastIndex, 0, sizeof(_lastIndex));
	memset(_firstMessage, 1, sizeof(_firstMessage));
	memset(_totalReceiveCounter, 0, sizeof(_totalReceiveCounter));
	memset(_totalLossCounter, 0, sizeof(_totalLossCounter));
	memset(_runningLossPercent, 0, sizeof(_runningLossPercent));

	_loadSettings();

	app = qgcApp();
	connect(this, &MAVLinkProtocol::protocolStatusMessage,  app, &QGCApplication::criticalMessageBoxOnMainThread);
	connect(this, &MAVLinkProtocol::saveTelemetryLog,       app, &QGCApplication::saveTelemetryLogOnMainThread);
	connect(this, &MAVLinkProtocol::checkTelemetrySavePath, app, &QGCApplication::checkTelemetrySavePathOnMainThread);

	vehicle_manager = app->toolbox()->multiVehicleManager();
	connect(vehicle_manager, &MultiVehicleManager::vehicleAdded,   this, &MAVLinkProtocol::_vehicleCountChanged);
	connect(vehicle_manager, &MultiVehicleManager::vehicleRemoved, this, &MAVLinkProtocol::_vehicleCountChanged);
}
	  
MAVLinkProtocol::~MAVLinkProtocol()
{
    _storeSettings();
    _closeLogFile();	
}

MAVLinkProtocol* MAVLinkProtocol::getInstance()
{
	if( _instance == NULL )
	{
		qRegisterMetaType<mavlink_message_t>("mavlink_message_t");
		_instance = new MAVLinkProtocol();
		connect( qgcApp(), &QGCApplication::aboutToQuit, _instance, &MAVLinkProtocol::_aboutToQuit );
	}
	return _instance;
}

int MAVLinkProtocol::getSystemId()
{
	return _systemId;
}

void MAVLinkProtocol::setSystemId(int id)
{
	_systemId = id;
}

int MAVLinkProtocol::getComponentId()
{
	return 0;
}

bool MAVLinkProtocol::versionCheckEnabled()
{
	return _enableVersionCheck;
}

void MAVLinkProtocol::enableVersionCheck(bool enabled)
{
	_enableVersionCheck = enabled;
}

unsigned MAVLinkProtocol::getCurrentVersion()
{
	return _currentVersion;
}

void MAVLinkProtocol::setCurrentVersion(unsigned version)
{
	QList<CommInterface*> links = CommManager::getCommManager()->interfaces();

	//Loop through all links
	for (int i = 0; i < links.length(); i++)
	{
		//Cast link to MAVLinkCommInterface
		MAVLinkCommInterface* mavlink = dynamic_cast<MAVLinkCommInterface*>(links[i]);

		//If the link is not a MAVLink link, do nothing
		if( !mavlink )
			continue;

		//Get the mavlink_status_t associated with this link's mavlink channel
        mavlink_status_t* mavlink_status = mavlink_get_channel_status(mavlink->mavlinkChannel());
		
        // Set flags for version
        if (version < 200)
            mavlink_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		else
            mavlink_status->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }

    _currentVersion = version;	
}

int MAVLinkProtocol::getVersion()
{
	return MAVLINK_VERSION;
}

void MAVLinkProtocol::resetMetadataForInterface(CommInterface *interface)
{
	MAVLinkCommInterface* mavlink = dynamic_cast<MAVLinkCommInterface*>(interface);

	if( !mavlink )
		return;
	
    int channel = mavlink->mavlinkChannel();
    _totalReceiveCounter[channel] = 0;
    _totalLossCounter[channel]    = 0;
    _runningLossPercent[channel]  = 0.0f;
    for(int i = 0; i < 256; i++)
        _firstMessage[channel][i] =  1;

    mavlink->resetDecodedFirstMAVLinkPacket();
}

void MAVLinkProtocol::statusMessage(int uasId, uint64_t totalSent, uint64_t totalReceived, uint64_t totalLoss, float lossPercent)
{
	emit mavlinkMessageStatus(uasId, totalSent, totalReceived, totalLoss, lossPercent);
}

int MAVLinkProtocol::reserveMAVLinkChannel()
{
    // Find a mavlink channel to use for this link, Channel 0 is reserved for internal use.
    for (uint8_t mavlink_channel = 1; mavlink_channel < MAVLINK_COMM_NUM_BUFFERS; mavlink_channel++)
	{
        if (!(_mavlinkChannelsUsedBitMask & 1 << mavlink_channel))
		{
            mavlink_reset_channel_status(mavlink_channel);
			
            // Start the channel on Mav 1 protocol
            mavlink_status_t* mavlink_status = mavlink_get_channel_status(mavlink_channel);
            mavlink_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
            _mavlinkChannelsUsedBitMask |= 1 << mavlink_channel;
            return mavlink_channel;
        }
    }
    return 0;   // All channels reserved
}

void MAVLinkProtocol::freeMAVLinkChannel(int channel)
{
	_mavlinkChannelsUsedBitMask &= ~(1 << channel);
}


void MAVLinkProtocol::suspendLogForReplay(bool suspend)
{
	_logSuspendReplay = suspend;
}

void MAVLinkProtocol::logMessage( mavlink_message_t* message )
{
	if (!_logSuspendError && !_logSuspendReplay && _tempLogFile.isOpen())
	{
		uint8_t buf[MAVLINK_MAX_PACKET_LEN+sizeof(quint64)];
		
		// Write the uint64 time in microseconds in big endian format before the message.
		// This timestamp is saved in UTC time. We are only saving in ms precision because
		// getting more than this isn't possible with Qt without a ton of extra code.
		quint64 time = static_cast<quint64>(QDateTime::currentMSecsSinceEpoch() * 1000);
		qToBigEndian(time, buf);
		
		// Then write the message to the buffer
		int len = mavlink_msg_to_send_buffer(buf + sizeof(quint64), message);
		
		// Determine how many bytes were written by adding the timestamp size to the message size
		len += sizeof(quint64);
		
		// Now write this timestamp/message pair to the log.
		QByteArray b(reinterpret_cast<const char*>(buf), len);
		if(_tempLogFile.write(b) != len)
		{
			// If there's an error logging data, raise an alert and stop logging.
			emit protocolStatusMessage(tr("MAVLink Protocol"), tr("MAVLink Logging failed. Could not write to file %1, logging disabled.").arg(_tempLogFile.fileName()));
			_stopLogging();
			_logSuspendError = true;
		}
		
		// Check for the vehicle arming going by. This is used to trigger log save.
		if (!_vehicleWasArmed && message->msgid == MAVLINK_MSG_ID_HEARTBEAT)
		{
			mavlink_heartbeat_t state;
			mavlink_msg_heartbeat_decode(message, &state);
			if (state.base_mode & MAV_MODE_FLAG_DECODE_POSITION_SAFETY)
			{
				_vehicleWasArmed = true;
			}
		}
	}	
}

void MAVLinkProtocol::deleteTempLogFiles()
{
    QDir tempDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation));

    QString filter(QString("*.%1").arg(_logFileExtension));
    QFileInfoList fileInfoList = tempDir.entryInfoList(QStringList(filter), QDir::Files);

    for(const QFileInfo fileInfo: fileInfoList)
        QFile::remove(fileInfo.filePath());
}

bool MAVLinkProtocol::_closeLogFile()
{
    if (_tempLogFile.isOpen())
	{
        if (_tempLogFile.size() == 0)
		{
            // Don't save zero byte files
            _tempLogFile.remove();
            return false;
        }
		else
		{
            _tempLogFile.flush();
            _tempLogFile.close();
            return true;
        }
    }
    return false;	
}

void MAVLinkProtocol::_startLogging()
{
    //-- Are we supposed to write logs?
    if (qgcApp()->runningUnitTests())
        return;

#ifdef __mobile__
    //-- Mobile build don't write to /tmp unless told to do so
    if (!qgcApp()->toolbox()->settingsManager()->appSettings()->telemetrySave()->rawValue().toBool())
        return;

#endif
    //-- Log is always written to a temp file. If later the user decides they want
    //   it, it's all there for them.
    if (!_tempLogFile.isOpen())
	{
        if (!_logSuspendReplay)
		{
            if (!_tempLogFile.open())
			{
                emit protocolStatusMessage(tr("MAVLink Protocol"), tr("Opening Flight Data file for writing failed. "
                                                                      "Unable to write to %1. Please choose a different file location.").arg(_tempLogFile.fileName()));
                _closeLogFile();
                _logSuspendError = true;
                return;
            }

            qDebug() << "Temp log" << _tempLogFile.fileName();
            emit checkTelemetrySavePath();

            _logSuspendError = false;
        }
    }	
}

void MAVLinkProtocol::_stopLogging()
{
    if (_tempLogFile.isOpen())
	{
        if (_closeLogFile())
		{
            if ((_vehicleWasArmed || qgcApp()->toolbox()->settingsManager()->appSettings()->telemetrySaveNotArmed()->rawValue().toBool()) &&
                qgcApp()->toolbox()->settingsManager()->appSettings()->telemetrySave()->rawValue().toBool())
			{
                emit saveTelemetryLog(_tempLogFile.fileName());
            }
			else
			{
                QFile::remove(_tempLogFile.fileName());
            }
        }
    }
    _vehicleWasArmed = false;	
}

void MAVLinkProtocol::_loadSettings()
{
    // Load defaults from settings
    QSettings settings;
    settings.beginGroup("QGC_MAVLINK_PROTOCOL");
    enableVersionCheck(settings.value("VERSION_CHECK_ENABLED", _enableVersionCheck).toBool());

    // Only set system id if it was valid
    int temp = settings.value("GCS_SYSTEM_ID", _systemId).toInt();
    if (temp > 0 && temp < 256)
    {
        _systemId = temp;
    }	
}

void MAVLinkProtocol::_storeSettings()
{
    // Store settings
    QSettings settings;
    settings.beginGroup("QGC_MAVLINK_PROTOCOL");
    settings.setValue("VERSION_CHECK_ENABLED", _enableVersionCheck);
    settings.setValue("GCS_SYSTEM_ID", _systemId);
}

void MAVLinkProtocol::checkForLostLogFiles()
{
    QDir tempDir(QStandardPaths::writableLocation(QStandardPaths::TempLocation));

    QString filter(QString("*.%1").arg(_logFileExtension));
    QFileInfoList fileInfoList = tempDir.entryInfoList(QStringList(filter), QDir::Files);
    //qDebug() << "Orphaned log file count" << fileInfoList.count();

    for(const QFileInfo fileInfo: fileInfoList)
	{
        //qDebug() << "Orphaned log file" << fileInfo.filePath();
        if (fileInfo.size() == 0)
		{
            // Delete all zero length files
            QFile::remove(fileInfo.filePath());
            continue;
        }
        emit saveTelemetryLog(fileInfo.filePath());
    }	
}

void MAVLinkProtocol::_aboutToQuit()
{
	if( _instance != NULL )
		delete _instance;
	_instance = NULL;
}

void MAVLinkProtocol::_vehicleCountChanged()
{
	if( qgcApp()->toolbox()->multiVehicleManager()->vehicles()->count() == 0)
	{
        // Last vehicle is gone, close out logging
        _stopLogging();
        _radioVersionMismatchCount = 0;
    }
}
