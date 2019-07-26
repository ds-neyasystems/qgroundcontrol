#include "CommInterface.h"
#include "MavlinkMessagesTimer.h"
#include "QGCApplication.h"
#include "MultiVehicleManager.h"
#include "Vehicle.h"

CommInterface::CommInterface( CommInterfaceConfiguration::SharedPointer config, bool is_px4 )
	: QThread(),
	  _isPX4Flow(is_px4),
	  _highLatency(false),
	  _config(config)
{
	QQmlEngine::setObjectOwnership(this, QQmlEngine::CppOwnership);

//    _config->setLink(this);

    // Initialize everything for the data rate calculation buffers.
//    _inDataIndex  = 0;
//    _outDataIndex = 0;

    // Initialize our data rate buffers.
//    memset(_inDataWriteAmounts, 0, sizeof(_inDataWriteAmounts));
//    memset(_inDataWriteTimes,   0, sizeof(_inDataWriteTimes));
//    memset(_outDataWriteAmounts,0, sizeof(_outDataWriteAmounts));
//    memset(_outDataWriteTimes,  0, sizeof(_outDataWriteTimes));

//    QObject::connect(this, &LinkInterface::_invokeWriteBytes, this, &LinkInterface::_writeBytes);
//    qRegisterMetaType<CommInterface*>("CommInterface*");

	connect(this, &CommInterface::vehicleHeartbeatInfo, qgcApp()->toolbox()->multiVehicleManager(), &MultiVehicleManager::vehicleHeartbeatInfo);
	
	QObject::connect(this, &CommInterface::signalCommandInt, this, &CommInterface::slotCommandInt);
	QObject::connect(this, &CommInterface::signalCommandLong, this, &CommInterface::slotCommandLong);
	QObject::connect(this, &CommInterface::signalCommandAck, this, &CommInterface::slotCommandAck);
	QObject::connect(this, &CommInterface::signalDataTransmissionHandshake, this, &CommInterface::slotDataTransmissionHandshake);
    QObject::connect(this, &CommInterface::signalFollowTarget, this, &CommInterface::slotFollowTarget);
	QObject::connect(this, &CommInterface::signalFileTransfer, this, &CommInterface::slotFileTransfer);
    QObject::connect(this, &CommInterface::signalGPSRTCM, this, &CommInterface::slotGPSRTCM);
    QObject::connect(this, &CommInterface::signalHILGPS, this, &CommInterface::slotHILGPS);
    QObject::connect(this, &CommInterface::signalHILSensor, this, &CommInterface::slotHILSensor);
    QObject::connect(this, &CommInterface::signalHILStateQuaternion, this, &CommInterface::slotHILStateQuaternion);
    QObject::connect(this, &CommInterface::signalHeartbeat, this, &CommInterface::slotHeartbeat);
    QObject::connect(this, &CommInterface::signalLogErase, this, &CommInterface::slotLogErase);
    QObject::connect(this, &CommInterface::signalLogRequestData, this, &CommInterface::slotLogRequestData);
    QObject::connect(this, &CommInterface::signalLogRequestList, this, &CommInterface::slotLogRequestList);
	QObject::connect(this, &CommInterface::signalLoggingAck, this, &CommInterface::slotLoggingAck);
    QObject::connect(this, &CommInterface::signalManualControl, this, &CommInterface::slotManualControl);
	QObject::connect(this, &CommInterface::signalMissionAck, this, &CommInterface::slotMissionAck);
	QObject::connect(this, &CommInterface::signalMissionAllClear , this, &CommInterface::signalMissionAllClear);
	QObject::connect(this, &CommInterface::signalMissionCount , this, &CommInterface::signalMissionCount);
	QObject::connect(this, &CommInterface::signalMissionItem , this, &CommInterface::signalMissionItem);
	QObject::connect(this, &CommInterface::signalMissionItemInt , this, &CommInterface::signalMissionItemInt);
	QObject::connect(this, &CommInterface::signalMissionRequest , this, &CommInterface::signalMissionRequest);
	QObject::connect(this, &CommInterface::signalMissionRequestInt , this, &CommInterface::signalMissionRequestInt);
	QObject::connect(this, &CommInterface::signalMissionRequestList , this, &CommInterface::signalMissionRequestList);
	QObject::connect(this, &CommInterface::signalMissionSetCurrentSequence , this, &CommInterface::signalMissionSetCurrentSequence);
	QObject::connect(this, &CommInterface::signalMode , this, &CommInterface::signalMode);
    QObject::connect(this, &CommInterface::signalParameterExtRequestList, this, &CommInterface::slotParameterExtRequestList);
    QObject::connect(this, &CommInterface::signalParameterExtRequestRead, this, &CommInterface::slotParameterExtRequestRead);
    QObject::connect(this, &CommInterface::signalParameterExtSet, this, &CommInterface::slotParameterExtSet);
    QObject::connect(this, &CommInterface::signalParameterMapRC, this, &CommInterface::slotParameterMapRC);
    QObject::connect(this, &CommInterface::signalParameterRequestList, this, &CommInterface::slotParameterRequestList);
    QObject::connect(this, &CommInterface::signalParameterRequestRead, this, &CommInterface::slotParameterRequestRead);
    QObject::connect(this, &CommInterface::signalParameterSet, this, &CommInterface::slotParameterSet);
	QObject::connect(this, &CommInterface::signalROS2GlobalWaypointCommand, this, &CommInterface::slotROS2GlobalWaypointCommand);
    QObject::connect(this, &CommInterface::signalSetAttitudeTarget, this, &CommInterface::slotSetAttitudeTarget);
    QObject::connect(this, &CommInterface::signalSetPositionTargetLocalNED, this, &CommInterface::slotSetPositionTargetLocalNED);
	QObject::connect(this, &CommInterface::signalSystemTime, this, &CommInterface::slotSystemTime);
}

CommInterface::~CommInterface()
{
}

bool CommInterface::interfaceActive(int vehicle_id) const
{
	if( _activityTimers.contains(vehicle_id) )
		return _activityTimers.value(vehicle_id)->getActive();
	return false;
}

bool CommInterface::getHighLatency(void) const
{
	return _highLatency;
}

void CommInterface::sendGCSHeartbeat()
{
	sendHeartbeat();
}

bool CommInterface::isPX4Flow(void) const
{
	return _isPX4Flow;
}

bool CommInterface::isLogReplay(void)
{
	return false;
}

CommInterfaceConfiguration::SharedPointer CommInterface::getConfiguration()
{
	return _config;
}

void CommInterface::startActivityTimer(int vehicle_id)
{
    if (_activityTimers.contains(vehicle_id))
        _activityTimers.value(vehicle_id)->restartTimer();
	else
	{
        _activityTimers.insert(vehicle_id, new MavlinkMessagesTimer(vehicle_id, _highLatency));
        QObject::connect(_activityTimers.value(vehicle_id), &MavlinkMessagesTimer::activeChanged, this, &CommInterface::_activeChanged);
        _activityTimers.value(vehicle_id)->init();
    }
}

void CommInterface::clearActivityTimers()
{
   QMapIterator<int, MavlinkMessagesTimer*> iter(_activityTimers);
    while (iter.hasNext())
	{
        iter.next();
        QObject::disconnect(iter.value(), &MavlinkMessagesTimer::activeChanged, this, &CommInterface::_activeChanged);
        _activityTimers[iter.key()]->deleteLater();
        _activityTimers[iter.key()] = nullptr;
    }

    _activityTimers.clear();	
}

QString CommInterface::getName() const
{
	return _config->name();
}

bool CommInterface::active() const
{
    QMapIterator<int, MavlinkMessagesTimer*> iter(_activityTimers);
    while (iter.hasNext())
	{
        iter.next();
        if (iter.value()->getActive())
            return true;
    }

    return false;
}

void CommInterface::sendCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
	emit signalCommandInt( target_system, target_component, frame, command, current, autocontinue, param1, param2, param3, param4,  x, y, z);
}

void CommInterface::sendCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 )
{
	emit signalCommandLong( target_system, target_component, command, confirmation, param1, param2, param3, param4, param5, param6, param7 );
}

void CommInterface::sendCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component )
{
	emit signalCommandAck( command, result, progress, result_param2, target_system, target_component );
}

void CommInterface::sendDataTransmissionHandshake( uint8_t type, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t payload, uint8_t jpg_quality)
{
	emit signalDataTransmissionHandshake( type, size, width, height, packets, payload, jpg_quality);
}

void CommInterface::sendFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities )
{
	emit signalFollowTarget( latitude, longitude, altitude, velocity, acceleration, attitude_q, rates, position_cov, est_capabilities );
}

void CommInterface::sendFileTransfer( uint8_t target_system, QByteArray payload )
{
	emit sendFileTransfer( target_system, payload );
}

void CommInterface::sendGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id )
{
	emit signalGPSRTCM( rtcm_data, sequence_id );
}

void CommInterface::sendHILGPS( uint8_t fix_type, int32_t lat, int32_t lon, int32_t alt, uint16_t eph, uint16_t epv, uint16_t vel, int16_t vn, int16_t ve, int16_t vd, uint16_t cog, uint8_t satellites_visible )
{
	emit signalHILGPS( fix_type, lat, lon, alt, eph, epv, vel, vn, ve, vd, cog, satellites_visible );
}

void CommInterface::sendHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated )
{
	emit signalHILSensor( xacc, yacc, zacc, xgyro, ygyro, zgyro, xmag, ymag, zmag, abs_pressure, diff_pressure, pressure_alt, temperature, fields_updated );
}

void CommInterface::sendHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
	emit signalHILStateQuaternion( attitude_quaternion, rollspeed, pitchspeed, yawspeed, lat, lon, alt, vx, vy, vz, ind_airspeed, true_airspeed, xacc, yacc, zacc );
}

void CommInterface::sendHeartbeat()
{
	emit signalHeartbeat();
}

void CommInterface::sendLogErase( uint8_t target_system, uint8_t target_component )
{
	emit signalLogErase( target_system,  target_component );
}

void CommInterface::sendLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count )
{
	emit signalLogRequestData( target_system, target_component, id, offset, count );
}

void CommInterface::sendLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end)
{
	emit signalLogRequestList( target_system, target_component, start, end );
}

void CommInterface::sendLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq )
{
	emit signalLoggingAck( target_system, target_component, seq );
}

void CommInterface::sendManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons )
{
	emit signalManualControl( target_system, pitch, roll, thrust, yaw, buttons );
}

void CommInterface::sendMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type)
{
	emit signalMissionAck(target_system, target_component, type, mission_type);
}

void CommInterface::sendMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type )
{
	emit signalMissionAllClear( target_system, target_component, mission_type );
}

void CommInterface::sendMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type )
{
	emit signalMissionCount( target_system, target_component, count, mission_type );
}

void CommInterface::sendMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
	emit signalMissionItem( target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type);
}

void CommInterface::sendMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type)
{
	emit signalMissionItemInt(  target_system, target_component, seq, frame, command, current, autocontinue, param1, param2, param3, param4, x, y, z, mission_type);
}

void CommInterface::sendMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type)
{
	emit signalMissionRequest( target_system, target_component, seq, mission_type);
}

void CommInterface::sendMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type)
{
	emit signalMissionRequestInt( target_system, target_component, seq, mission_type);
}

void CommInterface::sendMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type)
{
	emit signalMissionRequestList(target_system, target_component, mission_type);
}

void CommInterface::sendMissionSetCurrentSequence( uint16_t seq )
{
	emit signalMissionSetCurrentSequence( seq );
}

void CommInterface::sendMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode)
{
	emit signalMode(target_system, base_mode, custom_mode);
}

void CommInterface::sendParameterExtRequestList( uint8_t target_system, uint8_t target_component )
{
	emit signalParameterExtRequestList( target_system, target_component );
}

void CommInterface::sendParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_id )
{
	emit signalParameterExtRequestRead( target_system, target_component, param_id );
}

void CommInterface::sendParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_id, uint8_t param_value[128],  uint8_t param_type )
{
	emit signalParameterExtSet( target_system, target_component, param_id, param_value, param_type );
}

void CommInterface::sendParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max )
{
	emit signalParameterMapRC( target_system, target_component, param_id, param_index, parameter_rc_channel_index, param_value0, scale, param_value_min, param_value_max );
}

void CommInterface::sendParameterRequestList( uint8_t target_system, uint8_t target_component )
{
	emit signalParameterRequestList( target_system, target_component );
}

void CommInterface::sendParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex )
{
	emit signalParameterRequestRead( target_system, target_component, paramName, paramIndex );
}

void CommInterface::sendParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type )
{
	emit signalParameterSet( target_system, target_component, parameter_value, parameter_id, type );
}

void CommInterface::sendROS2GlobalWaypointCommand( int target_system, int target_component, double latitude, double longitude, double altitude )
{
	emit signalROS2GlobalWaypointCommand( target_system, target_component, latitude, longitude, altitude );
}

void CommInterface::sendSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust)
{
	emit signalSetAttitudeTarget( target_system, target_component, type_mask, attitude_quaternion, body_roll_rate, body_pitch_rate, body_yaw_rate, thrust );
}

void CommInterface::sendSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame)
{
	emit signalSetPositionTargetLocalNED( target_system, target_component, position, velocity, acceleration, yaw, yaw_rate, type_mask, coordinate_frame );
}

void CommInterface::sendSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms)
{
	emit signalSystemTime(time_unix_usec, time_boot_ms);
}

void CommInterface::_activeChanged(bool active, int vehicle_id)
{
	emit activeChanged(this, active, vehicle_id);
}
