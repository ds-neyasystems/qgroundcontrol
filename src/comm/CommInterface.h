#pragma once

#include <QThread>
#include <CommInterfaceConfiguration.h>
#include <MissionItem.h>

class MavlinkMessagesTimer;
class CommManager;
class Vehicle;

class CommInterface : public QThread
{
	Q_OBJECT

	friend class CommManager;
	friend class CommInterfaceConfiguration;
protected:
	CommInterface( CommInterfaceConfiguration::SharedPointer config, bool is_px4 = false );
	
public:
	typedef QSharedPointer<CommInterface> SharedPointer;
	
    Q_PROPERTY(bool active      READ active     NOTIFY activeChanged)
    Q_PROPERTY(bool isPX4Flow   READ isPX4Flow  CONSTANT)

	virtual ~CommInterface();
	
    Q_INVOKABLE bool interfaceActive(int vehicle_id) const;
    Q_INVOKABLE bool getHighLatency(void) const;
	Q_INVOKABLE void sendGCSHeartbeat();
	
	static CommInterfaceConfiguration* create(int type, const QString name);
	
    bool isPX4Flow(void) const;// { return _isPX4Flow; }
	virtual bool isLogReplay(void);

	CommInterfaceConfiguration::SharedPointer getConfiguration();
	
	void startActivityTimer(int vehicle_id);
    void clearActivityTimers();
	
	Q_INVOKABLE virtual QString getName() const;
	virtual bool isConnected() const = 0;
    virtual bool active() const;

	virtual void resetMetaData() = 0;
	virtual void requestReset() = 0;

	//Send Message Functions
    void sendCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z) ;
	void sendCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 ) ;
    void sendCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component ) ;
	void sendDataTransmissionHandshake( uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality)  ;
    void sendFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities ) ;
	void sendFileTransfer( uint8_t target_system, QByteArray payload );
    void sendGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id ) ;
    void sendHILGPS( uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible ) ;
    void sendHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated ) ;
    void sendHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc) ;
    void sendHeartbeat() ;
    void sendLogErase( uint8_t target_system, uint8_t target_component ) ;
    void sendLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count ) ;
    void sendLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end) ;
	void sendLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq );
    void sendManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons ) ;
	void sendMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type);
	void sendMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type );
	void sendMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type );
	void sendMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type);
	void sendMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type);
	void sendMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type);
	void sendMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type);
	void sendMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type);
	void sendMissionSetCurrentSequence( uint16_t seq );
	void sendMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode);
	void sendParameterExtRequestList( uint8_t target_system, uint8_t target_component ) ;
    void sendParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_id ) ;
    void sendParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_id, uint8_t param_value[128],  uint8_t param_type ) ;
    void sendParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max ) ;
    void sendParameterRequestList( uint8_t target_system, uint8_t target_component ) ;
    void sendParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex ) ;
    void sendParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type ) ;
	void sendROS2GlobalWaypointCommand( int target_system, int target_component, double latitude, double longitude, double altitude );
    void sendSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust) ;
    void sendSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame) ;
	void sendSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms);
	
protected:
	virtual bool _connect(void) = 0;
    virtual void _disconnect(void) = 0;
	
public slots:
	//Send Message Slots
    virtual void slotCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z) = 0;
	virtual void slotCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 ) = 0;
    virtual void slotCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component ) = 0;
	virtual void slotDataTransmissionHandshake( uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality)  = 0;
    virtual void slotFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities ) = 0;
	virtual void slotFileTransfer( uint8_t target_system, QByteArray payload ) = 0;
    virtual void slotGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id ) = 0;
    virtual void slotHILGPS( uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible ) = 0;
    virtual void slotHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated ) = 0;
    virtual void slotHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc) = 0;
    virtual void slotHeartbeat() = 0;
    virtual void slotLogErase( uint8_t target_system, uint8_t target_component ) = 0;
    virtual void slotLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count ) = 0;
    virtual void slotLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end) = 0;
	virtual void slotLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq ) = 0;
    virtual void slotManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons ) = 0;
	virtual void slotMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type) = 0;
	virtual void slotMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type ) = 0;
	virtual void slotMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type ) = 0;
	virtual void slotMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type) = 0;
	virtual void slotMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type) = 0;
	virtual void slotMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type) = 0;
	virtual void slotMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type) = 0;
	virtual void slotMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type) = 0;
	virtual void slotMissionSetCurrentSequence( uint16_t seq ) = 0;
	virtual void slotMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode) = 0;
    virtual void slotParameterExtRequestList( uint8_t target_system, uint8_t target_component ) = 0;
    virtual void slotParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_id ) = 0;
    virtual void slotParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_id, uint8_t param_value[128],  uint8_t param_type ) = 0;
    virtual void slotParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max ) = 0;
    virtual void slotParameterRequestList( uint8_t target_system, uint8_t target_component ) = 0;
    virtual void slotParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex ) = 0;
    virtual void slotParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type ) = 0;
	virtual void slotROS2GlobalWaypointCommand( int target_system, int target_component, double latitude, double longitude, double altitude ) = 0;
    virtual void slotSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust) = 0;
    virtual void slotSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame) = 0;
	virtual void slotSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms) = 0;

protected slots:
	void _activeChanged(bool active, int vehicle_id);
		
signals:	
	void activeChanged(CommInterface* link, bool active, int vehicle_id);
	void communicationError(const QString& title, const QString& error);
	void connectionRemoved(CommInterface* interface);
	void vehicleHeartbeatInfo( CommInterface* interface, int vehicle_id, int component_id, int autopilot_type, int type );

	//Send Message Signals
    void signalCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z) ;
	void signalCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 ) ;
    void signalCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component ) ;
	void signalDataTransmissionHandshake( uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality)  ;
    void signalFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities ) ;
	void signalFileTransfer( uint8_t target_system, QByteArray payload );
    void signalGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id ) ;
    void signalHILGPS( uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible ) ;
    void signalHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated ) ;
    void signalHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc) ;
    void signalHeartbeat() ;
    void signalLogErase( uint8_t target_system, uint8_t target_component ) ;
    void signalLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count ) ;
    void signalLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end) ;
	void signalLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq );
    void signalManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons ) ;
	void signalMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type);
	void signalMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type );
	void signalMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type );
	void signalMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type);
	void signalMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type);
	void signalMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type);
	void signalMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type);
	void signalMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type);
	void signalMissionSetCurrentSequence( uint16_t seq );
	void signalMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode);
	void signalParameterExtRequestList( uint8_t target_system, uint8_t target_component ) ;
    void signalParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_id ) ;
    void signalParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_id, uint8_t param_value[128],  uint8_t param_type ) ;
    void signalParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max ) ;
    void signalParameterRequestList( uint8_t target_system, uint8_t target_component ) ;
    void signalParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex ) ;
    void signalParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type ) ;
	void signalROS2GlobalWaypointCommand( int target_system, int target_component, double latitude, double longitude, double altitude );
    void signalSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust) ;
    void signalSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame) ;
	void signalSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms);
		
	//Received Message Signals
	void receivedADSBVehicle( Vehicle* vehicle, uint32_t ICAO_address, int32_t latitude, int32_t longitude, int32_t altitude, uint16_t heading, uint16_t flags, QString callsign );
//	void receivedAltitude( Vehicle* vehicle, float altitude );
	void receivedAltitude( Vehicle* vehicle, float altitude_amsl, float altitude_relative );
	void receivedAttitude( Vehicle* vehicle, float roll, float pitch, float yaw );
	void receivedAttitudeQuaternion( Vehicle* vehicle, float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate );
	void receivedAttitudeTarget( Vehicle* vehicle, float roll, float pitch, float yaw, float roll_rate, float pitch_rate, float yaw_rate );
//	void receivedAutopilotVersion( Vehicle* vehicle, uint64_t uid, int firmware_version_type, int major_version, int minor_version, int patch_version, int custom_major_version, int custom_minor_version, int custom_patch_version, bool capability_misison, bool capability_command, bool capability_mavlink2, bool capability_fence, bool capability_rally );
	void receivedAutopilotVersion( Vehicle* vehicle, uint64_t uid, uint32_t version, uint64_t custom_version, uint64_t capabilities );
	void receivedBatteryChargeState( Vehicle* vehicle, int battery_id, int charge_state );
	void receivedBatteryCurrentConsumed( Vehicle* vehicle, int battery_id, int current_consumed );
	void receivedBatteryRemaining( Vehicle* vehicle, int battery_id, int8_t percent_remaining );
//	void receivedBatteryStatus( Vehicle* vehicle, int battery_id, double temperature, int current_consumed, int cell_count, int time_remaining, int charge_state );
	void receivedBatteryVoltage( Vehicle* vehicle, int battery_id, double voltage, double current_battery );
	void receivedCameraCaptureStatus( Vehicle* vehicle, float available_capacity, unsigned char image_status, unsigned char video_status );
//	void receivedCameraFeedback( Vehicle* vehicle, latitude, longitude, altitude );
	void receivedCameraImageCaptured( Vehicle* vehicle, double latitude, double longitude, float altitude );
	void receivedCameraInfo(Vehicle* vehicle);
	void receivedCameraSettings( Vehicle* vehicle, unsigned char mode, float zoom_level, float focus_level );
	void receivedCommandAck( Vehicle* vehicle, int component_id, int command, int result );
	void receivedCommandLong(Vehicle* vehicle);
	void receivedDataTransmissionHandshake( Vehicle* vehicle, uint32_t size, uint16_t width, uint16_t height, uint16_t packets, uint8_t type, uint8_t payload, uint8_t jpg_quality );
	void receivedDistanceSensor( Vehicle* vehicle, uint8_t id, uint16_t current_distnace, uint8_t orientation );
//	void receivedEncapsulatedData(Vehicle* vehicle);
	void receivedEstimatorStatus( Vehicle* vehicle, float vel_ratio, float pos_horiz_ratio, float pos_vert_ratio, float mag_ratio, float hagl_ratio, float tas_ratio, float pos_horiz_accuracy, float pos_vert_accuracy, bool estimator_attitude, bool estimator_velocity_horiz, bool estimator_velocity_vert, bool estimator_pos_horiz_rel, bool estimator_pos_horiz_abs, bool estimator_pos_vert_abs, bool estimator_pos_vert_agl, bool estimator_const_pos_mode, bool estimator_pred_pos_horiz_rel, bool estimator_pred_pos_horiz_abs, bool estimator_gps_glitch, bool estimator_accel_error );
	void receivedExtendedSysState( Vehicle* vehicle, uint8_t vtol_state, uint8_t landed_state );
	void receivedGPS( Vehicle* vehicle, double latitude, double longitude, float altitude );
	void receivedGPSData( Vehicle* vehicle, double latitude, double longitude, uint16_t eph, uint16_t epv, uint16_t course_over_ground, uint8_t fix_type, uint8_t satellites_visible );
//	void receivedHILActuatorControls(Vehicle* vehicle);
	void receivedHeartbeat( Vehicle* vehicle, uint8_t base_mode, uint8_t custom_mode );
//	void receivedHeartbeat( Vehicle* vehicle, CommInterface* interface, uint8_t vehicle_id, uint8_t component_id, uint8_t vehicle_firmware_type, uint8_t vehicle_type );
	void receivedHomePosition( Vehicle* vehicle, double latitude, double longitude, double altitude );
//	void receivedLogData(Vehicle* vehicle);
//	void receivedLogEntry(Vehicle* vehicle);
//	void receivedLogDataAck(Vehicle* vehicle);
//	void receivedMagneticCalibrationProgress(Vehicle* vehicle);
	void receivedMagneticCalibrationReport( Vehicle* vehicle, uint8_t compass_id, uint8_t calibration_status, float fitness );
	void receivedMissionAck( Vehicle* vehicle, uint8_t type, uint8_t mission_type );
	void receivedMissionCount( Vehicle* vehicle, uint8_t mission_type, uint16_t count );
	void receivedMissionCurrent( Vehicle* vehicle, uint16_t sequence );
	void receivedMissionItem( Vehicle* vehicle, MissionItem mission_item );
	void receivedMissionRequest( Vehicle* vehicle, uint8_t mission_type, uint16_t sequence, bool mission_int );
	void receivedOrbitExecutionStatus( Vehicle* vehicle, double latitude, double longitude, float radius );
//	void receivedParamAck(Vehicle* vehicle);
	void receivedParamValue( Vehicle* vehicle, uint16_t parameter_count, uint16_t parameter_index, QString parameter_name, QVariant parameter_value );
	void receivedPing( Vehicle* vehicle, uint64_t time_usec, uint32_t sequence );
	void receivedPreflightRebootShutdown( Vehicle* vehicle, float reboot_shutdown_command );
	void receivedProtocolVersion(Vehicle* vehicle);
	void receivedRCChannels( Vehicle* vehicle, uint8_t rssi, QList<int> pwm_values );
//	void receivedRCChannelsRaw(Vehicle* vehicle);
	void receivedRadioStatus(Vehicle* vehicle );
	void receivedRawIMU(Vehicle* vehicle );
	void receivedScaledIMU(Vehicle* vehicle );
//	void receivedScaledPressure(Vehicle* vehicle);
	void receivedStatusText(Vehicle* vehicle );
	void receivedStorageInfo(Vehicle* vehicle );
	void receivedSysStatus(Vehicle* vehicle );
	void receivedTemperature( Vehicle* vehicle, int sensor_id, double temperature );
	void receivedVFRHUD(Vehicle* vehicle );
	void receivedVelocity( Vehicle* vehicle, double vx, double vy, double vz );
	void receivedVibration(Vehicle* vehicle );
	void receivedWind(Vehicle* vehicle);
	void receivedWindCovariance(Vehicle* vehicle);	

protected:
	bool _isPX4Flow;
    bool _highLatency;
	CommInterfaceConfiguration::SharedPointer _config;
	QMap<int, MavlinkMessagesTimer*> _activityTimers;
};
typedef QSharedPointer<CommInterface> SharedCommInterfacePointer;
