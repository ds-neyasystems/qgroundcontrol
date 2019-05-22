#pragma once

#include "CommInterface.h"
#include "QGCMAVLink.h"
#include "QGCTemporaryFile.h"

class CommManager;

/* ******************** *
 * MAVLinkCommInterface *
 * ******************** */
class MAVLinkCommInterface : public CommInterface
{
	Q_OBJECT

    // Only LinkManager is allowed to create/delete or _connect/_disconnect a link
    friend class CommManager;
protected:
	MAVLinkCommInterface( CommInterfaceConfiguration::SharedPointer config, bool isPX4Flow = false );
	virtual ~MAVLinkCommInterface();
	
public:
    uint8_t mavlinkChannel() const;
	
	void resetDecodedFirstMAVLinkPacket();
	
	void resetMetaData() override;
protected:
	void _parseBytes( QByteArray bytes );
	virtual void _sendMAVLinkMessage( mavlink_message_t& msg ) = 0;	
	void _handleMessage( mavlink_message_t& message );
	
    void _setMavlinkChannel(uint8_t channel);

public slots:
	//From CommInterface
    void slotCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z);
	void slotCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 );
    void slotCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component );
	void slotDataTransmissionHandshake( uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality) ;
    void slotFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities );
	void slotFileTransfer( uint8_t target_system, QByteArray payload );
    void slotGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id );
    void slotHILGPS( uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible );
    void slotHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated );
    void slotHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc);
    void slotHeartbeat();
    void slotLogErase( uint8_t target_system, uint8_t target_component );
    void slotLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count );
    void slotLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end);
	void slotLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq );
	void slotManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons );
	void slotMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type);
	void slotMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type );
	void slotMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type );
	void slotMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type);
	void slotMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type);
	void slotMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type);
	void slotMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type);
	void slotMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type);
	void slotMissionSetCurrentSequence( uint16_t seq );
	void slotMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode);
	void slotParameterExtRequestList( uint8_t target_system, uint8_t target_component );
    void slotParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_id );
    void slotParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_id, uint8_t param_value[128],  uint8_t param_type );
    void slotParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max );
    void slotParameterRequestList( uint8_t target_system, uint8_t target_component );
    void slotParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex );
    void slotParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type );
    void slotSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust);
    void slotSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame);
	void slotSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms);
signals:	
	void protocolStatusMessage(const QString& title, const QString& message);
	
protected:
	bool _mavlinkChannelSet;         ///< true: _mavlinkChannel has been set
    uint8_t _mavlinkChannel;         ///< mavlink channel to use for this link, as used by mavlink_parse_char
	bool _decodedFirstMavlinkPacket; ///< true: link has correctly decoded it's first mavlink packet
    mavlink_message_t _message;
    mavlink_status_t _status;
	bool _gpsIntAvailable;
	QList<int> _unhandledMessageIDList;
};

/* *************** *
 * MAVLinkProtocol *
 * *************** */
class MAVLinkProtocol : public QObject
{
	Q_OBJECT

	friend MAVLinkCommInterface;
protected:
	MAVLinkProtocol();
	~MAVLinkProtocol();

public:
	static MAVLinkProtocol* getInstance();

	int getSystemId();
	void setSystemId(int id);
	
	int getComponentId();
	
	bool versionCheckEnabled();
	void enableVersionCheck(bool enabled);
	
	unsigned getCurrentVersion();
	void setCurrentVersion(unsigned version);
	
	int getVersion();
	
	void resetMetadataForInterface(CommInterface *link);

	void statusMessage(int uasId, uint64_t totalSent, uint64_t totalReceived, uint64_t totalLoss, float lossPercent);

	int reserveMAVLinkChannel();
	void freeMAVLinkChannel(int channel);
	
	void suspendLogForReplay(bool suspend);
	void logMessage( mavlink_message_t* message );
	void deleteTempLogFiles();

protected:
	
	bool _closeLogFile();
	void _startLogging();
    void _stopLogging();

	void _loadSettings();
	void _storeSettings();
	
public slots:
	void checkForLostLogFiles();
	
protected slots:
	static void _aboutToQuit();
	void _vehicleCountChanged();
	
signals:
	void protocolStatusMessage(const QString& title, const QString& message);
	void mavlinkMessageStatus(int uasId, uint64_t totalSent, uint64_t totalReceived, uint64_t totalLoss, float lossPercent);
	void saveTelemetryLog(QString tempLogfile);
	void checkTelemetrySavePath(void);
	
protected:
	static MAVLinkProtocol* _instance;

    static const char*  _tempLogFileTemplate;    ///< Template for temporary log file
    static const char*  _logFileExtension;       ///< Extension for log files
	
    bool        _enableVersionCheck;                             ///< Enable checking of version match of MAV and QGC
    uint8_t     _lastIndex[256][256];                            ///< Store the last received sequence ID for each system/componenet pair
    uint8_t     _firstMessage[256][256];                         ///< First message flag
    uint64_t    _totalReceiveCounter[MAVLINK_COMM_NUM_BUFFERS];  ///< The total number of successfully received messages
    uint64_t    _totalLossCounter[MAVLINK_COMM_NUM_BUFFERS];     ///< Total messages lost during transmission.
    float       _runningLossPercent[MAVLINK_COMM_NUM_BUFFERS];   ///< Loss rate

	bool        _versionMismatchIgnore;
	int         _systemId;
	unsigned    _currentVersion;
	int         _radioVersionMismatchCount;
	uint32_t    _mavlinkChannelsUsedBitMask;

    bool _logSuspendError;      ///< true: Logging suspended due to error
    bool _logSuspendReplay;     ///< true: Logging suspended due to replay
    bool _vehicleWasArmed;      ///< true: Vehicle was armed during log sequence

    QGCTemporaryFile    _tempLogFile;            ///< File to log to
};
