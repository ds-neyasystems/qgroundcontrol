#include "ROS2CommInterface.h"
#include "QGCApplication.h"
#include "MultiVehicleManager.h"

#include <memory>

//ROS2 libraries
#include "rcutils/cmdline_parser.h"

using std::placeholders::_1;

//Static members
bool ROS2CommInterface::_ros2Initialized = false;

/* ****************************** *
 * ROS2CommInterfaceConfiguration
 * ****************************** */
ROS2CommInterfaceConfiguration::ROS2CommInterfaceConfiguration(const QString& name)  : CommInterfaceConfiguration(name)
{
}

ROS2CommInterfaceConfiguration::~ROS2CommInterfaceConfiguration()
{
}

CommInterfaceConfiguration::CommInterfaceType ROS2CommInterfaceConfiguration::type()
{
	return CommInterfaceConfiguration::TypeROS2;	
}

void ROS2CommInterfaceConfiguration::loadSettings(QSettings& settings, const QString& root)
{
}

void ROS2CommInterfaceConfiguration::saveSettings(QSettings& settings, const QString& root)
{
}

QString ROS2CommInterfaceConfiguration::settingsURL()
{
	return ("ROS2Settings.qml");
}

QString ROS2CommInterfaceConfiguration::settingsTitle()
{
	return tr("ROS2 Link Settings");
}

/* ********** *
 * ROS2Thread *
 * ********** */
ROS2Thread::ROS2Thread(std::shared_ptr<rclcpp::Node> node) :
	QThread(),
	_node(node),
	_executor(std::make_shared<rclcpp::executors::SingleThreadedExecutor>())
{
	_executor->add_node(_node);
}

ROS2Thread::~ROS2Thread()
{
	stop();
}

void ROS2Thread::stop()
{
	_executor->cancel();
}

void ROS2Thread::run()
{
	_executor->spin();
}

/* ******************** *
 * ROS2CommInterface *
 * ******************** */
ROS2CommInterface::ROS2CommInterface( CommInterfaceConfiguration::SharedPointer config, bool isPX4Flow ) :
	CommInterface(config, isPX4Flow),
	_gpsIntAvailable(false),
	_ros2Thread(),
	_vehicleIDMap(),
	_globalWaypointCommandPub(nullptr),
	_globalPoseSub(nullptr)
{
	_initializeROS2();

	_node = std::make_shared<rclcpp::Node>("QGroundControl");

	_initializePublishers();
	_initializeSubscribers();
	
	moveToThread(this);

	_ros2Thread = new ROS2Thread(_node);
	_ros2Thread->start();
}

ROS2CommInterface::~ROS2CommInterface()
{
   _disconnect();
    quit();
	delete _ros2Thread;
    // Wait for it to exit
    wait();
    this->deleteLater();		
}

void ROS2CommInterface::_initializeROS2()
{
	if(_ros2Initialized)
		return;
	
	const char* argv = "QGroundControl";
	int argc = 1;
	
	rclcpp::init(argc, &argv);

	_ros2Initialized = true;
}

bool ROS2CommInterface::_connect(void)
{
    if(this->isRunning())
    {
        quit();
        wait();
    }

    start(NormalPriority);
    return true;		
}

void ROS2CommInterface::_disconnect(void)
{
	_ros2Thread->stop();
}
   
bool ROS2CommInterface::isConnected() const
{
	return true;
}

void ROS2CommInterface::resetMetaData()
{
	
}

void ROS2CommInterface::requestReset()
{
	
}

void ROS2CommInterface::slotCommandInt( uint8_t target_system, uint8_t target_component, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z)
{
	
}

void ROS2CommInterface::slotCommandLong( uint8_t target_system, uint8_t target_component, uint16_t command, uint8_t confirmation, float param1, float param2, float param3, float param4, float param5, float param6, float param7 )
{
	
}

void ROS2CommInterface::slotCommandAck( uint16_t command, uint8_t result, uint8_t progress, int32_t result_param2, uint8_t target_system, uint8_t target_component )
{
	
}

void ROS2CommInterface::slotDataTransmissionHandshake( uint8_t type,uint32_t size,uint16_t width,uint16_t height,uint16_t packets,uint8_t payload,uint8_t jpg_quality)
{
	
}
	  
void ROS2CommInterface::slotFollowTarget( int latitude, int longitude, float altitude, float velocity[3], float acceleration[3], float attitude_q[4], float rates[3], float position_cov[3], uint8_t est_capabilities )
{
	
}

void ROS2CommInterface::slotFileTransfer( uint8_t target_system, QByteArray payload )
{
	
}

void ROS2CommInterface::slotGPSRTCM( QByteArray rtcm_data, uint8_t sequence_id )
{
	
}

void ROS2CommInterface::slotHILGPS( uint8_t fix_type,int32_t lat,int32_t lon,int32_t alt,uint16_t eph,uint16_t epv,uint16_t vel,int16_t vn,int16_t ve,int16_t vd,uint16_t cog,uint8_t satellites_visible )
{
	
}

void ROS2CommInterface::slotHILSensor( float xacc,float yacc,float zacc,float xgyro,float ygyro,float zgyro,float xmag,float ymag,float zmag,float abs_pressure,float diff_pressure,float pressure_alt,float temperature,uint32_t fields_updated )
{
	
}

void ROS2CommInterface::slotHILStateQuaternion( float attitude_quaternion[4], float rollspeed, float pitchspeed, float yawspeed, int32_t lat, int32_t lon, int32_t alt, int16_t vx, int16_t vy, int16_t vz, uint16_t ind_airspeed, uint16_t true_airspeed, int16_t xacc, int16_t yacc, int16_t zacc)
{
	
}

void ROS2CommInterface::slotHeartbeat()
{
}

void ROS2CommInterface::slotLogErase( uint8_t target_system, uint8_t target_component )
{
	
}

void ROS2CommInterface::slotLogRequestData( uint8_t target_system, uint8_t target_component, uint16_t id, uint32_t offset, uint32_t count )
{
}

void ROS2CommInterface::slotLogRequestList( uint8_t target_system, uint8_t target_component, uint32_t start, uint32_t end)
{
}

void ROS2CommInterface::slotLoggingAck( uint8_t target_system, uint8_t target_component, uint16_t seq )
{
}

void ROS2CommInterface::slotManualControl( uint8_t target_system, float pitch, float roll, float thrust, float yaw, uint16_t buttons )
{
}

void ROS2CommInterface::slotMissionAck(uint8_t target_system,uint8_t target_component,uint8_t type,uint8_t mission_type)
{
}

void ROS2CommInterface::slotMissionAllClear( uint8_t target_system, uint8_t target_component, uint8_t mission_type )
{
}

void ROS2CommInterface::slotMissionCount( uint8_t target_system, uint8_t target_component, uint16_t count, uint8_t mission_type )
{
}

void ROS2CommInterface::slotMissionItem( uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current, uint8_t autocontinue, float param1, float param2, float param3, float param4, float x, float y, float z, uint8_t mission_type)
{
}

void ROS2CommInterface::slotMissionItemInt( uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t frame,uint16_t command,uint8_t current,uint8_t autocontinue,float param1,float param2,float param3,float param4,int32_t x,int32_t y,float z,uint8_t mission_type)
{
}

void ROS2CommInterface::slotMissionRequest(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type)
{
}

void ROS2CommInterface::slotMissionRequestInt(uint8_t target_system,uint8_t target_component,uint16_t seq,uint8_t mission_type)
{
}

void ROS2CommInterface::slotMissionRequestList(uint8_t target_system,uint8_t target_component,uint8_t mission_type)
{
}

void ROS2CommInterface::slotMissionSetCurrentSequence( uint16_t seq )
{
}

void ROS2CommInterface::slotMode(uint8_t target_system,uint8_t base_mode,uint32_t custom_mode)
{
}

void ROS2CommInterface::slotParameterExtRequestList( uint8_t target_system, uint8_t target_component )
{
}

void ROS2CommInterface::slotParameterExtRequestRead( uint8_t target_system, uint8_t target_component, QString param_id )
{
}

void ROS2CommInterface::slotParameterExtSet( uint8_t target_system, uint8_t target_component, QString param_id, uint8_t param_value[128],  uint8_t param_type )
{
}

void ROS2CommInterface::slotParameterMapRC( uint8_t target_system,uint8_t target_component, QString param_id,int16_t param_index,uint8_t parameter_rc_channel_index,float param_value0,float scale,float param_value_min,float param_value_max )
{
}

void ROS2CommInterface::slotParameterRequestList( uint8_t target_system, uint8_t target_component )
{
}

void ROS2CommInterface::slotParameterRequestRead( uint8_t target_system, uint8_t target_component, QString paramName, int paramIndex )
{
}

void ROS2CommInterface::slotParameterSet( uint8_t target_system, uint8_t target_component, QVariant parameter_value, QString parameter_id, FactMetaData::ValueType_t type )
{
}

void ROS2CommInterface::slotROS2GlobalWaypointCommand( int target_system, int target_component, double latitude, double longitude, double altitude )
{
	utc_msgs::msg::GlobalWaypointCommandType msg, upload_msg, start_msg;
	std::string target_system_id;

	msg.position.geodetic_latitude.latitude = latitude;
	msg.position.geodetic_longitude.longitude = longitude;
	msg.depth = 1.0;

	if( !_vehicleIDMap.find( target_system, target_system_id ) )
		return;

	//#TODO figure out source id and session id
	msg.session_id = "??";
	msg.source_subsystem_id = "QGroundControl Component";
	msg.source_system_id  = "QGroundControl";
	msg.target_system_id = target_system_id;
	_globalWaypointCommandPub->publish(msg);

	//#HACK upload mission
	upload_msg.target_subsystem_id = "1";
	upload_msg.target_system_id = target_system_id;
	_globalWaypointCommandPub->publish(upload_msg);

	//#HACK run mission
	start_msg.source_subsystem_id = "1";
	start_msg.target_system_id = target_system_id;
	_globalWaypointCommandPub->publish(start_msg);
}

void ROS2CommInterface::slotSetAttitudeTarget(uint8_t target_system,uint8_t target_component,uint8_t type_mask,const float attitude_quaternion[4],float body_roll_rate,float body_pitch_rate,float body_yaw_rate,float thrust)
{
}

void ROS2CommInterface::slotSetPositionTargetLocalNED( uint8_t target_system, uint8_t target_component, float position[3], float velocity[3], float acceleration[3], float yaw, float yaw_rate, uint16_t type_mask, uint8_t coordinate_frame)
{
}

void ROS2CommInterface::slotSystemTime(uint64_t time_unix_usec,uint32_t time_boot_ms)
{
}

void ROS2CommInterface::_initializePublishers()
{
	_globalWaypointCommandPub = _node->create_publisher<utc_msgs::msg::GlobalWaypointCommandType>("global_waypoint_item"); //#TODO: get topic
}

void ROS2CommInterface::_initializeSubscribers()
{
	_globalPoseSub = _node->create_subscription<utc_msgs::msg::GlobalPoseStatusType>("global_pose_sensor_report_global_pose", std::bind(&ROS2CommInterface::_handleGlobalPose, this, _1));
}

Vehicle* ROS2CommInterface::_getVehicle(int system_id)
{
	MultiVehicleManager* vehicle_manager;
	vehicle_manager = qgcApp()->toolbox()->multiVehicleManager();

	if( vehicle_manager == nullptr )
		return nullptr;

	return vehicle_manager->getVehicleById(system_id);
}

int ROS2CommInterface::_parseSystemID(const std::string& id)
{
	static int new_id_start = std::numeric_limits<unsigned short>::max() + 1;
	int vehicle_id;

	//If the id is already in the map, use it
	if( _vehicleIDMap.find(id, vehicle_id) )
		return vehicle_id;

	//If the id is not already in the map, use the next available number and add it to the map
	vehicle_id = new_id_start;
	_vehicleIDMap.insert( id, vehicle_id );
	new_id_start++;

	return vehicle_id;
}

int ROS2CommInterface::_parseComponentID(const std::string& id)
{
	//#PLACEHOLDER
	return MAV_COMP_ID_AUTOPILOT1;
}

void ROS2CommInterface::_handleGlobalPose(const utc_msgs::msg::GlobalPoseStatusType::SharedPtr msg)
{
	Vehicle *vehicle;
	int system_id;
	int component_id;
	double latitude, longitude, altitude, yaw;

	//Get id of vehicle
	system_id = _parseSystemID(msg->source_system_id);
	component_id = _parseComponentID(msg->source_subsystem_id);

	//Get vehicle from id
	vehicle = _getVehicle(system_id);

	//Emit hearbeat info to initialize vehicle in vehicle manager
	emit vehicleHeartbeatInfo(this, system_id, component_id, MAV_AUTOPILOT_RESERVED, MAV_TYPE_QUADROTOR );

	//If we don't have a valid vehicle, then ignore the rest of the message
	if( !vehicle )
		return;

	//Get lat/lon
	latitude = msg->position.geodetic_position.geodetic_latitude.latitude;
	longitude = msg->position.geodetic_position.geodetic_longitude.longitude;

	altitude = msg->position.height_above_ellipsoid.altitude;

	yaw = msg->heading;

	//Reset activity timeout timer
	startActivityTimer(system_id);

	//Emit gps
	emit receivedGPS( vehicle,
					  latitude,
					  longitude,
					  altitude );

	//Emit altitude
	emit receivedAltitude( vehicle, altitude, altitude );

	//Emit heading
	emit receivedAttitude( vehicle, 0.0, 0.0, yaw );

/*
	emit receivedVelocity( vehicle,
						   global_position_int.vx / 100.0,
						   global_position_int.vy / 100.0,
						   global_position_int.vz / 100.0 );
*/

	//#TODO get real status
	//Emit status
	emit receivedExtendedSysState( vehicle, MAV_LANDED_STATE_IN_AIR, MAV_VTOL_STATE_MC );
	emit receivedHeartbeat( vehicle, MAV_MODE_FLAG_DECODE_POSITION_SAFETY, 0 ); //#TODO figure out better arm status reporting
}
