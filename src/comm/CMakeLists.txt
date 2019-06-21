
set(EXTRA_SRC)
#if(BUILD_TESTING)
#	list(APPEND EXTRA_SRC
#		MockLink.cc
#		MockLinkFileServer.cc
#		MockLinkMissionItemHandler.cc
#	)
#endif()

## ROS2 STUFF ##
set( ROS2_LIB_NAMES
  rclcpp
  rcl
  rcutils
  std_msgs__rosidl_typesupport_cpp )

string(REPLACE ":" "/include;" ROS2_INCLUDE_PATHS $ENV{CMAKE_PREFIX_PATH} )
include_directories( ${ROS2_INCLUDE_PATHS} )

foreach( LIB ${ROS2_LIB_NAMES} )
  find_library( PATH_TO_${LIB} ${LIB} )
  if( NOT PATH_TO_${LIB} STREQUAL "PATH_TO_${LIB}-NOTFOUND" )
	set( ROS2_LIBS ${ROS2_LIBS} ${PATH_TO_${LIB}} )	
  else()
	set( ROS2_LIB_NOTFOUND "${ROS_LIB_NOTFOUND}\n\t${LIB}" )
  endif()
endforeach()

if( ROS2_LIB_NOTFOUND )
  message( WARNING "Failed to find ROS libraries ${ROS_LIB_NOTFOUND} \nMake sure to source \"local_setup.bash\" from your ROS2 workspace install directory" )
endif()
## END ROS2 STUFF ##

add_library(comm
  #BluetoothLink.cc
  CommInterface.cc
  CommInterfaceConfiguration.cc
  CommManager.cc
#	LinkConfiguration.cc
#	LinkInterface.cc
#	LinkManager.cc
#	LogReplayLink.cc
	MavlinkMessagesTimer.cc
	MAVLinkCommInterface.cc
#	MAVLinkProtocol.cc
	MAVLinkUDPCommInterface.cc
	QGCFlightGearLink.cc
	QGCJSBSimLink.cc
	QGCMAVLink.cc
	QGCSerialPortInfo.cc
	QGCXPlaneLink.cc
	ROS2CommInterface.cc
#	SerialLink.cc
#	TCPLink.cc
#	UDPLink.cc

	${EXTRA_SRC}

	# HEADERS
	# shouldn't be listed here, but aren't named properly for AUTOMOC
	QGCFlightGearLink.h
	QGCHilLink.h
	QGCJSBSimLink.h
)

target_link_libraries(comm
	PRIVATE
		qgc
	PUBLIC
		Qt5::Location
		Qt5::SerialPort
		Qt5::Test
		Qt5::TextToSpeech
		Qt5::Widgets
		${ROS2_LIBS}
)

target_include_directories(comm INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

