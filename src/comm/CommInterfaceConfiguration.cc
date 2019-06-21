#include <CommInterfaceConfiguration.h>
#include "MAVLinkUDPCommInterface.h"
#include "ROS2CommInterface.h"

const QString CommInterfaceConfiguration::settingsRoot = "LinkConfigurations";//"CommInterfaces";
CommInterfaceConfiguration::TypeMap CommInterfaceConfiguration::_typeMap;

CommInterfaceConfiguration::CommInterfaceConfiguration(const QString& name)
	: _interface(nullptr),
	  _name(name),
	  _dynamic(false),
	  _autoConnect(false),
	  _highLatency(false)
{
	if( _name.isEmpty() )
		qWarning() << "Internal error";
}

CommInterfaceConfiguration::CommInterfaceConfiguration(CommInterfaceConfiguration* copy)
{
	_interface = copy->_interface;
	_name = copy->_name;
	_dynamic = copy->_dynamic;
	_autoConnect = copy->_autoConnect;
	_highLatency = copy->_highLatency;
	Q_ASSERT(!_name.isEmpty());
}

CommInterfaceConfiguration::~CommInterfaceConfiguration()
{
}

QString CommInterfaceConfiguration::name(void) const
{
	return _name;
}

void CommInterfaceConfiguration::setName(const QString name)
{
	_name = name;
	emit nameChanged(_name);
}

void CommInterfaceConfiguration::fixName()
{
	if( _name != "Unnamed" )
		return;

	_name = "Unknown CommInterfaceConfiguration";
}

CommInterface* CommInterfaceConfiguration::commInterface(void)
{
	return _interface;
}

void CommInterfaceConfiguration::setCommInterface(CommInterface* interface)
{
	_interface = interface;
}

CommInterfaceConfiguration* CommInterfaceConfiguration::create(int type, QString name)
{
	switch( type )
	{
	case TypeSerial:
#ifdef NO_SERIAL_LINK
		qWarning() << "Serial Not Available";
#else
		//#TODO
#endif
		break;
	case TypeUdp:
		return new MAVLinkUDPCommInterfaceConfiguration(name);
		break;
	case TypeTcp:
		//#TODO
		break;
	case TypeBluetooth:
#ifdef QGC_ENABLE_BLUETOOTH
		//#TODO
#else
		qWarning() << "Bluetooth Not Available";
#endif
		break;
	case TypeMock:
#ifdef QT_DEBUG
		//#TODO
#else
		qWarning() << "Debug Not Available";
#endif
		break;
	case TypeLogReplay:
#ifdef __mobile__
		//#TODO
#else
		qWarning() << "Log Replay Not Available";
#endif
		break;
	case TypeROS2:
		return new ROS2CommInterfaceConfiguration(name);
		break;
	default:
		qWarning() << "Unknown Configuration Type";
		break;
	}

	printf("CommInterfaceConfiguration::create %i:\"%s\" - nullptr\n", type, name.toStdString().c_str());
	printf("\t%i - UDP\n",TypeUdp);
	printf("\t%i - ROS2\n",TypeROS2); 
	return nullptr;
}

CommInterfaceConfiguration* CommInterfaceConfiguration::create(QString type, QString name)
{
	typeMap();
	TypeMap::const_iterator i = _typeMap.find(type);
	if( i == _typeMap.constEnd() )
		return nullptr;
	return create( i.value(), name );
}

const CommInterfaceConfiguration::TypeMap& CommInterfaceConfiguration::typeMap()
{
	if( !_typeMap.size() )
	{
		_typeMap.insert( "UDP", TypeUdp );
		_typeMap.insert( "ROS2", TypeROS2 );
	}
	return _typeMap;
}

CommInterfaceConfiguration* CommInterfaceConfiguration::duplicate()
{
	return nullptr;
}

bool CommInterfaceConfiguration::isDynamic()
{
	return _dynamic;
}

bool CommInterfaceConfiguration::isAutoConnect()
{
	return _autoConnect;
}

bool CommInterfaceConfiguration::isHighLatency()
{
	return _highLatency;
}

void CommInterfaceConfiguration::setDynamic( bool dynamic )
{
	_dynamic = dynamic;
	emit dynamicChanged();
}

void CommInterfaceConfiguration::setAutoConnect( bool autoc )
{
	_autoConnect = autoc;
	emit autoConnectChanged();
}

void CommInterfaceConfiguration::setHighLatency( bool hl )
{
	_highLatency = hl;
	emit highLatencyChanged();
}

bool CommInterfaceConfiguration::isAutoConnectAllowed()
{
	return false;
}

bool CommInterfaceConfiguration::isHighLatencyAllowed()
{
	return false;
}

void CommInterfaceConfiguration::updateSettings()
{
}

void CommInterfaceConfiguration::copyFrom(CommInterfaceConfiguration* source)
{
    Q_ASSERT(source != NULL);
    _interface  = source->commInterface();
    _name       = source->name();
    _dynamic    = source->isDynamic();
    _autoConnect= source->isAutoConnect();
    _highLatency= source->isHighLatency();	
}

