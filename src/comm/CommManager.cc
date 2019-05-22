#include "CommManager.h"
#include "QGCApplication.h"
#include "SettingsManager.h"
#include "MAVLinkUDPCommInterface.h"
#include "QGCSerialPortInfo.h"

QGC_LOGGING_CATEGORY(LinkManagerLog, "LinkManagerLog")
QGC_LOGGING_CATEGORY(LinkManagerVerboseLog, "LinkManagerVerboseLog")

//Static members
CommManager* CommManager::_commManager                  = NULL;
const char*  CommManager::_defaultUDPLinkName           = "UDP Link (AutoConnect)";;
const int    CommManager::_autoconnectUpdateTimerMSecs  = 1000;;
const int    CommManager::_activeLinkCheckTimeoutMSecs  = 15000;   ///< Amount of time to wait for a heatbeat. Keep in mind ArduPilot stack heartbeat is slow to come.

#ifdef Q_OS_WIN
const int    CommManager::_autoconnectConnectDelayMSecs = 6000;
#else
const int    CommManager::_autoconnectConnectDelayMSecs = 1000;
#endif


CommManager::CommManager() :
	_interfaceList(),
	_sharedConfigurationList(),
	_sharedAutoconnectConfigurationList(),
	_qmlConfigurations(),
	_interfaceListMutex(),
	_timer(),
	_initialized(false),
	_autoConnectSettings(nullptr),
	_configUpdateSuspended(false),
    _configurationsLoaded(false),
    _connectionsSuspended(false),
	_connectionsSuspendedReason(),
	_autoconnectTimer()
{
	_timer.start();
    qmlRegisterUncreatableType<CommManager>                  ("QGroundControl", 1, 0, "LinkManager",         "Reference only");
    qmlRegisterUncreatableType<CommInterfaceConfiguration>   ("QGroundControl", 1, 0, "LinkConfiguration",   "Reference only");
    qmlRegisterUncreatableType<CommInterface>                ("QGroundControl", 1, 0, "LinkInterface",       "Reference only");
}

CommManager::~CommManager()
{
}

CommManager* CommManager::getCommManager()
{
	if( _commManager == NULL )
	{
		_commManager = new CommManager();		
	}
	return _commManager;
}

CommInterfaceConfiguration* CommManager::createConfiguration(int type, const QString& name)
{
	return CommInterfaceConfiguration::create(type, name);
}

CommInterfaceConfiguration* CommManager::startConfigurationEditing(CommInterfaceConfiguration* config)
{
	if( !config )
	{
		qWarning() << "Internal error";
        return nullptr;
	}

	return config->duplicate();
}

void CommManager::cancelConfigurationEditing(CommInterfaceConfiguration* config)
{
    delete config;
}

bool CommManager::endConfigurationEditing(CommInterfaceConfiguration* config, CommInterfaceConfiguration* editedConfig)
{
   if ( !config || !editedConfig)
   {
	    qWarning() << "Internal error";
		return true;
   }
   
   editedConfig->fixName();
   config->copyFrom(editedConfig);
   saveCommConfigurationList();

   // Tell link about changes (if any)
   config->updateSettings();
   
   // Discard temporary duplicate
   delete editedConfig;

   return true;	
}

bool CommManager::endCreateConfiguration(CommInterfaceConfiguration* config)
{
    if (!config)
	{
		qWarning() << "Internal error";
		return true;	
	}

	config->fixName();
	addConfiguration(config);
	saveCommConfigurationList();

    return true;	
}

void CommManager::removeConfiguration(CommInterfaceConfiguration* config)
{
	CommInterface* interface;
	
	if( !config )
	{
		qWarning() << "Internal error";
		return;
	}

	interface = config->commInterface();
	if( interface )
		disconnectLink(interface);

	_removeConfiguration(config);
	saveCommConfigurationList();	
}

void CommManager::createConnectedLink(CommInterfaceConfiguration* config)
{
   for(int i = 0; i < _sharedConfigurationList.count(); i++)
   {
	   CommInterfaceConfiguration::SharedPointer& shared_conf = _sharedConfigurationList[i];
	   if (shared_conf->name() == config->name())
		   createConnectedCommInterface(shared_conf);
    }	
}

void CommManager::disconnectLink(CommInterface* interface)
{
   if (!interface || !containsInterface(interface))
        return;    

    interface->_disconnect();

    CommInterfaceConfiguration* config = interface->getConfiguration().data();
    for (int i=0; i<_sharedAutoconnectConfigurationList.count(); i++)
	{
        if (_sharedAutoconnectConfigurationList[i].data() == config)
		{
            qCDebug(LinkManagerLog) << "Removing disconnected autoconnect config" << config->name();
            _sharedAutoconnectConfigurationList.removeAt(i);
            break;
        }
    }

    _deleteInterface(interface);
}

void CommManager::shutdown()
{
	setConnectionsSuspended(tr("Shutdown"));
	disconnectAll();	
}

void CommManager::initialize()
{
	if( _initialized )
		return;

	_autoConnectSettings = qgcApp()->toolbox()->settingsManager()->autoConnectSettings();

	 connect(&_autoconnectTimer, &QTimer::timeout, this, &CommManager::_updateAutoConnectLinks);
    _autoconnectTimer.start(_autoconnectUpdateTimerMSecs);
	
	_initialized = true;
}

QList<CommInterface*> CommManager::interfaces()
{
	QList<CommInterface*> raw_interface_list;
	for(int i = 0; i < _interfaceList.count(); i++)
		raw_interface_list.append( _interfaceList[i].data() );
	return raw_interface_list;
}

QStringList CommManager::commTypeStrings() const
{
    static QStringList list;
    if(!list.size())
    {
#ifndef NO_SERIAL_LINK
        list += tr("Serial");
#endif
        list += tr("UDP");
        list += tr("TCP");
#ifdef QGC_ENABLE_BLUETOOTH
        list += tr("Bluetooth");
#endif
#ifdef QT_DEBUG
        list += tr("Mock Link");
#endif
#ifndef __mobile__
        list += tr("Log Replay");
#endif
        if (list.size() != static_cast<int>(CommInterfaceConfiguration::TypeLast)) {
            qWarning() << "Internal error";
        }
    }
    return list;	
}

void CommManager::loadCommConfigurationList()
{
	bool links_changed = false;
    QSettings settings;
	int count;
	CommInterfaceConfiguration::CommInterfaceType type;
	QString name;
	CommInterfaceConfiguration* new_config;

printf("\n\nCommManager::loadCommConfigurationList()\n");
	if( !settings.contains(CommInterfaceConfiguration::settingsRoot + "/count") )
		return;
	
	count = settings.value(CommInterfaceConfiguration::settingsRoot + "/count").toInt();
printf("CommManager::loadCommConfigurationList() - %i configurations\n", count);
	for(int i = 0; i < count; i++)
	{
		QString root(CommInterfaceConfiguration::settingsRoot);
		root += QString("/Link%1").arg(i);

		//Get type
		if( !settings.contains(root + "/type") )
		{
			qWarning() << "Link Configuration" << root << "has no type." ;
			continue;
		}
		
		type = static_cast<CommInterfaceConfiguration::CommInterfaceType>(settings.value(root + "/type").toInt());

		if(type >= CommInterfaceConfiguration::TypeLast)
		{
			qWarning() << "Link Configuration" << root << "an invalid type: " << type;
			continue;
		}

		//Get name
		if( !settings.contains(root + "/name") )
		{
			qWarning() << "Link Configuration" << root << "has no name." ;
			continue;
		}
		
		name = settings.value(root + "/name").toString();
		if( name.isEmpty() )
		{
			qWarning() << "Link Configuration" << root << "has an empty name." ;
			continue;
		}

		//Initialize new comm interface configuration
		new_config = CommInterfaceConfiguration::create(type, name);

		if( !new_config )
		{
			qWarning() << "Link Configuration" << root << "has unsupported type: " << type;
			continue;
		}

		new_config->setAutoConnect( settings.value(root + "/auto").toBool() );
		new_config->setHighLatency( settings.value(root + "/high_latency").toBool() );
		new_config->loadSettings(settings, root);
printf("\t===\nCommManager new config: \"%s\" - %i\n", new_config->name().toStdString().c_str(), new_config->isAutoConnect());
		//Add new configuration
		addConfiguration(new_config);

		//Set links changed flag
		links_changed = true;
	}

	if(links_changed) 
        emit linkConfigurationsChanged();
}

void CommManager::saveCommConfigurationList()
{
    QSettings settings;
    int true_count;
	CommInterfaceConfiguration::SharedPointer config;
	QString root;
	
    settings.remove(CommInterfaceConfiguration::settingsRoot);
	
	true_count = 0;
    for (int i = 0; i < _sharedConfigurationList.count(); i++)
	{
        config = _sharedConfigurationList[i];
        if( !config )			
		{
			qWarning() << "Internal error for link configuration in LinkManager";
			continue;
		}
			
		if (config->isDynamic())
			continue;
			
		root = CommInterfaceConfiguration::settingsRoot;
		root += QString("/Link%1").arg(true_count++);
		settings.setValue(root + "/name", config->name());
		settings.setValue(root + "/type", config->type());
		settings.setValue(root + "/auto", config->isAutoConnect());
		settings.setValue(root + "/high_latency", config->isHighLatency());

		// Have the instance save its own values
		config->saveSettings(settings, root);
    }
	
    root = CommInterfaceConfiguration::settingsRoot;

    settings.setValue(root + "/count", true_count);

    emit linkConfigurationsChanged();	
}

void CommManager::suspendConfigurationUpdates(bool suspend)
{
	_configUpdateSuspended = suspend;
}

void CommManager::setConnectionsSuspended(QString reason)
{
	_connectionsSuspended = true;
	_connectionsSuspendedReason = reason;	
}

void CommManager::setConnectionsAllowed()
{
	_connectionsSuspended = false;
}	

CommInterface* CommManager::createConnectedCommInterface(CommInterfaceConfiguration::SharedPointer& config, bool isPX4Flow )
{
	CommInterface* interface;
	
//    Q_UNUSED(isPX4Flow)
    if (!config)
	{
        qWarning() << "CommInterfaceManager::createConnectedLink called with nullptr config";
        return nullptr;
    }

    interface = nullptr;
    switch(config->type())
	{
    case CommInterfaceConfiguration::TypeSerial:
#ifdef NO_SERIAL_LINK
#else
		//#TODO
#endif
        break;
    case CommInterfaceConfiguration::TypeUdp:
        interface = new MAVLinkUDPCommInterface(config);
        break;
    case CommInterfaceConfiguration::TypeTcp:
		//#TODO
        break;

    case CommInterfaceConfiguration::TypeBluetooth:
#ifdef QGC_ENABLE_BLUETOOTH
		//#TODO

#endif
		break;
    case CommInterfaceConfiguration::TypeLogReplay:
#ifndef __mobile__
		//#TODO
#endif
		break;
    case CommInterfaceConfiguration::TypeMock:
#ifdef QT_DEBUG
		//#TODO
#endif
		break;
	default:
		break;
    }
	
    if (interface)
        _addInterface(interface);

    return interface;
}

CommInterface* CommManager::createConnectedCommInterface(const QString& name)
{	
   if (name.isEmpty())
   {
        qWarning() << "Internal error";
		return nullptr;
   }
   
   for(int i = 0; i < _sharedConfigurationList.count(); i++)
   {
	   CommInterfaceConfiguration::SharedPointer& conf = _sharedConfigurationList[i];
	   if (conf->name() == name)
		   return createConnectedCommInterface(conf);
   }
   
   return nullptr;	
}

void CommManager::disconnectAll()
{
    for( int i = _interfaceList.count() - 1; i >= 0; i-- )
        disconnectLink(_interfaceList[i].data());
}


bool CommManager::isAutoconnectLink(CommInterface* interface)
{
	for (int i = 0; i < _sharedAutoconnectConfigurationList.count(); i++)
	{
		if( _sharedAutoconnectConfigurationList[i].data() == interface->getConfiguration() ) 
            return true;
    }
    return false;	
}

uint64_t CommManager::runtime()
{
	return _timer.nsecsElapsed() * 1e-6;
}

bool CommManager::containsInterface(CommInterface* interface)
{
	for (int i=0; i < _interfaceList.count(); i++)
        if (_interfaceList[i].data() == interface)
            return true;

    return false;
}

SharedCommInterfacePointer CommManager::findInterface(CommInterface* interface)
{
	for (int i=0; i < _interfaceList.count(); i++)
        if (_interfaceList[i].data() == interface)
            return _interfaceList[i];
	
	return SharedCommInterfacePointer(nullptr);
}

CommInterfaceConfiguration::SharedPointer CommManager::addConfiguration(CommInterfaceConfiguration* config)
{
    _qmlConfigurations.append(config);
    _sharedConfigurationList.append(CommInterfaceConfiguration::SharedPointer(config));
    return _sharedConfigurationList.last();
}

void CommManager::startAutoConnectedLinks()
{
	printf("CommManager::startAutoConnectedLinks() - %i configurations\n", _sharedConfigurationList.count());
	CommInterfaceConfiguration::SharedPointer conf;
	for(int i = 0; i < _sharedConfigurationList.count(); i++)
	{
        conf = _sharedConfigurationList[i];
        if (conf->isAutoConnect())
            createConnectedCommInterface(conf);
	}
}

void CommManager::sendGCSHeartbeat()
{
	for( SharedCommInterfacePointer interface : _interfaceList )
	{
		if( !interface->isConnected() || interface->getHighLatency() )
			continue;
		interface->sendHeartbeat();
	}
}

void CommManager::_deleteInterface(CommInterface* interface)
{
	if (thread() != QThread::currentThread())
	{
		qWarning() << "_deleteLink called from incorrect thread";
		return;
	}
	
    if (!interface)
        return;

    // Free up the mavlink channel associated with this link
//    _freeMavlinkChannel(link->mavlinkChannel());

    for (int i = 0; i < _interfaceList.count(); i++)
	{
        if (_interfaceList[i].data() == interface)
		{
            _interfaceList.removeAt(i);
            break;
        }
    }

    // Emit removal of link
//    emit linkDeleted(interface);	
}

void CommManager::_addInterface(CommInterface* interface)
{
//	_add
	//Ensure _addInterface is called on correct thread
	if( thread() != QThread::currentThread() )
	{
        qWarning() << "_addInterface called from incorrect thread";
        return;
    }

	//Ensure interface being added is valid
    if( !interface )
        return;

	//Add new interface to list
    if(!containsInterface(interface))
	{
/*		
        int mavlinkChannel = _reserveMavlinkChannel();
        if (mavlinkChannel != 0) {
            interface->_setMavlinkChannel(mavlinkChannel);
        } else {
            qWarning() << "Ran out of mavlink channels";
            return;
        }
*/
        _interfaceList.append(SharedCommInterfacePointer(interface));
        emit newInterface(interface);
    }

//    connect(link, &LinkInterface::communicationError,   _app,               &QGCApplication::criticalMessageBoxOnMainThread);
//    connect(link, &LinkInterface::bytesReceived,        _mavlinkProtocol,   &MAVLinkProtocol::receiveBytes);

//    _mavlinkProtocol->resetMetadataForLink(link);
//    _mavlinkProtocol->setVersion(_mavlinkProtocol->getCurrentVersion());

//    connect(interface, &CommInterface::connected,            this, &CommManager::_linkConnected);
//    connect(interface, &CommInterface::disconnected,         this, &CommManager::_linkDisconnected);

    // This connection is queued since it will cloe the link. So we want the link emitter to return otherwise we would
    // close the link our from under itself.
    connect(interface, &CommInterface::connectionRemoved,    this, &CommManager::_linkConnectionRemoved, Qt::QueuedConnection);
//	connect
   if (interface)
   {
	   if (_connectionsSuspendedMsg())
		   return;
	   interface->_connect();
   }
   else
   {
	   qWarning() << "Internal error";
   }
}

QmlObjectListModel* CommManager::_qmlLinkConfigurations()
{
	return &_qmlConfigurations;
}

bool CommManager::_connectionsSuspendedMsg(void)
{
    if (_connectionsSuspended)
	{
        qgcApp()->showMessage(tr("Connect not allowed: %1").arg(_connectionsSuspendedReason));
        return true;
    } else
        return false;
}

void CommManager::_updateAutoConnectLinks(void)
{
   if (_connectionsSuspended || qgcApp()->runningUnitTests())
        return;

    // Re-add UDP if we need to
    bool foundUDP = false;
    for (int i = 0; i < _interfaceList.count(); i++)
	{
        CommInterfaceConfiguration* config = _interfaceList[i]->getConfiguration().data();
        if (config->type() == CommInterfaceConfiguration::TypeUdp && config->name() == _defaultUDPLinkName)
		{
            foundUDP = true;
            break;
        }
    }
    if (!foundUDP && _autoConnectSettings->autoConnectUDP()->rawValue().toBool())
	{
        qCDebug(LinkManagerLog) << "New auto-connect UDP port added";
        //-- Default UDPConfiguration is set up for autoconnect
        MAVLinkUDPCommInterfaceConfiguration* udp_config = new MAVLinkUDPCommInterfaceConfiguration(_defaultUDPLinkName);
        udp_config->setDynamic(true);
		CommInterfaceConfiguration::SharedPointer config = addConfiguration(udp_config);
        createConnectedCommInterface(config);
        emit linkConfigurationsChanged();
    }
/*
#ifndef NO_SERIAL_LINK
    QStringList currentPorts;
    QList<QGCSerialPortInfo> portList;
#ifdef __android__
    // Android builds only support a single serial connection. Repeatedly calling availablePorts after that one serial
    // port is connected leaks file handles due to a bug somewhere in android serial code. In order to work around that
    // bug after we connect the first serial port we stop probing for additional ports.
    if (!_sharedAutoconnectConfigurationList.count()) {
        portList = QGCSerialPortInfo::availablePorts();
    }
#else
    portList = QGCSerialPortInfo::availablePorts();
#endif //__android__

    // Iterate Comm Ports
    for (QGCSerialPortInfo portInfo: portList) {
        qCDebug(LinkManagerVerboseLog) << "-----------------------------------------------------";
        qCDebug(LinkManagerVerboseLog) << "portName:          " << portInfo.portName();
        qCDebug(LinkManagerVerboseLog) << "systemLocation:    " << portInfo.systemLocation();
        qCDebug(LinkManagerVerboseLog) << "description:       " << portInfo.description();
        qCDebug(LinkManagerVerboseLog) << "manufacturer:      " << portInfo.manufacturer();
        qCDebug(LinkManagerVerboseLog) << "serialNumber:      " << portInfo.serialNumber();
        qCDebug(LinkManagerVerboseLog) << "vendorIdentifier:  " << portInfo.vendorIdentifier();
        qCDebug(LinkManagerVerboseLog) << "productIdentifier: " << portInfo.productIdentifier();

        // Save port name
        currentPorts << portInfo.systemLocation();

        QGCSerialPortInfo::BoardType_t boardType;
        QString boardName;

#ifndef NO_SERIAL_LINK
#ifndef __mobile__
        if (portInfo.systemLocation().trimmed() == _autoConnectSettings->autoConnectNmeaPort()->cookedValueString()) {
            if (portInfo.systemLocation().trimmed() != _nmeaDeviceName) {
                _nmeaDeviceName = portInfo.systemLocation().trimmed();
                qCDebug(LinkManagerLog) << "Configuring nmea port" << _nmeaDeviceName;
                QSerialPort* newPort = new QSerialPort(portInfo);
                _nmeaBaud = _autoConnectSettings->autoConnectNmeaBaud()->cookedValue().toUInt();
                newPort->setBaudRate(static_cast<qint32>(_nmeaBaud));
                qCDebug(LinkManagerLog) << "Configuring nmea baudrate" << _nmeaBaud;
                // This will stop polling old device if previously set
                _toolbox->qgcPositionManager()->setNmeaSourceDevice(newPort);
                if (_nmeaPort) {
                    delete _nmeaPort;
                }
                _nmeaPort = newPort;
            } else if (_autoConnectSettings->autoConnectNmeaBaud()->cookedValue().toUInt() != _nmeaBaud) {
                _nmeaBaud = _autoConnectSettings->autoConnectNmeaBaud()->cookedValue().toUInt();
                _nmeaPort->setBaudRate(static_cast<qint32>(_nmeaBaud));
                qCDebug(LinkManagerLog) << "Configuring nmea baudrate" << _nmeaBaud;
            }
        } else
#endif //__mobile__
#endif //NO_SERIAL_LINK

        if (portInfo.getBoardInfo(boardType, boardName)) {
            if (portInfo.isBootloader()) {
                // Don't connect to bootloader
                qCDebug(LinkManagerLog) << "Waiting for bootloader to finish" << portInfo.systemLocation();
                continue;
            }
            if (_autoconnectConfigurationsContainsPort(portInfo.systemLocation()) || _autoConnectRTKPort == portInfo.systemLocation()) {
                qCDebug(LinkManagerVerboseLog) << "Skipping existing autoconnect" << portInfo.systemLocation();
            } else if (!_autoconnectWaitList.contains(portInfo.systemLocation())) {
                // We don't connect to the port the first time we see it. The ability to correctly detect whether we
                // are in the bootloader is flaky from a cross-platform standpoint. So by putting it on a wait list
                // and only connect on the second pass we leave enough time for the board to boot up.
                qCDebug(LinkManagerLog) << "Waiting for next autoconnect pass" << portInfo.systemLocation();
                _autoconnectWaitList[portInfo.systemLocation()] = 1;
            } else if (++_autoconnectWaitList[portInfo.systemLocation()] * _autoconnectUpdateTimerMSecs > _autoconnectConnectDelayMSecs) {
                SerialConfiguration* pSerialConfig = nullptr;
                _autoconnectWaitList.remove(portInfo.systemLocation());
                switch (boardType) {
                case QGCSerialPortInfo::BoardTypePixhawk:
                    if (_autoConnectSettings->autoConnectPixhawk()->rawValue().toBool()) {
                        pSerialConfig = new SerialConfiguration(tr("%1 on %2 (AutoConnect)").arg(boardName).arg(portInfo.portName().trimmed()));
                        pSerialConfig->setUsbDirect(true);
                    }
                    break;
                case QGCSerialPortInfo::BoardTypePX4Flow:
                    if (_autoConnectSettings->autoConnectPX4Flow()->rawValue().toBool()) {
                        pSerialConfig = new SerialConfiguration(tr("%1 on %2 (AutoConnect)").arg(boardName).arg(portInfo.portName().trimmed()));
                    }
                    break;
                case QGCSerialPortInfo::BoardTypeSiKRadio:
                    if (_autoConnectSettings->autoConnectSiKRadio()->rawValue().toBool()) {
                        pSerialConfig = new SerialConfiguration(tr("%1 on %2 (AutoConnect)").arg(boardName).arg(portInfo.portName().trimmed()));
                    }
                    break;
                case QGCSerialPortInfo::BoardTypeOpenPilot:
                    if (_autoConnectSettings->autoConnectLibrePilot()->rawValue().toBool()) {
                        pSerialConfig = new SerialConfiguration(tr("%1 on %2 (AutoConnect)").arg(boardName).arg(portInfo.portName().trimmed()));
                    }
                    break;
					
#ifndef __mobile__
                case QGCSerialPortInfo::BoardTypeRTKGPS:
                    if (_autoConnectSettings->autoConnectRTKGPS()->rawValue().toBool() && !_toolbox->gpsManager()->connected()) {
                        qCDebug(LinkManagerLog) << "RTK GPS auto-connected" << portInfo.portName().trimmed();
                        _autoConnectRTKPort = portInfo.systemLocation();
                        _toolbox->gpsManager()->connectGPS(portInfo.systemLocation(), boardName);
                    }
                    break;
#endif

                default:
                    qWarning() << "Internal error";
                    continue;
                }
                if (pSerialConfig) {
                    qCDebug(LinkManagerLog) << "New auto-connect port added: " << pSerialConfig->name() << portInfo.systemLocation();
                    pSerialConfig->setBaud(boardType == QGCSerialPortInfo::BoardTypeSiKRadio ? 57600 : 115200);
                    pSerialConfig->setDynamic(true);
                    pSerialConfig->setPortName(portInfo.systemLocation());
                    _sharedAutoconnectConfigurationList.append(CommInterfaceConfiguration::SharedPointer(pSerialConfig));
                    createConnectedLink(_sharedAutoconnectConfigurationList.last(), boardType == QGCSerialPortInfo::BoardTypePX4Flow);
                }
            }
        }
    }

#ifndef __android__
    // Android builds only support a single serial connection. Repeatedly calling availablePorts after that one serial
    // port is connected leaks file handles due to a bug somewhere in android serial code. In order to work around that
    // bug after we connect the first serial port we stop probing for additional ports. That means we must rely on
    // the port disconnecting itself when the radio is pulled to signal communication list as opposed to automatically
    // closing the Link.

    // Now we go through the current configuration list and make sure any dynamic config has gone away
    QList<CommInterfaceConfiguration*>  _confToDelete;
    for (int i=0; i<_sharedAutoconnectConfigurationList.count(); i++) {
        SerialConfiguration* serialConfig = qobject_cast<SerialConfiguration*>(_sharedAutoconnectConfigurationList[i].data());
        if (serialConfig) {
            if (!currentPorts.contains(serialConfig->portName())) {
                if (serialConfig->link()) {
                    if (serialConfig->link()->isConnected()) {
                        if (serialConfig->link()->active()) {
                            // We don't remove links which are still connected which have been active with a vehicle on them
                            // even though at this point the cable may have been pulled. Instead we wait for the user to
                            // Disconnect. Once the user disconnects, the link will be removed.
                            continue;
                        }
                    }
                }
                _confToDelete.append(serialConfig);
            }
        } else {
            qWarning() << "Internal error";
        }
    }

    // Now remove all configs that are gone
    for (CommInterfaceConfiguration* pDeleteConfig: _confToDelete) {
        qCDebug(LinkManagerLog) << "Removing unused autoconnect config" << pDeleteConfig->name();
        if (pDeleteConfig->link()) {
            disconnectLink(pDeleteConfig->link());
        }
        for (int i=0; i<_sharedAutoconnectConfigurationList.count(); i++) {
            if (_sharedAutoconnectConfigurationList[i].data() == pDeleteConfig) {
                _sharedAutoconnectConfigurationList.removeAt(i);
                break;
            }
        }
    }

    // Check for RTK GPS connection gone
#if !defined(__mobile__)
    if (!_autoConnectRTKPort.isEmpty() && !currentPorts.contains(_autoConnectRTKPort)) {
        qCDebug(LinkManagerLog) << "RTK GPS disconnected" << _autoConnectRTKPort;
        _toolbox->gpsManager()->disconnectGPS();
        _autoConnectRTKPort.clear();
    }
#endif //__mobile__

#endif //__android__
#endif // NO_SERIAL_LINK
}
*/
}

void CommManager::_removeConfiguration(CommInterfaceConfiguration* config)
{
    _qmlConfigurations.removeOne(config);
    for (int i=0; i<_sharedConfigurationList.count(); i++)
	{
        if (_sharedConfigurationList[i].data() == config)
		{
            _sharedConfigurationList.removeAt(i);
            return;
        }
    }
    qWarning() << "CommManager::_removeConfiguration called with unknown config";
}
/*
void CommManager::_aboutToQuit()
{
}

void CommManager::_linkConnected()
{
}

void CommManager::_linkDisconnected()
{
}
*/
void CommManager::_linkConnectionRemoved(CommInterface* interface)
{
	disconnectLink(interface);
}
/*
void CommManager::_activeLinkCheck()
{
}
*/
//qmlRegisterSingletonType<CommManager>("QGroundControl.CommManager", 1, 0, "CommManager", CommManager::getCommManager());

