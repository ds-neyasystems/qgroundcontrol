#include "MAVLinkUDPCommInterface.h"
#include "SettingsManager.h"

/* ***************** *
 * Utility Functions *
 * ***************** */
bool is_ip(const QString& address)
{
    int a,b,c,d;
    if (sscanf(address.toStdString().c_str(), "%d.%d.%d.%d", &a, &b, &c, &d) != 4
		&& strcmp("::1", address.toStdString().c_str()))
        return false;
	else
        return true;
}

QString get_ip_address(const QString& address)
{
    if(is_ip(address))
        return address;
    // Need to look it up
    QHostInfo info = QHostInfo::fromName(address);
    if (info.error() == QHostInfo::NoError)
    {
        QList<QHostAddress> hostAddresses = info.addresses();
        QHostAddress address;
        for (int i = 0; i < hostAddresses.size(); i++)
        {
            // Exclude all IPv6 addresses
            if (!hostAddresses.at(i).toString().contains(":"))
            {
                return hostAddresses.at(i).toString();
            }
        }
    }
    return {};
}

bool contains_target(const QList<UDPClient*> list, const QHostAddress& address, quint16 port)
{
    for(UDPClient* target: list)
        if(target->address == address && target->port == port)
            return true;

    return false;
}

/* ********* *
 * UDPClient *
 * ********* */
UDPClient::UDPClient(const QHostAddress& address_, quint16 port_) :
	address(address_),
	port(port_)
{}
UDPClient::UDPClient(const UDPClient* other) :
	address(other->address),
	port(other->port)
{}

/* ************************************ *
 * MAVLinkUDPCommInterfaceConfiguration *
 * ************************************ */
MAVLinkUDPCommInterfaceConfiguration::MAVLinkUDPCommInterfaceConfiguration(const QString& name) : CommInterfaceConfiguration(name)
{
    AutoConnectSettings* settings = qgcApp()->toolbox()->settingsManager()->autoConnectSettings();
    _localPort = settings->udpListenPort()->rawValue().toInt();
    QString targetHostIP = settings->udpTargetHostIP()->rawValue().toString();
    if (!targetHostIP.isEmpty())
	{
        addHost(targetHostIP, settings->udpTargetHostPort()->rawValue().toUInt());
    }	
}

MAVLinkUDPCommInterfaceConfiguration::MAVLinkUDPCommInterfaceConfiguration(MAVLinkUDPCommInterfaceConfiguration* copy) : CommInterfaceConfiguration(copy)
{
	_copyFrom(copy);
}

MAVLinkUDPCommInterfaceConfiguration::~MAVLinkUDPCommInterfaceConfiguration()
{
	_clearTargetHosts();
}

void MAVLinkUDPCommInterfaceConfiguration::fixName()
{
	if( _name != "Unnamed" )
		return;

	_name = QString("UDP MAVLink Comm Interface on Port %1").arg(_localPort);
}

quint16 MAVLinkUDPCommInterfaceConfiguration::localPort()
{
	return _localPort;	
}

void MAVLinkUDPCommInterfaceConfiguration::setLocalPort(quint16 port)
{
    _localPort = port;	
}

void MAVLinkUDPCommInterfaceConfiguration::addHost (const QString host)
{
    // Handle x.x.x.x:p
    if (host.contains(":"))
    {
        addHost(host.split(":").first(), host.split(":").last().toUInt());
    }
    // If no port, use default
    else
    {
        addHost(host, _localPort);
    }	
}

void MAVLinkUDPCommInterfaceConfiguration::removeHost(const QString host)
{
    if (host.contains(":"))
    {
        QHostAddress address = QHostAddress(get_ip_address(host.split(":").first()));
        quint16 port = host.split(":").last().toUInt();
        for(int i = 0; i < _targetHosts.size(); i++)
		{
            UDPClient* target = _targetHosts.at(i);
            if(target->address == address && target->port == port)
			{
                _targetHosts.removeAt(i);
                delete target;
                _updateHostList();
                return;
            }
        }
    }
    qWarning() << "UDP:" << "Could not remove unknown host:" << host;
    _updateHostList();	
}

void MAVLinkUDPCommInterfaceConfiguration::addHost(const QString& host, quint16 port)
{
    QString ipAdd = get_ip_address(host);
	
    if(ipAdd.isEmpty())
	{
        qWarning() << "UDP:" << "Could not resolve host:" << host << "port:" << port;
		return;
    } 

    QHostAddress address(ipAdd);
	if(!contains_target(_targetHosts, address, port))
	{
		UDPClient* newTarget = new UDPClient(address, port);
		_targetHosts.append(newTarget);
		_updateHostList();
	}    	
}

QStringList MAVLinkUDPCommInterfaceConfiguration::hostList()
{
	return _hostList; 	
}

const QList<UDPClient*> MAVLinkUDPCommInterfaceConfiguration::targetHosts()
{
	return _targetHosts;
}

CommInterfaceConfiguration::CommInterfaceType MAVLinkUDPCommInterfaceConfiguration::type()
{
	return CommInterfaceConfiguration::TypeUdp;
}

void MAVLinkUDPCommInterfaceConfiguration::copyFrom(CommInterfaceConfiguration* source)
{
    CommInterfaceConfiguration::copyFrom(source);
    _copyFrom(source);	
}

void MAVLinkUDPCommInterfaceConfiguration::loadSettings(QSettings& settings, const QString& root)
{
    AutoConnectSettings* acSettings = qgcApp()->toolbox()->settingsManager()->autoConnectSettings();
    _clearTargetHosts();
    settings.beginGroup(root);
    _localPort = (quint16)settings.value("port", acSettings->udpListenPort()->rawValue().toInt()).toUInt();
    int hostCount = settings.value("hostCount", 0).toInt();
    for(int i = 0; i < hostCount; i++)
	{
        QString hkey = QString("host%1").arg(i);
        QString pkey = QString("port%1").arg(i);
        if(settings.contains(hkey) && settings.contains(pkey))
            addHost(settings.value(hkey).toString(), settings.value(pkey).toUInt());
    }
    settings.endGroup();
    _updateHostList();	
}

void MAVLinkUDPCommInterfaceConfiguration::saveSettings(QSettings& settings, const QString& root)
{
    settings.beginGroup(root);
    settings.setValue("port", (int)_localPort);
    settings.setValue("hostCount", _targetHosts.size());
    for(int i = 0; i < _targetHosts.size(); i++)
	{
        UDPClient* target = _targetHosts.at(i);
        QString hkey = QString("host%1").arg(i);
        settings.setValue(hkey, target->address.toString());
        QString pkey = QString("port%1").arg(i);
        settings.setValue(pkey, target->port);
    }
    settings.endGroup();	
}

void MAVLinkUDPCommInterfaceConfiguration::updateSettings()
{
    if(_interface)
	{
        MAVLinkUDPCommInterface* interface = dynamic_cast<MAVLinkUDPCommInterface*>(_interface);
        if(interface && interface->isConnected())
		{
            interface->_disconnect();
			interface->_connect();
		}
    }	
}

bool MAVLinkUDPCommInterfaceConfiguration::isAutoConnectAllowed()
{
	return true;
}

bool MAVLinkUDPCommInterfaceConfiguration::isHighLatencyAllowed()
{
	return true;
}

QString MAVLinkUDPCommInterfaceConfiguration::settingsURL()
{
	return "UdpSettings.qml";
}

QString MAVLinkUDPCommInterfaceConfiguration::settingsTitle()
{
	return tr("UDP Link Settings");
}

void MAVLinkUDPCommInterfaceConfiguration::_updateHostList()
{
    _hostList.clear();
    for(int i = 0; i < _targetHosts.size(); i++)
	{
        UDPClient* target = _targetHosts.at(i);
        QString host = QString("%1").arg(target->address.toString()) + ":" + QString("%1").arg(target->port);
        _hostList << host;
    }
    emit hostListChanged();	
}

void MAVLinkUDPCommInterfaceConfiguration::_clearTargetHosts()
{
    qDeleteAll(_targetHosts);
    _targetHosts.clear();	
}

void MAVLinkUDPCommInterfaceConfiguration::_copyFrom(CommInterfaceConfiguration *source)
{
    MAVLinkUDPCommInterfaceConfiguration* usource = dynamic_cast<MAVLinkUDPCommInterfaceConfiguration*>(source);
    if (usource)
	{
        _localPort = usource->localPort();
        _clearTargetHosts();
        for(UDPClient* target: usource->targetHosts())
		{
            if(!contains_target(_targetHosts, target->address, target->port))
			{
                UDPClient* newTarget = new UDPClient(target);
                _targetHosts.append(newTarget);
                _updateHostList();
            }
        }
    }
	else
	{
        qWarning() << "Internal error";
    }	
}

/* *********************** *
 * MAVLinkUDPCommInterface *
 * *********************** */
MAVLinkUDPCommInterface::MAVLinkUDPCommInterface(CommInterfaceConfiguration::SharedPointer config, bool is_px4) :
	MAVLinkCommInterface(config, is_px4),
	_decodedFirstMAVLinkPacket(false),
	_running(false),
	_connected(false),
	_socket(NULL),
	_sessionTargets(),
	_localAddress()
{
	MAVLinkUDPCommInterfaceConfiguration* udp_config = qobject_cast<MAVLinkUDPCommInterfaceConfiguration*>(config.data());
    if (!udp_config)
	{
        qWarning() << "Internal error";
    }
    for (const QHostAddress &address: QNetworkInterface::allAddresses())
	{
        _localAddress.append(QHostAddress(address));
    }
    moveToThread(this);	
}

MAVLinkUDPCommInterface::~MAVLinkUDPCommInterface()
{
    _disconnect();
    // Tell the thread to exit
    _running = false;
    // Clear client list
    qDeleteAll(_sessionTargets);
    _sessionTargets.clear();
    quit();
    // Wait for it to exit
    wait();
    this->deleteLater();	
}

void MAVLinkUDPCommInterface::run()
{
    if(_hardwareConnect())
        exec();

    if (_socket)
        _socket->close();
}



bool MAVLinkUDPCommInterface::isConnected() const
{
	return _connected;
}

void MAVLinkUDPCommInterface::requestReset()
{
}
	
void MAVLinkUDPCommInterface::readBytes()
{
    if (!_socket)
        return;

    QByteArray databuffer;
    while (_socket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(_socket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;
		
        //-- Note: This call is broken in Qt 5.9.3 on Windows. It always returns a blank sender and 0 for the port.
        _socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        databuffer.append(datagram);
        //-- Wait a bit before sending it over
        if(databuffer.size() > 10 * 1024)
		{
            _parseBytes( databuffer );
            databuffer.clear();
        }
		
//        _logInputDataRate(datagram.length(), QDateTime::currentMSecsSinceEpoch());
        // TODO: This doesn't validade the sender. Anything sending UDP packets to this port gets
        // added to the list and will start receiving datagrams from here. Even a port scanner
        // would trigger this.
        // Add host to broadcast list if not yet present, or update its port
        QHostAddress asender = sender;
		for (const QHostAddress &address: _localAddress)
		{
			if (address == sender)
			{
				asender = QHostAddress(QString("127.0.0.1"));
				break;
			}
		}
        
        if(!contains_target(_sessionTargets, asender, senderPort))
		{
            qDebug() << "Adding target" << asender << senderPort;
            UDPClient* target = new UDPClient(asender, senderPort);
            _sessionTargets.append(target);
        }
    }
    //-- Send whatever is left
    if(databuffer.size())
        _parseBytes(databuffer);

}

void MAVLinkUDPCommInterface::_sendMAVLinkMessage( mavlink_message_t& msg )
{
	uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
	int msg_len;
	MAVLinkUDPCommInterfaceConfiguration* config;

	//Get config
	config = qobject_cast<MAVLinkUDPCommInterfaceConfiguration*>(_config.data());
	if (!config )
		return;

	//Pack message into buffer
	msg_len = mavlink_msg_to_send_buffer(buffer, &msg);

	//Send buffer to manually specified targets
	for( UDPClient* target : config->targetHosts() )
	{
		if( contains_target(_sessionTargets, target->address, target->port) )
			continue;
		
		_socket->writeDatagram(QByteArray((char*)buffer, msg_len), target->address, target->port);
	}

	//Send buffer to session targets
	for( UDPClient* target : _sessionTargets )
		_socket->writeDatagram(QByteArray((char*)buffer, msg_len), target->address, target->port);
}

bool MAVLinkUDPCommInterface::_connect(void)
{
    if(this->isRunning() || _running)
    {
        _running = false;
        quit();
        wait();
    }
    _running = true;
    start(NormalPriority);
    return true;	
}

void MAVLinkUDPCommInterface::_disconnect(void)
{
   _running = false;
    quit();
    wait();
    if (_socket) {
        // Make sure delete happen on correct thread
        _socket->deleteLater();
        _socket = NULL;
//        emit disconnected();
    }
    _connected = false;	
}

bool  MAVLinkUDPCommInterface::_hardwareConnect()
{
    if (_socket)
	{
        delete _socket;
        _socket = NULL;
    }

	MAVLinkUDPCommInterfaceConfiguration* config;

	config = qobject_cast<MAVLinkUDPCommInterfaceConfiguration*>(_config.data());
	if( !config )
		return false;
	
    QHostAddress host = QHostAddress::AnyIPv4;
    _socket = new QUdpSocket(this);
    _socket->setProxy(QNetworkProxy::NoProxy);
    _connected = _socket->bind(host, config->localPort(), QAbstractSocket::ReuseAddressHint | QUdpSocket::ShareAddress);
	
    if (_connected)
	{
        _socket->joinMulticastGroup(QHostAddress("224.0.0.1"));
		
        //-- Make sure we have a large enough IO buffers
#ifdef __mobile__
        _socket->setSocketOption(QAbstractSocket::SendBufferSizeSocketOption,     64 * 1024);
        _socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 128 * 1024);
#else
        _socket->setSocketOption(QAbstractSocket::SendBufferSizeSocketOption,    256 * 1024);
        _socket->setSocketOption(QAbstractSocket::ReceiveBufferSizeSocketOption, 512 * 1024);
#endif
//        _registerZeroconf(_udpConfig->localPort(), kZeroconfRegistration);
        QObject::connect(_socket, &QUdpSocket::readyRead, this, &MAVLinkUDPCommInterface::readBytes);
//        emit connected();
    }
	else
	{
        emit communicationError(tr("UDP Link Error"), tr("Error binding UDP port: %1").arg(_socket->errorString()));
    }
    return _connected;
}
