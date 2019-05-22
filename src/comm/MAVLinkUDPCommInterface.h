#pragma once

#include "MAVLinkCommInterface.h"
#include "AutoConnectSettings.h"
#include "QGCApplication.h"
#include "CommInterfaceConfiguration.h"
#include "CommManager.h"

/* ********* *
 * UDPClient *
 * ********* */
class UDPClient
{
public:
    UDPClient(const QHostAddress& address_, quint16 port_);
    UDPClient(const UDPClient* other);
	
    QHostAddress    address;
    quint16         port;
};

/* ************************************ *
 * MAVLinkUDPCommInterfaceConfiguration *
 * ************************************ */
class MAVLinkUDPCommInterfaceConfiguration : public CommInterfaceConfiguration
{
    Q_OBJECT
	
public:
    Q_PROPERTY(quint16      localPort   READ localPort  WRITE setLocalPort  NOTIFY localPortChanged)
    Q_PROPERTY(QStringList  hostList    READ hostList                       NOTIFY hostListChanged)

	MAVLinkUDPCommInterfaceConfiguration(const QString& name);
	MAVLinkUDPCommInterfaceConfiguration(MAVLinkUDPCommInterfaceConfiguration* copy);
	virtual ~MAVLinkUDPCommInterfaceConfiguration();

	void fixName();
	
	quint16 localPort();
	void setLocalPort   (quint16 port);

	Q_INVOKABLE void addHost (const QString host);
	Q_INVOKABLE void removeHost  (const QString host);
	
	void addHost        (const QString& host, quint16 port);
	QStringList hostList ();
	const QList<UDPClient*> targetHosts();

    /// From CommInterfaceConfiguration
    CommInterfaceType type                 ();
    void              copyFrom             (CommInterfaceConfiguration* source);
    void              loadSettings         (QSettings& settings, const QString& root);
    void              saveSettings         (QSettings& settings, const QString& root);
    void              updateSettings       ();
    bool              isAutoConnectAllowed ();
    bool              isHighLatencyAllowed ();
    QString           settingsURL          ();
    QString           settingsTitle        ();

signals:
    void localPortChanged();
    void hostListChanged();

private:
    void _updateHostList();
    void _clearTargetHosts();
    void _copyFrom(CommInterfaceConfiguration *source);

private:
    QList<UDPClient*> _targetHosts;
    QStringList _hostList;
    quint16 _localPort;	
};

/* *********************** *
 * MAVLinkUDPCommInterface *
 * *********************** */
class MAVLinkUDPCommInterface : public MAVLinkCommInterface
{
	Q_OBJECT

	friend class CommManager;
	friend class MAVLinkUDPCommInterfaceConfiguration;
public:
	MAVLinkUDPCommInterface(CommInterfaceConfiguration::SharedPointer config, bool is_px4 = false);
	virtual ~MAVLinkUDPCommInterface();

	// Thread
    void run() override;

	//CommInterface
	bool isConnected() const;
	void requestReset();
	
public slots:
	void readBytes();

protected:
	//MAVLinkCommInterface
	void _sendMAVLinkMessage( mavlink_message_t& msg );

	//CommInterface
	bool _connect(void);
	void _disconnect(void);

   bool _hardwareConnect();
	
//	uint8_t _mavlinkSystemID;
//	uint8_t _mavlinkComponentID;
//	uint8_t _mavlinkChannel;

	bool _decodedFirstMAVLinkPacket;    ///< true: link has correctly decoded it's first mavlink packet
	bool _running;
	bool _connected;

	QUdpSocket*         _socket;
	QList<UDPClient*>   _sessionTargets;
    QList<QHostAddress> _localAddress;
};

