#pragma once

//#include <QObject.h>
#include <QList>
#include <QMap>
#include <QElapsedTimer>

#include "CommInterface.h"

class AutoConnectSettings;

class CommManager : public QObject
{
	Q_OBJECT
	
protected:
	CommManager();
	~CommManager();

	static CommManager* _commManager;
    static const char*  _defaultUDPLinkName;
    static const int    _autoconnectUpdateTimerMSecs;
    static const int    _autoconnectConnectDelayMSecs;
	static const int    _activeLinkCheckTimeoutMSecs;

public:
	static CommManager* getCommManager();
	
//  Q_PROPERTY(bool                 isBluetoothAvailable  READ isBluetoothAvailable    CONSTANT)
    Q_PROPERTY(QmlObjectListModel*  linkConfigurations    READ _qmlLinkConfigurations  NOTIFY linkConfigurationsChanged);
    Q_PROPERTY(QStringList          linkTypeStrings       READ commTypeStrings         CONSTANT);
//  Q_PROPERTY(QStringList          serialBaudRates       READ serialBaudRates         CONSTANT)
//  Q_PROPERTY(QStringList          serialPortStrings     READ serialPortStrings       NOTIFY commPortStringsChanged)
//  Q_PROPERTY(QStringList          serialPorts           READ serialPorts             NOTIFY commPortsChanged)

    // Create/Edit Link Configuration
    Q_INVOKABLE CommInterfaceConfiguration*  createConfiguration         (int type, const QString& name);
	Q_INVOKABLE CommInterfaceConfiguration*  createConfiguration         (const QString& type, const QString& name);
    Q_INVOKABLE CommInterfaceConfiguration*  startConfigurationEditing   (CommInterfaceConfiguration* config);
    Q_INVOKABLE void                         cancelConfigurationEditing  (CommInterfaceConfiguration* config);// { delete config; }
    Q_INVOKABLE bool                         endConfigurationEditing     (CommInterfaceConfiguration* config, CommInterfaceConfiguration* editedConfig);
    Q_INVOKABLE bool                         endCreateConfiguration      (CommInterfaceConfiguration* config);
    Q_INVOKABLE void                         removeConfiguration         (CommInterfaceConfiguration* config);

    Q_INVOKABLE void                         createConnectedLink         (CommInterfaceConfiguration* config);
	Q_INVOKABLE void                         disconnectLink              (CommInterface* interface);
	Q_INVOKABLE void                         shutdown                    ();

//	bool isBluetoothAvailable();
	void initialize();
	
    QList<CommInterface*> interfaces();
    QStringList commTypeStrings() const;
	
//    QStringList serialBaudRates();
//    QStringList serialPortStrings();
//    QStringList serialPorts();

    void loadCommConfigurationList();
    void saveCommConfigurationList();
    void suspendConfigurationUpdates(bool suspend);
    void setConnectionsSuspended(QString reason);
    void setConnectionsAllowed();
	
    CommInterface* createConnectedCommInterface(CommInterfaceConfiguration::SharedPointer& config, bool isPX4Flow = false);
	CommInterface* createConnectedCommInterface(const QString& name);
    void disconnectAll();
	
    bool isAutoconnectLink(CommInterface* interface);
	uint64_t runtime();

	bool containsInterface(CommInterface* interface);
	SharedCommInterfacePointer findInterface(CommInterface* interface);
	
	CommInterfaceConfiguration::SharedPointer addConfiguration(CommInterfaceConfiguration* config);

    void startAutoConnectedLinks(void);

	void sendGCSHeartbeat();
	
protected:
	void _deleteInterface(CommInterface* interface);
    void _addInterface(CommInterface* interface);
    QmlObjectListModel* _qmlLinkConfigurations();
    bool _connectionsSuspendedMsg(void);
    void _updateAutoConnectLinks(void);
    void _removeConfiguration(CommInterfaceConfiguration* config);

protected slots:
//	void _aboutToQuit();
//  void _linkConnected();
//  void _linkDisconnected();
    void _linkConnectionRemoved(CommInterface* interface);
//  void _activeLinkCheck();
	
signals:
//	void newLink(LinkInterface* link);
	void newInterface(CommInterface* interface);
    void interfaceDeleted(CommInterface* interface);
//	void linkConnected(LinkInterface* link);
//	void linkDisconnected(LinkInterface* link);
//  void linkActive(LinkInterface* link, int vehicleId, int vehicleFirmwareType, int vehicleType);
    void interfaceInactive(CommInterface* interface);
//  void commPortStringsChanged();
//  void commPortsChanged();
    void linkConfigurationsChanged();
	void vehicleHeartbeatInfo(CommInterface* link, int vehicleId, int componentId, int vehicleFirmwareType, int vehicleType);
	
protected:
	QList<SharedCommInterfacePointer> _interfaceList;
	QList<CommInterfaceConfiguration::SharedPointer> _sharedConfigurationList;
	QList<CommInterfaceConfiguration::SharedPointer> _sharedAutoconnectConfigurationList;
	QmlObjectListModel _qmlConfigurations;
	QMutex _interfaceListMutex;
	QElapsedTimer _timer;
	bool _initialized;
	AutoConnectSettings* _autoConnectSettings;
    bool    _configUpdateSuspended;                     ///< true: stop updating configuration list
    bool    _configurationsLoaded;                      ///< true: Link configurations have been loaded
    bool    _connectionsSuspended;                      ///< true: all new connections should not be allowed
    QString _connectionsSuspendedReason;
	QTimer _autoconnectTimer;
};
