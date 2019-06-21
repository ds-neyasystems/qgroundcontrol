#pragma once

#include <QMap>
#include <QDebug>
#include <QSettings>

class CommInterface;

class CommInterfaceConfiguration : public QObject
{
	Q_OBJECT

public:
	CommInterfaceConfiguration(const QString& name);
	CommInterfaceConfiguration(CommInterfaceConfiguration* copy);
	virtual ~CommInterfaceConfiguration();

	static const QString settingsRoot;
	
	typedef QSharedPointer<CommInterfaceConfiguration> SharedPointer;
	
	enum CommInterfaceType
	{
        TypeSerial,     ///< Serial Link
        TypeUdp,        ///< UDP Link
        TypeTcp,        ///< TCP Link
        TypeBluetooth,  ///< Bluetooth Link
        TypeMock,       ///< Mock Link for Unitesting
        TypeLogReplay,
		TypeROS2,
        TypeLast        // Last type value (type >= TypeLast == invalid)
    };
    Q_ENUM(CommInterfaceType)

	typedef QMap<QString, CommInterfaceType> TypeMap;
		
    Q_PROPERTY(QString            name                READ name           WRITE setName           NOTIFY nameChanged)
	Q_PROPERTY(CommInterface*     link                READ commInterface  WRITE setCommInterface  NOTIFY interfaceChanged)
    Q_PROPERTY(CommInterfaceType  linkType            READ type                                   CONSTANT)
    Q_PROPERTY(bool               dynamic             READ isDynamic      WRITE setDynamic        NOTIFY dynamicChanged)
    Q_PROPERTY(bool               autoConnect         READ isAutoConnect  WRITE setAutoConnect    NOTIFY autoConnectChanged)
    Q_PROPERTY(bool               autoConnectAllowed  READ isAutoConnectAllowed                   CONSTANT)
    Q_PROPERTY(QString            settingsURL         READ settingsURL                            CONSTANT)
    Q_PROPERTY(QString            settingsTitle       READ settingsTitle                          CONSTANT)
    Q_PROPERTY(bool               highLatency         READ isHighLatency  WRITE setHighLatency    NOTIFY highLatencyChanged)
    Q_PROPERTY(bool               highLatencyAllowed  READ isHighLatencyAllowed                   CONSTANT)
	
    QString name(void) const;
    void setName(const QString name);
	virtual void fixName();

	CommInterface* commInterface();
	void setCommInterface(CommInterface* interface);

	static CommInterfaceConfiguration* create(int type, QString name);
	static CommInterfaceConfiguration* create(QString type, QString name);
	static const TypeMap& typeMap();
	
	virtual CommInterfaceConfiguration* duplicate();
	
    bool isDynamic();
    bool isAutoConnect();
    bool isHighLatency();
    void setDynamic(bool dynamic = true);
    void setAutoConnect(bool autoc = true);
    void setHighLatency(bool hl = false);

    virtual bool isAutoConnectAllowed();
	virtual bool isHighLatencyAllowed();
	virtual void updateSettings();
	virtual void copyFrom(CommInterfaceConfiguration* source);

    virtual CommInterfaceType type() = 0;
    virtual void loadSettings(QSettings& settings, const QString& root) = 0;
	virtual void saveSettings(QSettings& settings, const QString& root) = 0;
	
	virtual QString settingsURL() = 0;
	virtual QString settingsTitle() = 0;
	
signals:
    void nameChanged        (const QString& name);
    void dynamicChanged     ();
    void autoConnectChanged ();
	void interfaceChanged   (CommInterface* interface);
    void highLatencyChanged ();

protected:
    CommInterface* _interface; ///< Link currently using this configuration (if any)
    QString _name;
    bool    _dynamic;       ///< A connection added automatically and not persistent (unless it's edited).
    bool    _autoConnect;   ///< This connection is started automatically at boot
    bool    _highLatency;
	
	static TypeMap _typeMap;
};
