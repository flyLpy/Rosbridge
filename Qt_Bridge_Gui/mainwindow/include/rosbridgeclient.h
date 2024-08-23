#ifndef ROSBRIDGECLIENT_H
#define ROSBRIDGECLIENT_H

#include <QObject>
#include <QtWebSockets/QWebSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextEdit>

class ROSBridgeClient : public QObject {
    Q_OBJECT
public:
    explicit ROSBridgeClient(QObject* parent = nullptr);
    void getTopicList();
    void subscribecameraTopic(const QString& topic, const QString& type);//摄像头消息
    void unsubscribeTopic(const QString& topic);
    void battery(const QString& topic, const QString& type);//电池消息
    void receive_odom(const QString& topic, const QString& type);//接受速度状态
    void sendspeed(QString message);//控制小车移动
    void map_topic(const QString &topic, const QString &type);//地图消息
signals:
    void cameramessageReceived(const QString& message);//摄像头
    void batterymessage(const QString& message);//电池电量
    void odommessage(const QString&message);//速度仪表盘
    void serverconnected();//连接建立完成
    void mapmassage(const QString&message);
private slots:
    void onConnected();
    void onTextMessageReceived(const QString& message);

private:
    QWebSocket webSocket;
    QTextEdit* textEdit;
    QString cameraTopic,batteryTopic,odomTopic,mapTopic;
};


#endif // ROSBRIDGECLIENT_H
