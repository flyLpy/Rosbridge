#ifndef MAPBRIDGE_H
#define MAPBRIDGE_H
#include <QObject>
#include <QtWebSockets/QWebSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextEdit>
#include "widgets/register.h"
class mapBridge : public QObject {
    Q_OBJECT
public:
    explicit mapBridge(QObject* parent = nullptr,QString a=QString(),QString b=QString());
    void reveive_lidar(const QString& topic, const QString& type);//雷达消息
    void receive_map(const QString &topic, const QString &type);//地图消息
    void reveive_GlobalCostMap(const QString &topic, const QString &type);//全局代价地图
    void reveive_Pose(const QString &topic, const QString &type);//位姿
signals:
    void lidarmessage(const QString& message);//雷达
    void mapmessage(const QString&message);//地图
    void GlobalCostMapmessage(const QString&message);
    void Posemessage(const QString&message);
private slots:
    void onTextMessageReceived(const QString& message);

private:
    QWebSocket webSocket;
    QString lidarTopic,mapTopic,globalTopic,poseTopic;
};
#endif // MAPBRIDGE_H
