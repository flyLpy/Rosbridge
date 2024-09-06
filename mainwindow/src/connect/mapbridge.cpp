#include "connect/mapbridge.h"
#include <QJsonArray>
#include <QDebug>
mapBridge::mapBridge(QObject* parent,QString a,QString b)
    : QObject(parent) {
    connect(&webSocket, &QWebSocket::textMessageReceived, this, &mapBridge::onTextMessageReceived);//接收到消息执行函数
    webSocket.open(QUrl("ws://" + a + ":" + b));
}

void mapBridge::receive_map(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    mapTopic = topic;
}
void mapBridge::reveive_lidar(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    lidarTopic = topic;
}
void mapBridge::reveive_GlobalCostMap(const QString& topic, const QString& type){
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    globalTopic = topic;
}
void mapBridge::reveive_Pose(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    poseTopic = topic;
}
void mapBridge::onTextMessageReceived(const QString& message) {
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
    if (!jsonDoc.isNull() && jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        QString topic = jsonObj.value("topic").toString();
        if (topic == mapTopic) {
            emit mapmessage(message);
        }else if(topic==lidarTopic){
            emit lidarmessage(message);
        }else if(topic==globalTopic){
            emit GlobalCostMapmessage(message);
        }else if(topic==poseTopic){
            emit Posemessage(message);
        }
    }
}
