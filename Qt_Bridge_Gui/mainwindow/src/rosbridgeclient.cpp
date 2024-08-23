#include "rosbridgeclient.h"
#include <QJsonArray>
#include <QDebug>
ROSBridgeClient::ROSBridgeClient(QObject* parent)
    : QObject(parent) {
    connect(&webSocket, &QWebSocket::connected, this, &ROSBridgeClient::onConnected);
    connect(&webSocket, &QWebSocket::textMessageReceived, this, &ROSBridgeClient::onTextMessageReceived);//接收到消息执行函数
    webSocket.open(QUrl("ws://192.168.8.62:9090")); // 修改为远程 IP 地址
}

void ROSBridgeClient::getTopicList() {
    QJsonObject json;
    json["op"] = "call_service";
    json["service"] = "/rosapi/topics";
    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
}

void ROSBridgeClient::subscribecameraTopic(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    cameraTopic = topic;
}

void ROSBridgeClient::unsubscribeTopic(const QString& topic) {
    QJsonObject json;
    json["op"] = "unsubscribe";
    json["topic"] = topic;
    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    if (cameraTopic == topic) {
        cameraTopic.clear();
    }
    if (batteryTopic == topic) {
        batteryTopic.clear();
    }
}
void ROSBridgeClient::battery(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    batteryTopic = topic;
}
void ROSBridgeClient::receive_odom(const QString &topic, const QString &type){
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    odomTopic = topic;
}
void ROSBridgeClient::map_topic(const QString &topic, const QString &type){
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    mapTopic = topic;
}
void ROSBridgeClient::onConnected() {
    qDebug() << "WebSocket connected";
    emit serverconnected();
}

void ROSBridgeClient::onTextMessageReceived(const QString& message) {
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
    if (!jsonDoc.isNull() && jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        QString topic = jsonObj.value("topic").toString();
        if (topic == cameraTopic) {
            emit cameramessageReceived(message);
        } else if (topic == batteryTopic) {
            emit batterymessage(message);
        }else if(topic ==odomTopic){
            emit odommessage(message);
        }else if(topic ==mapTopic){
            emit mapmassage(message);
        }
    }
}
void ROSBridgeClient::sendspeed(QString message){
    webSocket.sendTextMessage(message);
}
