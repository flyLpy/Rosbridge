#include "connect/camerabridge.h"
#include <QJsonArray>
#include <QDebug>
CameraBridgeClient::CameraBridgeClient(QObject* parent,QString a,QString b)
    : QObject(parent) {
    connect(&webSocket, &QWebSocket::textMessageReceived, this, &CameraBridgeClient::onTextMessageReceived);//接收到消息执行函数
    webSocket.open(QUrl("ws://" + a + ":" + b));
}

void CameraBridgeClient::subscribecameraTopic(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    cameraTopic = topic;
}
void CameraBridgeClient::onTextMessageReceived(const QString& message) {
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
    if (!jsonDoc.isNull() && jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        QString topic = jsonObj.value("topic").toString();
        if (topic == cameraTopic) {
            emit cameramessageReceived(message);
        }
    }
}
