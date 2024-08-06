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
    explicit ROSBridgeClient(QTextEdit* textEdit, QObject* parent = nullptr);
    void getTopicList();
    void subscribecameraTopic(const QString& topic, const QString& type);
    void unsubscribeTopic(const QString& topic);
    void battery(const QString& topic, const QString& type);
signals:
    void cameramessageReceived(const QString& message);
    void batterymessage(const QString& message);
private slots:
    void onConnected();
    void onTextMessageReceived(const QString& message);

private:
    QWebSocket webSocket;
    QTextEdit* textEdit;
    QString cameraTopic,batteryTopic;
};


#endif // ROSBRIDGECLIENT_H
