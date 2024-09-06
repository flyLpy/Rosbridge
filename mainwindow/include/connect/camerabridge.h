#ifndef CAMERABRIDGE_H
#define CAMERABRIDGE_H
#include <QObject>
#include <QtWebSockets/QWebSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QTextEdit>
class CameraBridgeClient : public QObject {
    Q_OBJECT
public:
    explicit CameraBridgeClient(QObject* parent = nullptr,QString a=QString(),QString b=QString());
    void subscribecameraTopic(const QString& topic, const QString& type);//摄像头消息
    void unsubscribeTopic(const QString& topic);
signals:
    void cameramessageReceived(const QString& message);//摄像头
private slots:
    void onTextMessageReceived(const QString& message);
private:
    QWebSocket webSocket;
    QString cameraTopic;
};
#endif // CAMERABRIDGE_H

