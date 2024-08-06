#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QtWebSockets/QWebSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include "rosbridgeclient.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void startserver();
    void closeserver();
    void startcamera();
    void end();
    void battery();
    void show_battery(QString message);
    void show_msgs(QString message);
private:
    Ui::MainWindow *ui;
    ROSBridgeClient* rosClient;
};
#endif // MAINWINDOW_H
