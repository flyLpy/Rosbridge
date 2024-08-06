#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QCoreApplication>
#include <QProcess>
#include <QDebug>
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    connect(ui->pushButton,&QPushButton::clicked,this,&MainWindow::startserver);//开启服务
    connect(ui->pushButton_2,&QPushButton::clicked,this,&MainWindow::closeserver);//关闭服务
    connect(ui->pushButton_3,&QPushButton::clicked,this,&MainWindow::startcamera);
    connect(ui->pushButton_4,&QPushButton::clicked,this,&MainWindow::end);
    connect(ui->pushButton_5,&QPushButton::clicked,this,&MainWindow::battery);
}

MainWindow::~MainWindow()
{
    delete ui;
}
//启动rosbridge服务
void MainWindow::startserver(){
    QString command = "ssh";
    QStringList args;
    args << "firefly@192.168.8.62" << "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 launch rosbridge_server rosbridge_websocket_launch.xml";
    // 创建QProcess对象
    QProcess *process = new QProcess();
    // 启动SSH命令
    process->start(command, args);
    // 获取命令输出
    QByteArray output = process->readAllStandardOutput();
    qDebug() << "SSH process output:" << output;
    // init client
    rosClient = new ROSBridgeClient(ui->textEdit, this);
}
void MainWindow::closeserver(){

}
//启动摄像头
void MainWindow::startcamera(){
    QString topic = "/camera/color/image_raw";
    QString type = "sensor_msgs/msg/Image";
    rosClient->subscribecameraTopic(topic, type);
    connect(rosClient, &ROSBridgeClient::cameramessageReceived, this, &MainWindow::show_msgs);
    //cameramessageReceived是收到消息传到这个函数继续执行，如果有不同的topic可以根据topic的名字判断分发到那里
}
//显示摄像头的实现
void MainWindow::show_msgs(QString message){
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject jsonObj = jsonDoc.object();

    // 从 JSON 对象中提取消息字段
    QJsonObject msgObj = jsonObj.value("msg").toObject();
    QString encoding = msgObj.value("encoding").toString();
    int width = msgObj.value("width").toInt();
    int height = msgObj.value("height").toInt();
    QByteArray data = QByteArray::fromBase64(msgObj.value("data").toString().toUtf8());

    // 处理 "rgb8" 编码的图像数据
    if (encoding == "rgb8") {
        // 创建 QImage 对象
        QImage image(reinterpret_cast<const uchar*>(data.data()), width, height, width * 3, QImage::Format_RGB888);

        // 显示图像
        ui->imageLabel->setPixmap(QPixmap::fromImage(image));
    } else {
        qWarning() << "Unsupported encoding:" << encoding;
    }

    // ui->textEdit->append(message);
}
void MainWindow::end(){
     QString topic = "/camera/color/image_raw";
     rosClient->unsubscribeTopic(topic);
     ui->imageLabel->clear();
}
void MainWindow::battery(){
    QString topic = "/battery";
    QString type = "sensor_msgs/msg/BatteryState";
    rosClient->battery(topic, type);
    connect(rosClient, &ROSBridgeClient::batterymessage, this, &MainWindow::show_battery);
}
void MainWindow::show_battery(QString message){
    double batteryVoltage,batteryPercentage;
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
        if (!jsonDoc.isNull() && jsonDoc.isObject()) {
            QJsonObject jsonObj = jsonDoc.object();

            // 提取电池状态字段
            QJsonObject batteryObj = jsonObj.value("msg").toObject();
            batteryVoltage = batteryObj.value("voltage").toDouble();
            batteryPercentage = batteryObj.value("percentage").toDouble();

            // 更新 UI
            ui->batteryLabel->setText(QString("percent: %1%\nVoltage: %2V")
                                      .arg(batteryPercentage)
                                      .arg(batteryVoltage));
        }
        qDebug()<<batteryVoltage<<"     "<<batteryPercentage;
}
