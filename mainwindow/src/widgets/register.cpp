#include "widgets/register.h"
#include "QDebug"
Sensor::Sensor(QWidget* parent)
    : QWidget(parent)
{
  this->resize(parent->size());
  this->parent = parent;
        pixmap1.load(":/images/false.png");
        pixmap2.load(":/images/false.png");
        pixmap3.load(":/images/false.png");
        pixmap4.load(":/images/false.png");
            QLabel *ipLabel = new QLabel("ip:");
            ipEdit = new QLineEdit;
            // 创建密码标签和输入框
            QLabel *portLabel = new QLabel("port:");
            portEdit = new QLineEdit;
            ipEdit->setText("192.168.8.62");
            portEdit->setText("9090");
            // 创建登录按钮
            QPushButton *loginButton = new QPushButton("建立连接");
            QPushButton *dloginButton = new QPushButton("断开连接");
            //创建建图标签
            QPushButton *startmap = new QPushButton("开始建图");
            QPushButton *closemap = new QPushButton("结束建图");
            //创建功能
            QPushButton *open_camera = new QPushButton("打开摄像头");
            QPushButton *open_map = new QPushButton("打开导航");
            // 布局设置
            QVBoxLayout *mainLayout = new QVBoxLayout();
            QHBoxLayout *ipLayout = new QHBoxLayout();
            QHBoxLayout *portLayout = new QHBoxLayout();
            QHBoxLayout *buttonLayout = new QHBoxLayout();
            QHBoxLayout *mapLayout=new QHBoxLayout();
            QHBoxLayout *openLayout=new QHBoxLayout();
            openLayout->addWidget(open_camera);
            openLayout->addWidget(open_map);
            ipLayout->addWidget(ipLabel);
            ipLayout->addWidget(ipEdit);
            portLayout->addWidget(portLabel);
            portLayout->addWidget(portEdit);
            buttonLayout->addWidget(loginButton);
            buttonLayout->addWidget(dloginButton);
            mapLayout->addWidget(startmap);
            mapLayout->addWidget(closemap);

            mainLayout->addLayout(ipLayout);
            mainLayout->addLayout(portLayout);
            mainLayout->addLayout(buttonLayout);
            mainLayout->addLayout(mapLayout);
            mainLayout->addLayout(openLayout);
            // 创建第一个水平布局
             QHBoxLayout *layout1 = new QHBoxLayout();
             label1 = new QLabel("IMU");
             labelimg1=new QLabel();
             labelimg1->setPixmap(pixmap1);
             label1->setFont(QFont("Arial", 14)); // 设置字体大小为14像素
             layout1->addWidget(label1);
             layout1->setSpacing(50);
             layout1->addWidget(labelimg1);
             // 创建第二个水平布局
              QHBoxLayout *layout2 = new QHBoxLayout();
              label2 = new QLabel("雷达");
              labelimg2 = new QLabel();
              labelimg2->setPixmap(pixmap2);
              label2->setFont(QFont("Arial", 14)); // 设置字体大小为14像素
              layout2->addWidget(label2);
              layout2->setSpacing(50);
              layout2->addWidget(labelimg2);
              // 创建第三个水平布局
              QHBoxLayout *layout3 = new QHBoxLayout();
              label3 = new QLabel("摄像头");
              labelimg3 = new QLabel();
              labelimg3->setPixmap(pixmap3);
              label3->setFont(QFont("Arial", 14)); // 设置字体大小为14像素
              layout3->addWidget(label3);
              layout3->setSpacing(50);
              layout3->addWidget(labelimg3);
              // 创建第三个水平布局
              QHBoxLayout *layout4 = new QHBoxLayout();
              label4 = new QLabel("GNSS");
              labelimg4 = new QLabel();
              labelimg4->setPixmap(pixmap4);
              label4->setFont(QFont("Arial", 14)); // 设置字体大小为14像素
              layout4->addWidget(label4);
              layout4->setSpacing(50);
              layout4->addWidget(labelimg4);
                mainLayout->addLayout(layout1);
                mainLayout->addLayout(layout2);
                mainLayout->addLayout(layout3);
                mainLayout->addLayout(layout4);
            setLayout(mainLayout);
            /////////////////////////////////////////////////////////////////////////////////////////////槽链接
            connect(loginButton,&QPushButton::clicked,this,&Sensor::start_server);
            connect(dloginButton,&QPushButton::clicked,this,&Sensor::close_server);
            connect(open_camera,&QPushButton::clicked,this,&Sensor::start_camera);
            connect(open_map,&QPushButton::clicked,this,&Sensor::start_map);
            connect(startmap,&QPushButton::clicked,this,&Sensor::begin_map);
            connect(closemap,&QPushButton::clicked,this,&Sensor::end_map);
            connect(&webSocket, &QWebSocket::textMessageReceived, this, &Sensor::onTextMessageReceived);//接收到消息执行函数
}

void Sensor::paintEvent(QPaintEvent* event) {
  this->resize(parent->size());
  QWidget::paintEvent(event);
  int side = qMin(int(parent->width() / 1.8), parent->height());
    label1->setFixedSize(side/2, side/2);
    label2->setFixedSize(side/2, side/2);
    label3->setFixedSize(side/2, side/2);
    label4->setFixedSize(side/2, side/2);
    labelimg1->setFixedSize(side/13,side/13);
    labelimg2->setFixedSize(side/13,side/13);
    labelimg3->setFixedSize(side/13,side/13);
    labelimg4->setFixedSize(side/13,side/13);
    int fontSize = std::max(10, parent->width() / 20); // 设置一个最小字体大小为10
    QFont font = label1->font();
    font.setPointSize(fontSize);
    label1->setFont(font);
    label2->setFont(font);
    label3->setFont(font);
    label4->setFont(font);
}
void Sensor::start_server(){
    QString ipaddress,port;
    ipaddress = ipEdit->text();
    port = portEdit->text();
    connect(&webSocket, &QWebSocket::connected, this, [this]() {
        emit websocketConnected();
        QMessageBox::information(this, "WebSocket", "WebSocket connect successful!");
    });
    connect(&webSocket, QOverload<QAbstractSocket::SocketError>::of(&QWebSocket::error), this, [this](QAbstractSocket::SocketError error) {
        QMessageBox::warning(this, "WebSocket", "Please connect again: " + webSocket.errorString());
    });

    webSocket.open(QUrl("ws://" + ipaddress + ":" + port));
}
void Sensor::close_server(){
    emit websocketClosed();
}
void Sensor::receive_battery(const QString& topic, const QString& type) {
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;
    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    batteryTopic = topic;
}
void Sensor::receive_odom(const QString &topic, const QString &type){
    QJsonObject json;
    json["op"] = "subscribe";
    json["topic"] = topic;
    json["type"] = type;

    webSocket.sendTextMessage(QJsonDocument(json).toJson(QJsonDocument::Compact));
    odomTopic = topic;
}
void Sensor::sendmessage(QString message){
    webSocket.sendTextMessage(message);
}
void Sensor::start_camera(){
    emit opencamera();
}
void Sensor::start_map(){
    emit openmap();
}
void Sensor::update_pose(){
    emit updatepose();
}
void Sensor::onTextMessageReceived(const QString& message) {
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
    if (!jsonDoc.isNull() && jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        QString topic = jsonObj.value("topic").toString();
        if (topic == batteryTopic){
                emit batterymessage(message);
        }else if(topic ==odomTopic){
            emit odommessage(message);
        }
    }
}
void Sensor::setpicture1(){
    QPixmap pixmap;
    pixmap.load(":/images/ok.png");
    labelimg1->setPixmap(pixmap);
}
void Sensor::setpicture2(){
    QPixmap pixmap;
    pixmap.load(":/images/ok.png");
    labelimg2->setPixmap(pixmap);
}
void Sensor::setpicture3(){
    QPixmap pixmap;
    pixmap.load(":/images/ok.png");
    labelimg3->setPixmap(pixmap);
}
void Sensor::begin_map()
{
    // 创建 QProcess 对象
    QProcess *process = new QProcess(this);

    // 完整的命令字符串
    QString command = "ssh -X firefly@192.168.8.62 'source /home/firefly/lpy/startmap.bash'";

    // 连接信号槽以处理输出和错误
    connect(process, &QProcess::readyReadStandardOutput, [process]() {
        qDebug() << "Output:" << process->readAllStandardOutput().trimmed();
    });
    connect(process, &QProcess::readyReadStandardError, [process]() {
        qDebug() << "Error:" << process->readAllStandardError().trimmed();
    });
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [](int exitCode, QProcess::ExitStatus exitStatus) {
        qDebug() << "Process finished with exit code" << exitCode << "and status" << exitStatus;
    });

    // 启动 SSH 命令
    process->start("bash", QStringList() << "-c" << command);
}


void Sensor::end_map()
{
    QProcess *process = new QProcess(this);

    // 完整的命令字符串
    QString command = "ssh -X firefly@192.168.8.62 'source /home/firefly/lpy/endmap.bash'";

    // 连接信号槽以处理输出和错误
    connect(process, &QProcess::readyReadStandardOutput, [process]() {
        qDebug() << "Output:" << process->readAllStandardOutput().trimmed();
    });
    connect(process, &QProcess::readyReadStandardError, [process]() {
        qDebug() << "Error:" << process->readAllStandardError().trimmed();
    });
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), [](int exitCode, QProcess::ExitStatus exitStatus) {
        qDebug() << "Process finished with exit code" << exitCode << "and status" << exitStatus;
    });

    // 启动 SSH 命令
    process->start("bash", QStringList() << "-c" << command);

}
