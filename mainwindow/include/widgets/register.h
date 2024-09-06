#ifndef REGISTER_H
#define REGISTER_H
#include <QImageReader>
#include <QtWebSockets/QWebSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QCalendarWidget>
#include <QComboBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QGraphicsItem>
#include <QHBoxLayout>
#include <QLabel>
#include <QMainWindow>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QProgressBar>
#include <QPushButton>
#include <QRadioButton>
#include <QSettings>
#include <QTableWidget>
#include <QToolBar>
#include <QWidgetAction>
#include <QBuffer>
#include <QImageReader>
#include <QLineEdit>
#include <QProcess>
class Sensor : public QWidget {
  Q_OBJECT

 public:
  explicit Sensor(QWidget* parent = nullptr);

 protected:
  void paintEvent(QPaintEvent* event);
 public:
  QWebSocket webSocket;//连接服务器
  QVBoxLayout *mainLayout;
  QWidget* parent;
  QPixmap pixmap1,pixmap2,pixmap3,pixmap4;
  QLabel *labelimg1,*labelimg2,*labelimg3,*labelimg4;
  QLabel *label1,*label2,*label3,*label4;
  QLineEdit *ipEdit,*portEdit;//ip地址和端口号
  QString batteryTopic,odomTopic;//话题
public:
  void setpicture1();
  void setpicture2();
  void setpicture3();
  void start_server();//建立连接
  void close_server();//断开链接
  void receive_battery(const QString& topic, const QString& type);//电池消息
  void receive_odom(const QString& topic, const QString& type);//接受速度状态
  void sendmessage(QString message);//发布消息
  void start_camera();//打开摄像头
  void start_map();//打开地图
  void update_pose();//更新位姿
  void begin_map();//开始建图
  void end_map();//开始建图
signals:
  void websocketConnected();//连接成功
  void websocketClosed();//连接失败
  void batterymessage(const QString& message);//电池电量
  void odommessage(const QString&message);//速度仪表盘
  void opencamera();//打开摄像头
  void openmap();//打开导航
  void updatepose();
public slots:
  void onTextMessageReceived(const QString& message);//接收到消息
};

#endif // REGISTER_H
