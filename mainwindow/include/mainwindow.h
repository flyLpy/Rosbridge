#ifndef MAINWINDOW_H
#define MAINWINDOW_H
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
#include <QInputDialog>
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
#include <QTreeView>
#include <QWidgetAction>
#include "DockAreaWidget.h"
#include "DockManager.h"
#include "DockWidget.h"
#include "config/config_manager.h"
#include "display/manager/display_manager.h"
#include "point_type.h"
#include "widgets/dashboard.h"
#include "widgets/nav_goal_table_view.h"
#include "widgets/set_pose_widget.h"
#include "widgets/speed_ctrl.h"
#include "widgets/register.h"
#include "connect/rosbridgeclient.h"
#include "connect/camerabridge.h"
#include "connect/mapbridge.h"
#include <chrono>
#include <QMessageBox>
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  void startserver();//打开服务器
  void closeserver();//关闭服务器
  void odom();//接收小车速度话题
  void updateOdomInfo(QString message);//显示小车速度
  void battery();//订阅电池
  void show_battery(QString message);//显示电池电量
  void update_speed(const RobotSpeed &speed);//发布速度控制指令
  void startcamera();//订阅摄像头
  void show_msgs(QString message);//显示摄像头消息
  void PubNavGoal(const RobotPose &pose);//发布导航目标
  void PubRelocPose(const RobotPose &pose);//带协方差的位姿消息
  void submap();//接收地图信息
  void show_map(QString message);//显示地图
  void sub_lidar();//接受雷达话题
  void show_lidar(QString message);//显示雷达
  void sub_GlobalCostMap();//全局规划地图
  void show_GlobalCostMap(QString message);//显示全局规划地图、
  void sub_RobotPose();//订阅位置
  double calculateYawFromQuaternion(double qx, double qy, double qz, double qw);//计算偏向角
  void show_pose(QString message);
 protected:
  virtual void closeEvent(QCloseEvent *event) override;
 private:
  mapBridge *mapclient;
  Sensor* sensor;//注册界面
  ROSBridgeClient* rosClient;//服务器类
  CameraBridgeClient* cameraClient;//摄像头进程
  QProcess *process;//启动服务的进程
  QLabel *imagelabel;//摄像头图片
  QAction *SavePerspectiveAction = nullptr;
  QWidgetAction *PerspectiveListAction = nullptr;
  Ui::MainWindow *ui;
  DashBoard *speed_dash_board_;
  ads::CDockManager *dock_manager_;
  ads::CDockAreaWidget *StatusDockArea;
  ads::CDockWidget *TimelineDockWidget;
  Display::DisplayManager *display_manager_;
  QLabel *label_pos_map_;
  QLabel *label_pos_scene_;
  QThread message_thread_;
  SpeedCtrlWidget *speed_ctrl_widget_;
  NavGoalTableView *nav_goal_table_view_;
  QProgressBar *battery_bar_;
  QLabel *label_power_;
  ads::CDockAreaWidget *center_docker_area_;
  basic::OccupancyMap occ_map_;//地图
 signals:
  void OnRecvChannelData(const MsgId &id, const std::any &data);
 public slots:
  void RestoreState();
  void signalCursorPose(QPointF pos);
  void handleNavGoal(const RobotPose &pose);//发送数据
 private:
  void setupUi();
  void SaveState();
};
#endif  // MAINWINDOW_H
