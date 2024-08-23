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
#include "rosbridgeclient.h"
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
 protected:
  virtual void closeEvent(QCloseEvent *event) override;
 private:
  ROSBridgeClient* rosClient;//服务器类
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
 signals:
  void OnRecvChannelData(const MsgId &id, const std::any &data);
 public slots:
    void RestoreState();
 private:
  void setupUi();
  void SaveState();
};
#endif  // MAINWINDOW_H
