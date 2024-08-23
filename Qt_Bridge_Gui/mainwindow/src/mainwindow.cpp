
#include "mainwindow.h"
#include <QDebug>
#include <iostream>
#include "AutoHideDockContainer.h"
#include "DockAreaTabBar.h"
#include "DockAreaTitleBar.h"
#include "DockAreaWidget.h"
#include "DockComponentsFactory.h"
#include "Eigen/Dense"
#include "FloatingDockContainer.h"
#include "algorithm.h"
#include "logger/logger.h"
#include "ui_mainwindow.h"

#include "widgets/speed_ctrl.h"
using namespace ads;
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  Q_INIT_RESOURCE(images);
  Q_INIT_RESOURCE(media);
  LOG_INFO(" MainWindow init thread id" << QThread::currentThreadId());
  qRegisterMetaType<std::string>("std::string");
  qRegisterMetaType<RobotPose>("RobotPose");
  qRegisterMetaType<RobotSpeed>("RobotSpeed");
  qRegisterMetaType<RobotState>("RobotState");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<OccupancyMap>("OccupancyMap");
  qRegisterMetaType<LaserScan>("LaserScan");
  qRegisterMetaType<RobotPath>("RobotPath");
  qRegisterMetaType<MsgId>("MsgId");
  qRegisterMetaType<std::any>("std::any");
  qRegisterMetaType<TopologyMap>("TopologyMap");
  qRegisterMetaType<TopologyMap::PointInfo>("TopologyMap::PointInfo");
  setupUi();
  QTimer::singleShot(50, [=]() { RestoreState(); });
}

MainWindow::~MainWindow() { delete ui; }
void MainWindow::setupUi() {
  ui->setupUi(this);
  CDockManager::setConfigFlag(CDockManager::OpaqueSplitterResize, true);
  CDockManager::setConfigFlag(CDockManager::XmlCompressionEnabled, false);
  CDockManager::setConfigFlag(CDockManager::FocusHighlighting, true);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasUndockButton, false);
  CDockManager::setConfigFlag(CDockManager::DockAreaHasTabsMenuButton, false);
  CDockManager::setConfigFlag(CDockManager::MiddleMouseButtonClosesTab, true);
  CDockManager::setConfigFlag(CDockManager::EqualSplitOnInsertion, true);
  CDockManager::setConfigFlag(CDockManager::ShowTabTextOnlyForActiveTab, true);
  CDockManager::setAutoHideConfigFlags(CDockManager::DefaultAutoHideConfig);
  dock_manager_ = new CDockManager(this);
  QVBoxLayout *center_layout = new QVBoxLayout();    //垂直
  QHBoxLayout *center_h_layout = new QHBoxLayout();  //水平

  ///////////////////////////////////////////////////////////////地图工具栏
  QHBoxLayout *horizontalLayout_tools = new QHBoxLayout();
  horizontalLayout_tools->setSpacing(0);
  horizontalLayout_tools->setObjectName(
      QString::fromUtf8(" horizontalLayout_tools"));

  QToolButton *reloc_btn = new QToolButton();
  reloc_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  reloc_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   background-color: lightblue;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");

  QIcon icon4;
  icon4.addFile(QString::fromUtf8(":/images/reloc2.svg"),
                QSize(64, 64), QIcon::Normal, QIcon::Off);
  reloc_btn->setIcon(icon4);
  reloc_btn->setText("重定位");
  // reloc_btn->setMaximumSize(QSize(54, 54));
  reloc_btn->setIconSize(QSize(32, 32));
  horizontalLayout_tools->addWidget(reloc_btn);
  QIcon icon5;
  icon5.addFile(QString::fromUtf8(":/images/edit.svg"),
                QSize(64, 64), QIcon::Normal, QIcon::Off);
  QToolButton *edit_map_btn = new QToolButton();
  edit_map_btn->setIcon(icon5);
  edit_map_btn->setText("编辑地图");
  // edit_map_btn->setMaximumSize(QSize(54, 54));
  edit_map_btn->setIconSize(QSize(32, 32));
  edit_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  edit_map_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   background-color: lightblue;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  horizontalLayout_tools->addWidget(edit_map_btn);
  /////////////////////////////////////////////////////////////////////////////////编辑地图

  QIcon icon6;
  icon6.addFile(QString::fromUtf8(":/images/open.svg"),
                QSize(64, 64), QIcon::Normal, QIcon::Off);
  QToolButton *open_map_btn = new QToolButton();
  open_map_btn->setIcon(icon6);
  open_map_btn->setText("打开地图");
  // open_map_btn->setMaximumSize(QSize(54, 54));
  open_map_btn->setIconSize(QSize(32, 32));
  open_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  open_map_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   background-color: lightblue;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  horizontalLayout_tools->addWidget(open_map_btn);
  QIcon icon7;
  icon7.addFile(QString::fromUtf8(":/images/save.svg"),
                QSize(64, 64), QIcon::Normal, QIcon::Off);
  QToolButton *save_map_btn = new QToolButton();
  save_map_btn->setIcon(icon7);
  save_map_btn->setText("保存地图");
  // save_map_btn->setMaximumSize(QSize(54, 54));
  save_map_btn->setIconSize(QSize(32, 32));
  save_map_btn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  save_map_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:hover {"
      "   background-color: lightblue;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  horizontalLayout_tools->addWidget(save_map_btn);
  horizontalLayout_tools->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Expanding, QSizePolicy::Minimum));
  center_layout->addLayout(horizontalLayout_tools);
  ///////////////////////////////////////////////////////////////////服务器连接
  QHBoxLayout *server_layout=new QHBoxLayout();
  QPushButton *server_start= new QPushButton("建立连接");
  QPushButton *server_close= new QPushButton("断开连接");
  QPushButton *server_camera=new QPushButton("摄像头");
  QSpacerItem *spacer_right = new QSpacerItem(100, 0, QSizePolicy::Fixed, QSizePolicy::Minimum);
  server_layout->addWidget(server_start);
  server_layout->addWidget(server_close);
  server_layout->addWidget(server_camera);
  server_layout->addItem(spacer_right);
  horizontalLayout_tools->addLayout(server_layout);
  connect(server_start,&QPushButton::clicked,this,&MainWindow::startserver);
  connect(server_close,&QPushButton::clicked,this,&MainWindow::closeserver);
  connect(server_camera,&QPushButton::clicked,this,&MainWindow::startcamera);
  ///////////////////////////////////////////////////////////////////电池电量
  battery_bar_ = new QProgressBar();
  battery_bar_->setObjectName(QString::fromUtf8("battery_bar_"));
  battery_bar_->setMaximumSize(QSize(90, 16777215));
  battery_bar_->setAutoFillBackground(true);
  battery_bar_->setStyleSheet(QString::fromUtf8(
      "QProgressBar#battery_bar_\n"
      "{\n"
      "      border:none;   /*\346\227\240\350\276\271\346\241\206*/\n"
      "      background:rgb(211, 215, 207);\n"
      "      border-radius:5px;\n"
      "      text-align:center;   "
      "/*\346\226\207\346\234\254\347\232\204\344\275\215\347\275\256*/\n"
      "      color: rgb(229, 229, 229);  "
      "/*\346\226\207\346\234\254\351\242\234\350\211\262*/\n"
      "}\n"
      " \n"
      "QProgressBar::chunk \n"
      "{\n"
      "      background-color:rgb(115, 210, 22);\n"
      "      border-radius:4px;\n"
      "}\n"
      ""));

  battery_bar_->setAlignment(Qt::AlignBottom | Qt::AlignHCenter);
  battery_bar_->setValue(0);
  horizontalLayout_tools->addWidget(battery_bar_);

  QLabel *label_11 = new QLabel();
  label_11->setObjectName(QString::fromUtf8("label_11"));
  label_11->setMinimumSize(QSize(32, 32));
  label_11->setMaximumSize(QSize(32, 32));
  label_11->setPixmap(QPixmap(QString::fromUtf8(":/images/power-v.png")));

  horizontalLayout_tools->addWidget(label_11);

  label_power_ = new QLabel();
  label_power_->setObjectName(QString::fromUtf8("label_power_"));
  label_power_->setMinimumSize(QSize(50, 32));
  label_power_->setMaximumSize(QSize(50, 32));
  label_power_->setStyleSheet(QString::fromUtf8(""));

  horizontalLayout_tools->addWidget(label_power_);
  //////////////////////////////////////////////////////////////编辑地图工具栏
  QWidget *tools_edit_map_widget = new QWidget();

  QVBoxLayout *layout_tools_edit_map = new QVBoxLayout();
  tools_edit_map_widget->setLayout(layout_tools_edit_map);
  layout_tools_edit_map->setSpacing(0);
  layout_tools_edit_map->setObjectName(
      QString::fromUtf8(" layout_tools_edit_map"));
  //地图编辑 设置鼠标按钮
  QToolButton *normal_cursor_btn = new QToolButton();
  normal_cursor_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  normal_cursor_btn->setToolTip("鼠标");
  normal_cursor_btn->setCursor(Qt::PointingHandCursor);
  normal_cursor_btn->setIconSize(QSize(32, 32));

  QIcon pose_tool_btn_icon;
  pose_tool_btn_icon.addFile(QString::fromUtf8(":/images/cursor_point_btn.svg"),
                             QSize(), QIcon::Normal, QIcon::Off);
  normal_cursor_btn->setIcon(pose_tool_btn_icon);
  layout_tools_edit_map->addWidget(normal_cursor_btn);

  //添加点位按钮

  QToolButton *add_point_btn = new QToolButton();
  add_point_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  add_point_btn->setToolTip("添加工位点");
  add_point_btn->setCursor(Qt::PointingHandCursor);
  add_point_btn->setIconSize(QSize(32, 32));

  QIcon add_point_btn_icon;
  add_point_btn_icon.addFile(QString::fromUtf8(":/images/point_btn.svg"),
                             QSize(), QIcon::Normal, QIcon::Off);
  add_point_btn->setIcon(add_point_btn_icon);
  layout_tools_edit_map->addWidget(add_point_btn);

  QToolButton *add_topology_path_btn = new QToolButton();
  add_topology_path_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  add_topology_path_btn->setToolTip("连接工位点");
  add_topology_path_btn->setCursor(Qt::PointingHandCursor);
  add_topology_path_btn->setIconSize(QSize(32, 32));

  QIcon add_topology_path_btn_icon;
  add_topology_path_btn_icon.addFile(QString::fromUtf8(":/images/topo_link_btn.svg"),
                                     QSize(), QIcon::Normal, QIcon::Off);
  add_topology_path_btn->setIcon(add_topology_path_btn_icon);
  layout_tools_edit_map->addWidget(add_topology_path_btn);
  //TODO 拓扑点连接
  add_topology_path_btn->setEnabled(false);
  //添加区域按钮
  QToolButton *add_region_btn = new QToolButton();
  add_region_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  add_region_btn->setToolTip("添加区域");
  add_region_btn->setCursor(Qt::PointingHandCursor);
  add_region_btn->setIconSize(QSize(32, 32));

  QIcon add_region_btn_icon;
  add_region_btn_icon.addFile(QString::fromUtf8(":/images/region_btn.svg"),
                              QSize(), QIcon::Normal, QIcon::Off);
  add_region_btn->setIcon(add_region_btn_icon);
  add_region_btn->setEnabled(false);
  layout_tools_edit_map->addWidget(add_region_btn);

  //分隔
  QFrame *separator = new QFrame();
  separator->setFrameShape(QFrame::HLine);
  separator->setFrameShadow(QFrame::Sunken);

  // 将分割符号添加到布局中
  layout_tools_edit_map->addWidget(separator);

  //橡皮擦按钮

  QToolButton *erase_btn = new QToolButton();
  erase_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  erase_btn->setToolTip("橡皮擦");
  erase_btn->setCursor(Qt::PointingHandCursor);
  erase_btn->setIconSize(QSize(32, 32));

  QIcon erase_btn_icon;
  erase_btn_icon.addFile(QString::fromUtf8(":/images/erase_btn.svg"),
                         QSize(), QIcon::Normal, QIcon::Off);
  erase_btn->setIcon(erase_btn_icon);
  layout_tools_edit_map->addWidget(erase_btn);
  //画笔按钮
  QToolButton *draw_pen_btn = new QToolButton();
  draw_pen_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  draw_pen_btn->setToolTip("线条");
  draw_pen_btn->setCursor(Qt::PointingHandCursor);
  draw_pen_btn->setIconSize(QSize(32, 32));

  QIcon draw_pen_btn_icon;
  draw_pen_btn_icon.addFile(QString::fromUtf8(":/images/pen.svg"),
                            QSize(), QIcon::Normal, QIcon::Off);
  draw_pen_btn->setIcon(draw_pen_btn_icon);

  layout_tools_edit_map->addWidget(draw_pen_btn);
  //线段按钮

  QToolButton *draw_line_btn = new QToolButton();
  draw_line_btn->setStyleSheet(
      "QToolButton {"
      "   border: none;"
      "   background-color: transparent;"
      "}"
      "QToolButton:pressed {"
      "   background-color: lightblue;"
      "}");
  draw_line_btn->setToolTip("线条");
  draw_line_btn->setCursor(Qt::PointingHandCursor);
  draw_line_btn->setIconSize(QSize(32, 32));

  QIcon draw_line_btn_icon;
  draw_line_btn_icon.addFile(QString::fromUtf8(":/images/line_btn.svg"),
                             QSize(), QIcon::Normal, QIcon::Off);
  draw_line_btn->setIcon(draw_line_btn_icon);

  layout_tools_edit_map->addWidget(draw_line_btn);

  layout_tools_edit_map->addItem(
      new QSpacerItem(1, 1, QSizePolicy::Minimum, QSizePolicy::Expanding));
  tools_edit_map_widget->hide();
  center_h_layout->addWidget(tools_edit_map_widget);
  center_layout->addLayout(center_h_layout);

  /////////////////////////////////////////////////////////////////////////地图显示

  display_manager_ = new Display::DisplayManager();
  center_h_layout->addWidget(display_manager_->GetViewPtr());

  //////////////////////////////////////////////////////////////////////////坐标显示
  QHBoxLayout *horizontalLayout_12 = new QHBoxLayout();
  horizontalLayout_12->setObjectName(QString::fromUtf8("horizontalLayout_12"));
  QLabel *label = new QLabel();
  label->setText("map:");
  label->setObjectName(QString::fromUtf8("label"));

  horizontalLayout_12->addWidget(label);

  label_pos_map_ = new QLabel();
  label_pos_map_->setObjectName(QString::fromUtf8("label_pos_map_"));
  label_pos_map_->setStyleSheet(QString::fromUtf8(""));

  horizontalLayout_12->addWidget(label_pos_map_);

  QLabel *label_5 = new QLabel();
  label_5->setText("scene:");

  label_5->setObjectName(QString::fromUtf8("label_5"));

  horizontalLayout_12->addWidget(label_5);

  label_pos_scene_ = new QLabel();
  label_pos_scene_->setObjectName(QString::fromUtf8("label_pos_scene_"));

  horizontalLayout_12->addWidget(label_pos_scene_);

  horizontalLayout_12->addItem(
      new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));

  center_layout->addLayout(horizontalLayout_12);

  /////////////////////////////////////////////////中心主窗体
  QWidget *center_widget = new QWidget();

  center_widget->setLayout(center_layout);
  CDockWidget *CentralDockWidget = new CDockWidget("CentralWidget");
  CentralDockWidget->setWidget(center_widget);
  center_docker_area_ = dock_manager_->setCentralWidget(CentralDockWidget);
  center_docker_area_->setAllowedAreas(DockWidgetArea::OuterDockAreas);

  //////////////////////////////////////////////////////////速度仪表盘
  ads::CDockWidget *DashBoardDockWidget = new ads::CDockWidget("速度仪表盘");
  QWidget *speed_dashboard_widget = new QWidget();
  // speed_dashboard_widget->setMinimumSize(QSize(300, 300));
  DashBoardDockWidget->setWidget(speed_dashboard_widget);
  speed_dash_board_ = new DashBoard(speed_dashboard_widget);
  auto dashboard_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::LeftDockWidgetArea,
                                   DashBoardDockWidget, center_docker_area_);
  ui->menuView->addAction(DashBoardDockWidget->toggleViewAction());

  ////////////////////////////////////////////////////////速度控制
  speed_ctrl_widget_ = new SpeedCtrlWidget();
  connect(speed_ctrl_widget_, &SpeedCtrlWidget::signalControlSpeed,
          [this](const RobotSpeed &speed) {
            update_speed(speed);
          });
  ads::CDockWidget *SpeedCtrlDockWidget = new ads::CDockWidget("速度控制");
  SpeedCtrlDockWidget->setWidget(speed_ctrl_widget_);
  auto speed_ctrl_area =
      dock_manager_->addDockWidget(ads::DockWidgetArea::BottomDockWidgetArea,
                                   SpeedCtrlDockWidget, dashboard_area);
  ui->menuView->addAction(SpeedCtrlDockWidget->toggleViewAction());
  /////////////////////////////////////////////////////////导航任务列表
  QWidget *task_list_widget = new QWidget();
  nav_goal_table_view_ = new NavGoalTableView();
  QVBoxLayout *horizontalLayout_13 = new QVBoxLayout();
  horizontalLayout_13->addWidget(nav_goal_table_view_);
  task_list_widget->setLayout(horizontalLayout_13);
  ads::CDockWidget *nav_goal_list_dock_widget = new ads::CDockWidget("导航任务栏");
  QPushButton *btn_add_one_goal = new QPushButton("Add Point");
  QHBoxLayout *horizontalLayout_15 = new QHBoxLayout();
  QPushButton *btn_start_task_chain = new QPushButton("Start Task Chain");
  QCheckBox *loop_task_checkbox = new QCheckBox("Loop Task");
  QHBoxLayout *horizontalLayout_14 = new QHBoxLayout();
  horizontalLayout_15->addWidget(btn_add_one_goal);
  horizontalLayout_14->addWidget(btn_start_task_chain);
  horizontalLayout_14->addWidget(loop_task_checkbox);
  QPushButton *btn_load_task_chain = new QPushButton("Load Task Chain");
  QPushButton *btn_save_task_chain = new QPushButton("Save Task Chain");
  QHBoxLayout *horizontalLayout_16 = new QHBoxLayout();
  horizontalLayout_16->addWidget(btn_load_task_chain);
  horizontalLayout_16->addWidget(btn_save_task_chain);

  horizontalLayout_13->addLayout(horizontalLayout_15);
  horizontalLayout_13->addLayout(horizontalLayout_14);
  horizontalLayout_13->addLayout(horizontalLayout_16);
  nav_goal_list_dock_widget->setWidget(task_list_widget);
  nav_goal_list_dock_widget->setMinimumSizeHintMode(
      CDockWidget::MinimumSizeHintFromDockWidget);
  nav_goal_list_dock_widget->setMinimumSize(200, 150);
  nav_goal_list_dock_widget->setMaximumSize(480, 9999);
  dock_manager_->addDockWidget(ads::DockWidgetArea::RightDockWidgetArea,
                               nav_goal_list_dock_widget, center_docker_area_);
  nav_goal_list_dock_widget->toggleView(false);
  // nav_goal_list_dock_widget->toggleView(false);
  ui->menuView->addAction(nav_goal_list_dock_widget->toggleViewAction());
  /////////////////////////////////////////////////////////////////////////////////////////摄像头
      ads::CDockWidget *dock_widget = new ads::CDockWidget("摄像头");
      QWidget *imageWidget = new QWidget();
      QVBoxLayout *bottomLayout = new QVBoxLayout(imageWidget);
      imagelabel = new QLabel(imageWidget);
      imagelabel->setFixedWidth(dock_widget->width());
      imagelabel->setFixedHeight(dock_widget->height());
      bottomLayout->addWidget(imagelabel);
      imagelabel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      imageWidget->setLayout(bottomLayout);
      dock_widget->setWidget(imageWidget);
      dock_widget->setMinimumHeight(400);
      dock_manager_->addDockWidget(ads::DockWidgetArea::BottomDockWidgetArea,
                                   dock_widget,
                                   center_docker_area_);
      dock_widget->toggleView(true);
      ui->menuView->addAction(dock_widget->toggleViewAction());
  /////////////////////////////////////////////////////////////////////////////////////////////槽连接
  //重定位
      connect(reloc_btn, &QToolButton::clicked,
                [this]() { display_manager_->StartReloc(); });
  //打开地图
      connect(open_map_btn, &QToolButton::clicked, [this]() {
        QStringList filters;
        filters
            << "地图(*.yaml)"
            << "拓扑地图(*.topology)";

        QString fileName = QFileDialog::getOpenFileName(nullptr, "OPen Map files",
                                                        "", filters.join(";;"));
        if (!fileName.isEmpty()) {
          // 用户选择了文件夹，可以在这里进行相应的操作
          LOG_INFO("用户选择的打开地图路径：" << fileName.toStdString());
          display_manager_->OpenMap(fileName.toStdString());
        } else {
          // 用户取消了选择
          LOG_INFO("取消打开地图");
        }
      });
   //编辑地图
   connect(edit_map_btn, &QToolButton::clicked, [this, tools_edit_map_widget, edit_map_btn]() {
      if (edit_map_btn->text() == "编辑地图") {
        display_manager_->SetEditMapMode(Display::MapEditMode::kNormal);
        edit_map_btn->setText("结束编辑");
        tools_edit_map_widget->show();
      } else {
        display_manager_->SetEditMapMode(Display::MapEditMode::kStop);
        edit_map_btn->setText("编辑地图");
        tools_edit_map_widget->hide();
      }
    });
   //保存地图
   connect(save_map_btn, &QToolButton::clicked, [this]() {
   QString fileName = QFileDialog::getSaveFileName(nullptr, "Save Map files",
                                                   "", "Map files (*.yaml,*.pgm,*.pgm.json)");
   if (!fileName.isEmpty()) {
     // 用户选择了文件夹，可以在这里进行相应的操作
     LOG_INFO("用户选择的保存地图路径：" << fileName.toStdString());
     display_manager_->SaveMap(fileName.toStdString());
   } else {
     LOG_INFO("取消保存地图");
   }
 });

}
void MainWindow::RestoreState() {
  QSettings settings("state.ini", QSettings::IniFormat);
  this->restoreGeometry(settings.value("mainWindow/Geometry").toByteArray());
  this->restoreState(settings.value("mainWindow/State").toByteArray());
  dock_manager_->loadPerspectives(settings);
  dock_manager_->openPerspective("history");
}
void MainWindow::closeEvent(QCloseEvent *event) {
  // Delete dock manager here to delete all floating widgets. This ensures
  // that all top level windows of the dock manager are properly closed
  // write state
  SaveState();
  dock_manager_->deleteLater();
  QMainWindow::closeEvent(event);
  LOG_INFO("qt_bridge_gui close!");
}
void MainWindow::SaveState() {
  QSettings settings("state.ini", QSettings::IniFormat);
  settings.setValue("mainWindow/Geometry", this->saveGeometry());
  settings.setValue("mainWindow/State", this->saveState());
  dock_manager_->addPerspective("history");
  dock_manager_->savePerspectives(settings);
}
//连接服务器
void MainWindow::startserver(){
    QString command = "ssh";
    QStringList args;
    args << "firefly@192.168.8.62" << "source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=62 && ros2 launch rosbridge_server rosbridge_websocket_launch.xml";
    // 创建QProcess对象
    process = new QProcess();
    // 启动SSH命令
    process->start(command, args);
    // 获取命令输出
    QByteArray output = process->readAllStandardOutput();
    qDebug() << "SSH process output:" << output;
    // init client
    rosClient = new ROSBridgeClient(this);
    ///////////////////////////////////////////////////////////////////////////////////////////////连接服务器就自动订阅话题
    connect(rosClient,&ROSBridgeClient::serverconnected,this,&MainWindow::odom);
    connect(rosClient,&ROSBridgeClient::serverconnected,this,&MainWindow::battery);
}
//断开服务器
void MainWindow::closeserver() {
    if (process) {  // 检查进程对象是否存在
        // 结束进程
        process->terminate();  // 请求进程终止
        if (!process->waitForFinished(3000)) {  // 等待最多 3 秒
            process->kill();  // 强制终止进程
        }
        delete process;  // 释放内存
        process = nullptr;  // 清空指针
    }

    if (rosClient) {
        delete rosClient;  // 释放 ROSBridgeClient 对象
        rosClient = nullptr;  // 清空指针
    }

    qDebug() << "Server process closed.";
}
void MainWindow::odom(){
    QString topic="/odom";
    QString type="nav_msgs/msg/Odometry";
    rosClient->receive_odom(topic,type);
    connect(rosClient, &ROSBridgeClient::odommessage, this, &MainWindow::updateOdomInfo);
}

//显示速度
void MainWindow::updateOdomInfo(QString message) {
    QJsonDocument jsonDoc = QJsonDocument::fromJson(message.toUtf8());
    if (!jsonDoc.isNull() && jsonDoc.isObject()) {
        QJsonObject jsonObj = jsonDoc.object();
        QJsonObject msgObj = jsonObj.value("msg").toObject(); // 提取 msg 部分

        // 提取第一个 twist
        QJsonObject twistObj = msgObj.value("twist").toObject(); // 提取顶级 twist
        QJsonObject innerTwistObj = twistObj.value("twist").toObject(); // 提取内部 twist
        QJsonObject linearObj = innerTwistObj.value("linear").toObject(); // 提取 linear 部分
        double linearX = linearObj.value("x").toDouble(); // 提取 x 方向的线速度
        speed_dash_board_->set_speed(abs(linearX * 100));
        if (linearX > 0.001) {
          speed_dash_board_->set_gear(DashBoard::kGear_D);
        } else if (linearX < -0.001) {
          speed_dash_board_->set_gear(DashBoard::kGear_R);
        } else {
          speed_dash_board_->set_gear(DashBoard::kGear_N);
        }
    }
}
//订阅电池电量
void MainWindow::battery(){
    QString topic = "/battery";
    QString type = "sensor_msgs/msg/BatteryState";
    rosClient->battery(topic, type);
    connect(rosClient, &ROSBridgeClient::batterymessage, this, &MainWindow::show_battery);
}
//电池电量显示
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
            label_power_->setText(QString::number(batteryVoltage, 'f', 2) + "V");
            battery_bar_->setValue(batteryPercentage);
        }
}
//发布速度控制指令
void MainWindow::update_speed(const RobotSpeed &speed){
    QJsonObject twist_msg;
       QJsonObject linear_obj;
       linear_obj["x"] = speed.vx;
       linear_obj["y"] = speed.vy;
       linear_obj["z"] = 0;

       QJsonObject angular_obj;
       angular_obj["x"] = 0;
       angular_obj["y"] = 0;
       angular_obj["z"] = speed.w;

       twist_msg["linear"] = linear_obj;
       twist_msg["angular"] = angular_obj;

       // 创建完整的 ROSBridge 消息对象
       QJsonObject rosbridge_msg;
       rosbridge_msg["op"] = "publish";
       rosbridge_msg["topic"] = "/cmd_vel"; // 需要根据实际情况修改 topic 名称
       rosbridge_msg["msg"] = twist_msg;

       // 将 JSON 对象转换为字符串
       QJsonDocument doc(rosbridge_msg);
       QString json_str = doc.toJson(QJsonDocument::Compact);
       rosClient->sendspeed(json_str);
}
void MainWindow::startcamera(){
    QString topic = "/camera/color/image_raw/compressed";
    QString type = "sensor_msgs/msg/CompressedImage";
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
    QString format = msgObj.value("format").toString();
    QByteArray data = QByteArray::fromBase64(msgObj.value("data").toString().toUtf8());

    // 解码压缩的图像数据
    QBuffer buffer(&data);
    buffer.open(QIODevice::ReadOnly);
    QImageReader reader(&buffer);
    QImage image = reader.read();

    // 检查图像是否成功解码
    if (!image.isNull()) {
        // 显示图像
        imagelabel->setPixmap(QPixmap::fromImage(image));
    } else {
        qWarning() << "解码压缩图像失败。";
    }
}
