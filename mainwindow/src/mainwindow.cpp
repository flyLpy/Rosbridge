
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

  QPushButton *btn_add_one_goal = new QPushButton("添加点");
  QHBoxLayout *horizontalLayout_15 = new QHBoxLayout();
  QPushButton *btn_start_task_chain = new QPushButton("开始任务链");
  QCheckBox *loop_task_checkbox = new QCheckBox("循环任务");
  QHBoxLayout *horizontalLayout_14 = new QHBoxLayout();
  horizontalLayout_15->addWidget(btn_add_one_goal);
  horizontalLayout_14->addWidget(btn_start_task_chain);
  horizontalLayout_14->addWidget(loop_task_checkbox);
  QPushButton *btn_load_task_chain = new QPushButton("加载任务链");
  QPushButton *btn_save_task_chain = new QPushButton("保存任务链");
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
  nav_goal_list_dock_widget->toggleView(true);
  // nav_goal_list_dock_widget->toggleView(false);
  ui->menuView->addAction(nav_goal_list_dock_widget->toggleViewAction());
  //连接信号和槽
  //发布位置信息
  //connect(nav_goal_table_view_, &NavGoalTableView::signalSendNavGoal, [this](const RobotPose &pose) {
    //   PubNavGoal(pose);
   //});
  connect(nav_goal_table_view_, &NavGoalTableView::signalSendNavGoal,
          this, [this](const RobotPose &pose) {
              // 使用 Qt::QueuedConnection 来确保 PubNavGoal(pose) 在主线程中执行
              QMetaObject::invokeMethod(this, [this, pose]() {
                  PubNavGoal(pose);
              }, Qt::QueuedConnection);
          });

  //加载任务链
  connect(btn_load_task_chain, &QPushButton::clicked, [this]() {
       QString fileName = QFileDialog::getOpenFileName(nullptr, "Open JSON file", "", "JSON files (*.json)");
       if (!fileName.isEmpty()) {
           qDebug() << "Selected file:" << fileName;
           nav_goal_table_view_->LoadTaskChain(fileName.toStdString());
       }
   });
  //保存任务链
  connect(btn_save_task_chain, &QPushButton::clicked, [this]() {
        QString fileName = QFileDialog::getSaveFileName(nullptr, "Save JSON file", "", "JSON files (*.json)");
        if (!fileName.isEmpty()) {
            qDebug() << "Selected file:" << fileName;
            if (!fileName.endsWith(".json")) {
                fileName += ".json";
            }
            nav_goal_table_view_->SaveTaskChain(fileName.toStdString());
        }
    });
  //添加点
  connect(btn_add_one_goal, &QPushButton::clicked, [this, nav_goal_list_dock_widget]() {
        nav_goal_table_view_->AddItem();
    });
  //开始任务链
  connect(btn_start_task_chain, &QPushButton::clicked, [this, btn_start_task_chain, loop_task_checkbox]() {
      if (btn_start_task_chain->text() == "Start Task Chain") {
          btn_start_task_chain->setText("Stop Task Chain");
          nav_goal_table_view_->StartTaskChain(loop_task_checkbox->isChecked());
      } else {
          btn_start_task_chain->setText("Start Task Chain");
          nav_goal_table_view_->StopTaskChain();
      }
  });
  connect(nav_goal_table_view_, &NavGoalTableView::signalTaskFinish, [this, btn_start_task_chain]() {
        LOG_INFO("task finish!");
        btn_start_task_chain->setText("Start Task Chain");
    });
  connect(display_manager_, SIGNAL(signalTopologyMapUpdate(const TopologyMap &)), nav_goal_table_view_, SLOT(UpdateTopologyMap(const TopologyMap &)));

  connect(display_manager_, SIGNAL(signalCurrentSelectPointChanged(const TopologyMap::PointInfo &)), nav_goal_table_view_, SLOT(UpdateSelectPoint(const TopologyMap::PointInfo &)));
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

  /////////////////////////////////////////////////////////////////////////////////////////////注册
      ads::CDockWidget* sensorDockWidget1 = new ads::CDockWidget("注册");
      QWidget* sensorWidget1 = new QWidget();
      sensor = new Sensor(sensorWidget1);
      sensorDockWidget1->setWidget(sensorWidget1);
      auto sensorArea = dock_manager_->addDockWidget(
          ads::DockWidgetArea::RightDockWidgetArea,
          sensorDockWidget1,
          center_docker_area_
      );
      ui->menuView->addAction(sensorDockWidget1->toggleViewAction());
      ///////////////////////////////////////////////////////////////////服务器连接
      connect(sensor,&Sensor::websocketConnected,this,&MainWindow::startserver);
      connect(sensor,&Sensor::websocketClosed,this,&MainWindow::closeserver);
      connect(sensor,&Sensor::opencamera,this,&MainWindow::startcamera);
      connect(sensor,&Sensor::openmap,this,&MainWindow::submap);
      connect(sensor,&Sensor::openmap,this,&MainWindow::sub_lidar);
      connect(sensor,&Sensor::openmap,this,&MainWindow::sub_GlobalCostMap);
      connect(sensor,&Sensor::openmap,this,&MainWindow::sub_RobotPose);
  /////////////////////////////////////////////////////////////////////////////////////////////槽连接
      connect(display_manager_, &Display::DisplayManager::signalPub2DPose,
                [this](const RobotPose &pose) {
                  PubRelocPose(pose);
                });
     //   connect(display_manager_, &Display::DisplayManager::signalPub2DGoal,
            //    [this](const RobotPose &pose) {
               //   PubNavGoal(pose);
             //  });
        connect(display_manager_, &Display::DisplayManager::signalPub2DGoal,
                this, [this](const RobotPose &pose) {
                    // 使用 QMetaObject::invokeMethod 确保 PubNavGoal 在主线程中执行
                    QMetaObject::invokeMethod(this, [this, pose]() {
                        PubNavGoal(pose);
                    }, Qt::QueuedConnection);
                });
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
   connect(add_point_btn, &QToolButton::clicked, [this]() {
      display_manager_->SetEditMapMode(Display::MapEditMode::kAddPoint);
    });
    connect(normal_cursor_btn, &QToolButton::clicked, [this]() { display_manager_->SetEditMapMode(Display::MapEditMode::kNormal); });
    connect(erase_btn, &QToolButton::clicked, [this]() { display_manager_->SetEditMapMode(Display::MapEditMode::kErase); });
    connect(draw_line_btn, &QToolButton::clicked, [this]() { display_manager_->SetEditMapMode(Display::MapEditMode::kDrawLine); });
    connect(draw_pen_btn, &QToolButton::clicked, [this]() { display_manager_->SetEditMapMode(Display::MapEditMode::kDrawWithPen); });
    connect(display_manager_->GetDisplay(DISPLAY_MAP),
             SIGNAL(signalCursorPose(QPointF)), this,
             SLOT(signalCursorPose(QPointF)));
}
void MainWindow::signalCursorPose(QPointF pos) {
  basic::Point mapPos =
      display_manager_->mapPose2Word(basic::Point(pos.x(), pos.y()));
  label_pos_map_->setText("( x:" + QString::number(mapPos.x).mid(0, 4) +
                          " y:" + QString::number(mapPos.y).mid(0, 4) + ") ");
  label_pos_scene_->setText("(x:" + QString::number(pos.x()).mid(0, 4) +
                            " y:" + QString::number(pos.y()).mid(0, 4) + ")");
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
    QString ipaddress,port;
    ipaddress = sensor->ipEdit->text();
    port = sensor->portEdit->text();
    cameraClient=new CameraBridgeClient(this,ipaddress,port);
    rosClient = new ROSBridgeClient(this,ipaddress,port);
    mapclient=new mapBridge(this,ipaddress,port);
    ///////////////////////////////////////////////////////////////////////////////////////////////连接服务器就自动订阅话题
    odom();
    battery();
}
//断开服务器
void MainWindow::closeserver() {
    if (rosClient) {
        delete rosClient;  // 释放 ROSBridgeClient 对象
        rosClient = nullptr;  // 清空指针
    }
    if (cameraClient) {
        delete cameraClient;
        cameraClient = nullptr;
    }
    if(sensor){
        delete sensor;
        sensor=nullptr;
    }
    if(mapclient){
        delete mapclient;
        mapclient=nullptr;
    }
    QMessageBox::information(this,"Notification","Server process closed");
}
void MainWindow::odom(){
    QString topic="/odom";
    QString type="nav_msgs/msg/Odometry";
    sensor->receive_odom(topic,type);
    connect(sensor, &Sensor::odommessage, this, &MainWindow::updateOdomInfo);
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
    sensor->receive_battery(topic, type);
    connect(sensor, &Sensor::batterymessage, this, &MainWindow::show_battery);
}
//电池电量显示
void MainWindow::show_battery(QString message){
     sensor->setpicture1();
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
       sensor->sendmessage(json_str);
}
void MainWindow::startcamera(){
    QString topic = "/camera/color/image_raw/compressed";
    QString type = "sensor_msgs/msg/CompressedImage";
    cameraClient->subscribecameraTopic(topic, type);
    connect(cameraClient, &CameraBridgeClient::cameramessageReceived, this, &MainWindow::show_msgs);
}
//显示摄像头的实现
void MainWindow::show_msgs(QString message){
    sensor->setpicture3();
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
void MainWindow::PubNavGoal(const RobotPose &pose) {
    double x = pose.x;
    double y = pose.y;
    double theta = pose.theta;
    // 创建 JSON 对象
    QJsonObject geo_pose;
    // 设置 header
    QJsonObject header;
    header["frame_id"] = "map";
    // 使用 std::chrono 获取的时间戳
    auto now = std::chrono::steady_clock::now();
    auto time_since_epoch = now.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch).count();
    header["stamp"] = QString::number(nanoseconds);

    QJsonObject position;
    position["x"] = x;
    position["y"] = y;
    position["z"] = 0;
    // 使用 Eigen 计算四元数
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    quaternion = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) * quaternion;

    QJsonObject orientation;
    orientation["x"] = quaternion.x();
    orientation["y"] = quaternion.y();
    orientation["z"] = quaternion.z();
    orientation["w"] = quaternion.w();

    // 将各部分添加到 geo_pose 对象中
    QJsonObject poseObject;
    poseObject["position"] = position;
    poseObject["orientation"] = orientation;
    geo_pose["header"] = header;
    geo_pose["pose"] = poseObject;


    QJsonObject rosbridge_msg;
        rosbridge_msg["op"] = "publish";
        rosbridge_msg["topic"] = "/goal_pose"; // 替换为实际的 ROS 话题名
        rosbridge_msg["msg"] = geo_pose;
    qDebug()<<geo_pose<<"----------------------------------------------";
        // 将 JSON 对象转换为 QString
        QJsonDocument doc(rosbridge_msg);
        QString rosbridgeMessage = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
      for(int i=0;i<100;i++)
    rosClient->sendmessage(rosbridgeMessage);
}
void MainWindow::handleNavGoal(const RobotPose &pose){
    PubNavGoal(pose);
}
void MainWindow::PubRelocPose(const RobotPose &pose) {
    double x = pose.x;
    double y = pose.y;
    double theta = pose.theta;

    // 创建 JSON 对象
    QJsonObject geo_pose;

    // 设置 header
    QJsonObject header;
    header["frame_id"] = "map";

    // 使用 std::chrono 获取的时间戳
    auto now = std::chrono::steady_clock::now();
    auto time_since_epoch = now.time_since_epoch();
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_since_epoch).count();
    header["stamp"] = QString::number(nanoseconds);

    // 设置 position
    QJsonObject position;
    position["x"] = x;
    position["y"] = y;
    position["z"] = 0; // PoseWithCovarianceStamped 默认有 z 维度，通常设置为 0

    // 使用 Eigen 计算四元数
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    quaternion = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ()) * quaternion;

    QJsonObject orientation;
    orientation["x"] = quaternion.x();
    orientation["y"] = quaternion.y();
    orientation["z"] = quaternion.z();
    orientation["w"] = quaternion.w();

    // 设置协方差（PoseWithCovarianceStamped 默认包含协方差）
    QJsonArray covariance;
    for (int i = 0; i < 36; ++i) { // PoseWithCovarianceStamped 协方差矩阵是 6x6
        covariance.append(0); // 这里初始化为 0，实际应用中需要填充实际协方差值
    }
    QJsonObject poseObject;
    poseObject["position"] = position;
    poseObject["orientation"] = orientation;
    poseObject["covariance"] = covariance;

    // 将各部分添加到 geo_pose 对象中
    geo_pose["header"] = header;
    geo_pose["pose"] = poseObject;

    // 将 JSON 对象转换为 QString
    QJsonObject rosbridge_msg;
        rosbridge_msg["op"] = "publish";
        rosbridge_msg["topic"] = "/initialpose"; // 替换为实际的 ROS 话题名
        rosbridge_msg["msg"] = geo_pose;

        // 将 JSON 对象转换为 QString
        QJsonDocument doc(rosbridge_msg);
        QString rosbridgeMessage = QString::fromUtf8(doc.toJson(QJsonDocument::Compact));
    // 发送消息
    rosClient->sendmessage(rosbridgeMessage);
}
void MainWindow::submap(){
    QString topic = "/map";
    QString type = "nav_msgs/msg/OccupancyGrid";
    mapclient->receive_map(topic, type);
    connect(mapclient, &mapBridge::mapmessage, this, &MainWindow::show_map);
}
void MainWindow::show_map(QString message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject jsonObj = doc.object()["msg"].toObject();
        QJsonObject info = jsonObj["info"].toObject();
        double origin_x = info["origin"].toObject()["position"].toObject()["x"].toDouble();
        double origin_y = info["origin"].toObject()["position"].toObject()["y"].toDouble();
        int width = info["width"].toInt();
        int height = info["height"].toInt();
        double resolution = info["resolution"].toDouble();
        qDebug()<<width<<"          "<<height;
        // 创建OccupancyMap对象
        occ_map_ = basic::OccupancyMap(
            height, width, Eigen::Vector3d(origin_x, origin_y, 0), resolution);
        // 填充地图数据
        QJsonArray data = jsonObj["data"].toArray();
        for (int i = 0; i < data.size(); i++) {
            int x = int(i / width);
            int y = i % width;
            occ_map_(x, y) = data[i].toInt();
        }
        // 设置地图方向并触发回调
        occ_map_.SetFlip();
        display_manager_->UpdateTopicData(MsgId::kOccupancyMap, occ_map_);//订阅激光往这里传
}
void MainWindow::sub_lidar(){
    QString topic = "/scan";
    QString type = "sensor_msgs/msg/LaserScan";
    mapclient->reveive_lidar(topic, type);
    connect(mapclient, &mapBridge::lidarmessage, this, &MainWindow::show_lidar);
}
void MainWindow::show_lidar(QString message) {
    sensor->setpicture2();
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject obj = doc.object();

    // 从嵌套的 "msg" 对象中提取数据
    QJsonObject msgObj = obj["msg"].toObject();

    double angle_min = msgObj["angle_min"].toDouble();
    double angle_max = msgObj["angle_max"].toDouble();
    double angle_increment = msgObj["angle_increment"].toDouble();
    QJsonArray rangesArray = msgObj["ranges"].toArray();

    basic::LaserScan laser_points;
    for (int i = 0; i < rangesArray.size(); ++i) {
        double angle = angle_min + i * angle_increment;
        double dist = rangesArray[i].toDouble();
        if (std::isinf(dist)) continue;

        double x = dist * std::cos(angle);
        double y = dist * std::sin(angle);

        basic::Point p;
        p.x = x;
        p.y = y;
        laser_points.push_back(p);
    }

    laser_points.id = 0;
    display_manager_->UpdateTopicData(MsgId::kLaserScan, laser_points);
}
void MainWindow::sub_GlobalCostMap(){
    QString topic = "/global_costmap/costmap";
    QString type = "nav_msgs/msg/OccupancyGrid";
    mapclient->reveive_GlobalCostMap(topic, type);
    connect(mapclient, &mapBridge::GlobalCostMapmessage, this, &MainWindow::show_GlobalCostMap);
}
void MainWindow::show_GlobalCostMap(QString message) {
    // 解析 JSON 消息
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    QJsonObject obj = doc.object();
    // 从嵌套的 "msg" 对象中提取数据
    QJsonObject msgObj = obj["msg"].toObject();
    // 提取网格信息
    QJsonObject infoObj = msgObj["info"].toObject();
    int width = infoObj["width"].toInt();
    int height = infoObj["height"].toInt();
    double resolution = infoObj["resolution"].toDouble();
    QJsonObject originObj = infoObj["origin"].toObject();
    QJsonObject positionObj = originObj["position"].toObject();
    double origin_x = positionObj["x"].toDouble();
    double origin_y = positionObj["y"].toDouble();

    // 初始化 OccupancyMap
    basic::OccupancyMap cost_map(height, width,
                                 Eigen::Vector3d(origin_x, origin_y, 0),
                                 resolution);
    QJsonArray dataArray = msgObj["data"].toArray();
    for (int i = 0; i < dataArray.size(); ++i) {
        int x = i / width;
        int y = i % width;
        cost_map(x, y) = dataArray[i].toInt();
    }

    cost_map.SetFlip();
    display_manager_->UpdateTopicData(MsgId::kGlobalCostMap,cost_map);
}
void MainWindow::sub_RobotPose(){
    QString topic = "/robot_pose";
    QString type = "geometry_msgs/PoseStamped";
    mapclient->reveive_Pose(topic, type);
    connect(mapclient, &mapBridge::Posemessage, this, &MainWindow::show_pose);
}
double MainWindow::calculateYawFromQuaternion(double qx, double qy, double qz, double qw) {
    // 使用公式将四元数转换为yaw角
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    return std::atan2(siny_cosp, cosy_cosp);
}
void MainWindow::show_pose(QString message){
    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
        QJsonObject obj = doc.object();

        // 检查是否包含 "msg" 字段
        if (obj.contains("msg")) {
            QJsonObject msg = obj["msg"].toObject();

            // 提取位姿中的位移信息 (x, y) 和四元数信息
            double x = msg["pose"].toObject()["position"].toObject()["x"].toDouble();
            double y = msg["pose"].toObject()["position"].toObject()["y"].toDouble();

            QJsonObject orientation = msg["pose"].toObject()["orientation"].toObject();
            double qx = orientation["x"].toDouble();
            double qy = orientation["y"].toDouble();
            double qz = orientation["z"].toDouble();
            double qw = orientation["w"].toDouble();

            // 将四元数转换为yaw角（假设机器人是在2D平面运动）
            double yaw = calculateYawFromQuaternion(qx, qy, qz, qw);
            basic::RobotPose ret;
            ret.x=x;
            ret.y=y;
            ret.theta=yaw;
            display_manager_->UpdateTopicData(MsgId::kRobotPose,ret);
    }
}
