
#include <QApplication>
#include <QLabel>
#include <QMovie>
#include <QPixmap>
#include <QSplashScreen>
#include <QThread>
#include <csignal>
#include <iostream>
#include "logger/logger.h"
#include "runtime/application_manager.h"
void signalHandler(int signal) {
  if (signal == SIGINT) {
    QCoreApplication::quit();
  }
}
int main(int argc, char *argv[]) {
  QApplication a(argc, argv);
  // 注册信号处理函数
  std::signal(SIGINT, signalHandler);

  ApplicationManager manager_;
  LOG_INFO("qt_bridge_gui init!")
  return a.exec();
}
