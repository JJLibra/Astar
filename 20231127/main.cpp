#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //添加Qss样式
    QFile qssFile(":/img/qss/1.qss");
    if(qssFile.open(QFile::ReadOnly)){
        a.setStyleSheet(qssFile.readAll());
    }
    qssFile.close();

    MainWindow w;
    w.show();
    return a.exec();
}
//main.cpp
