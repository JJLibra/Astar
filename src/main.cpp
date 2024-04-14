#include "mainwindow.h"
#include <QApplication>
#include <QSplashScreen>
#include <QMovie>
#include <QThread>
#include<QElapsedTimer>
#include<QGraphicsOpacityEffect>
#include<QPropertyAnimation>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

//    //启动动画
//    QMovie movie("D:/Tools/github/xxfer/img/loading3.gif");
//    QLabel *label = new QLabel();
//    label->resize(600,700);
//    label->setMovie(&movie);
//    label->setScaledContents(true); //自动适应窗口大小
//    label->setWindowFlags(Qt::FramelessWindowHint); //去除边框
//    label->setAttribute(Qt::WA_TranslucentBackground);
//    label->setAlignment(Qt::AlignCenter); //设置对齐方式为居中

//    movie.start();
//    QPropertyAnimation *animation3 = new QPropertyAnimation(label,"windowOpacity");
//    animation3->setDuration(500);
//    animation3->setStartValue(0);
//    animation3->setEndValue(1);
//    animation3->start();
//    label->show();

//    //延迟
//    QElapsedTimer t;
//    t.start();
//    while(t.elapsed() < 4200)
//    {
//        QApplication::processEvents();
//    }

    //添加Qss样式
    QFile qssFile(":/img/qss/1.qss");
    if(qssFile.open(QFile::ReadOnly)){
        a.setStyleSheet(qssFile.readAll());
    }
    qssFile.close();

    MainWindow w;
    //关闭动画
//    label->close();
    //主程序淡入
    QPropertyAnimation *animation2 = new QPropertyAnimation(&w,"windowOpacity");
    animation2->setDuration(300);
    animation2->setStartValue(0);
    animation2->setEndValue(1);
    animation2->start();
    w.show();

    return a.exec();
}
//main.cpp
