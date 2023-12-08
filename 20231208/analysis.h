#ifndef ANALYSIS_H
#define ANALYSIS_H
#include <QProcess>
#include <QApplication>
#include<QDialog>
#include<QVBoxLayout>
#include<QHBoxLayout>
#include<QLayout>
#include<QLabel>
#include<QIcon>
#include<QDebug>
#include<QPixmap>
#include<QPicture>
#include<QPainterPath>
#include <QPushButton>
#include <QToolBar>
#include <QToolButton>

#include <QWidget>
//#include<mainwindow.h>

class analysis : public QDialog{
public:
    analysis(const QString &text, QWidget *parent = nullptr);
    double DijkstraTime;

protected:
    QPixmap Analysismappic;
    QLabel *Analysismapl;
};

#endif // ANALYSIS_H
