#ifndef SETXYDIALOG_H
#define SETXYDIALOG_H
#include <QSpinBox>
#include <QDialog>
#include <mainwindow.h>
#include <QGridLayout>
#include <QLabel>

class SetxyDialog : public QDialog{
     Q_OBJECT
public:
    SetxyDialog(QWidget *parent = nullptr,int xx=5, int yy=5);  //创建一个聊天对象
    int x,y;
public slots:
    void setxy();
private:
    QSpinBox *spx;
    QSpinBox *spy;
    QPushButton *setbtn;
    QGridLayout *box;
    QLabel *xtext;
    QLabel *ytext;
};


#endif // SETXYDIALOG_H
