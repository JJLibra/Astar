#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QLabel>
#include <QPainter>
#include <maplabel.h>
#include <QToolBar>
#include <QMenuBar>
#include <QDialog>
#include <QDebug>
#include <QInputDialog>
#include <QPushButton>
#include <setxyDialog.h>
#include <astar.h>
#include <QRadioButton>
#include <QVBoxLayout>
#include <QGraphicsDropShadowEffect>
#include <QScrollArea>
#include <QToolButton>
#include <QButtonGroup>
#include <QTimer>
#include <about.h>
#include <QStatusBar>
#include <QLayout>
#include <QFileDialog>
#include <QComboBox>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define TOOLBAR_HEIGHT 30

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void setMenuBar();
    void setToolBar();
    void initmainWindow();
    void setStatusBar();
    void setStatusBar(int x,int y,int dir,int hfunc);
    Astar* map;

private slots:
    void openxyDialog();
    void clearmap();
    void setpainter();
    void startA();
    void clearways();
    void nextpath();
    void setWindowSize();
    void openAboutDialog();
    void sethfunc();
    void setdir();
    void setfactor();
    void setbackpic();
    void clearbackpic();
    void setRect(int);
    void savemap();
    void loadmap();
private:
    int mapWidth;
    int mapLength;
    QDialog *xyDialog;
    QToolButton* startbtn;
    QToolButton* endbtn;
    QToolButton* blocksbtn;
    QButtonGroup* btngroup;
    QScrollArea* mainarea;
    QTimer* timer;
    QAction *dfsAction;//定义深度优先
    QAction *mindfsAction;//定义深度优先
    QAction *bfsAction;//定义广度优先
    QAction *normalAction;//定义传统A星
    QAction *safeAction;//定义考虑安全因素
    QAction *angleAction;//定义考虑拐角
    QAction *allAction;
    QAction *nothingAction;//不考虑
    QAction *euAction;
    QAction *manAction;
    QAction *diaAction;
    QAction *set4dir;
    QAction *set8dir;
    QLabel *widthStatus;
    QLabel *heightStatus;
    QLabel *dirStatus;
    QLabel *hfuncStatus;
    QImage pic;
    QComboBox* setrectsize;
    bool isbgset;
    int rectl[10];
};

#endif // MAINWINDOW_H
