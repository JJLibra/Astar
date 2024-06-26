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
#include <analysis.h>
#include <QStatusBar>
#include <QLayout>
#include <QFileDialog>
#include <QComboBox>

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 720
#define TOOLBAR_HEIGHT 30

//extern int startMap;
//extern int endMap;
extern bool israndOK;

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
    void CreateRandMap();
    void setpainter();
    void startA();
    void secondSearch();
    void clearways();
    void nextpath();
    void showmin_dfs();
    void setWindowSize();
    void openAboutDialog();
    void openAnalysis(); //打开“性能分析”窗口
    void openTime();
    void openSearch();
    void sethfunc();
    void setdir();
    void setfactor();
    void setbezier();
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
    QAction *bfsAction;//定义广度优先
    QAction *dijkstraAction;//定义Dijkstra
    QAction *acoAction;//定义蚁群算法
    QAction *gbfsAction;//定义GBFS
    QAction *nAstarEudistance;//定义传统A星欧几里得距离
    QAction *nAstarMandistance;//定义传统A星曼哈顿距离
    QAction *nAstarDiadistance;//定义传统A星切比雪夫距离
    QAction *dAstarEudistance;//定义双向A星欧几里得距离
    QAction *dAstarMandistance;//定义双向A星曼哈顿距离
    QAction *dAstarDiadistance;//定义双向A星切比雪夫距离
    QAction *yAstarEudistance;//定义优化A星欧几里得距离
    QAction *yAstarMandistance;//定义优化A星曼哈顿距离
    QAction *yAstarDiadistance;//定义优化A星切比雪夫距离
    QAction *DstarEudistance;//定义D*欧几里得距离
    QAction *DstarMandistance;//定义D*曼哈顿距离
    QAction *DstarDiadistance;//定义D*切比雪夫距离
    QAction *LPAEudistance;//定义LPA*欧几里得距离
    QAction *LPAMandistance;//定义LPA*曼哈顿距离
    QAction *LPADiadistance;//定义LPA*切比雪夫距离
    QAction *DliteEudistance;//定义Dlite欧几里得距离
    QAction *DliteMandistance;//定义Dlite曼哈顿距离
    QAction *DliteDiadistance;//定义Dlite切比雪夫距离
    QAction *safeAction;//定义考虑安全因素
    QAction *angleAction;//定义考虑拐角
    QAction *bezierAction;//绘制贝塞尔曲线
    QAction *allAction; //同时考虑
    QAction *nothingAction;//不考虑
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
