#include "mainwindow.h"
#include <QActionGroup>
#include <setxyDialog.h> //引入设置xy坐标的对话框类的头文件
#include "maplabel.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    //this->setProperty("windowOpacity", 0.1);
    //this->setWindowFlags(Qt::WindowContextHelpButtonHint);
    //setAttribute(Qt::WA_TranslucentBackground);//背景透明
    //this->setWindowFlags(Qt::FramelessWindowHint);
    //this->showFullScreen();
    this->resize(1100,600);
    setWindowTitle("无人机避障和路径搜索");
    setWindowIcon(QIcon(":/img/nwpu.png"));
    map=new Astar("Map",this,15,15,20);

    mainarea=new QScrollArea(this);
    mainarea->setAlignment(Qt::AlignCenter);
    mainarea->setWidget(map);

    setStatusBar();
    setMenuBar();
    setToolBar();
    setCentralWidget(mainarea);

    timer=new QTimer(this);
    connect(timer,SIGNAL(timeout()),this,SLOT(setWindowSize()));
    timer->start(300);

    isbgset=false;
    rectl[0]=20;
    rectl[1]=30;
    rectl[2]=45;
    rectl[3]=60;
}

MainWindow::~MainWindow()
{
}
//菜单栏
void MainWindow::setMenuBar(){
    //菜单栏
    QMenu *fileMenu=menuBar()->addMenu("文件");
    QMenu *settingMenu=menuBar()->addMenu("选项");
    QMenu *aboutMenu=menuBar()->addMenu("其他");
    //“传统A*”子菜单栏
    QMenu *normalAstarmenu = new QMenu("传统A*");
    nAstarEudistance = new QAction("欧几里得距离");
    nAstarEudistance->setCheckable(true);
    nAstarMandistance = new QAction("曼哈顿距离");
    nAstarMandistance->setCheckable(true);
    nAstarDiadistance = new QAction("切比雪夫距离");
    nAstarDiadistance->setCheckable(true);
    //“双向A*”子菜单栏
    QMenu *doubleAstarmenu = new QMenu("双向A*");
    dAstarEudistance = new QAction("欧几里得距离(推荐)");
    dAstarEudistance->setCheckable(true);
    dAstarMandistance = new QAction("曼哈顿距离");
    dAstarMandistance->setCheckable(true);
    dAstarDiadistance = new QAction("切比雪夫距离");
    dAstarDiadistance->setCheckable(true);
    //“优化A*”子菜单栏
    QMenu *yydsAstarmenu = new QMenu("优化A*");
    yAstarEudistance = new QAction("欧几里得距离");
    yAstarEudistance->setCheckable(true);
    yAstarMandistance = new QAction("曼哈顿距离");
    yAstarMandistance->setCheckable(true);
    yAstarDiadistance = new QAction("切比雪夫距离");
    yAstarDiadistance->setCheckable(true);
    //扩展因素
    safeAction=new QAction("避开障碍");
    safeAction->setCheckable(true);
    angleAction=new QAction("减少拐角");
    angleAction->setCheckable(true);
    allAction=new QAction("同时考虑");
    allAction->setCheckable(true);
    nothingAction=new QAction("不考虑其他因素");
    nothingAction->setCheckable(true);
    //“LPA*”子菜单栏
    QMenu *LPAstarmenu = new QMenu("LPA*");
    LPAEudistance=new QAction("欧几里得距离");
    LPAEudistance->setCheckable(true);
    LPAMandistance=new QAction("曼哈顿距离");
    LPAMandistance->setCheckable(true);
    LPADiadistance=new QAction("切比雪夫距离");
    LPADiadistance->setCheckable(true);
    //“D*”子菜单栏
    QMenu *Dstarmenu = new QMenu("D*");
    DstarEudistance=new QAction("欧几里得距离");
    DstarEudistance->setCheckable(true);
    DstarMandistance=new QAction("曼哈顿距离");
    DstarMandistance->setCheckable(true);
    DstarDiadistance=new QAction("切比雪夫距离");
    DstarDiadistance->setCheckable(true);
    //“D*lite”子菜单栏
    QMenu *Dlitemenu = new QMenu("D*lite");
    DliteEudistance=new QAction("欧几里得距离");
    DliteEudistance->setCheckable(true);
    DliteMandistance=new QAction("曼哈顿距离");
    DliteMandistance->setCheckable(true);
    DliteDiadistance=new QAction("切比雪夫距离");
    DliteDiadistance->setCheckable(true);

    QAction *setxyAction=new QAction("设置地图宽高");
    QAction *fileopenAction=new QAction("打开地图");
    QAction *filesaveAction=new QAction("保存地图");
    QAction *aboutAction=new QAction("关于我们");
    QAction *backAction=new QAction("打开背景");
    QAction *clearbackAction=new QAction("清除背景");

    //切换方向
    set4dir= new QAction("四方向");
    set4dir->setCheckable(true);
    set8dir= new QAction("八方向");
    set8dir->setCheckable(true);
    //算法选择
    dfsAction=new QAction("深度优先");
    dfsAction->setCheckable(true);
    bfsAction=new QAction("广度优先");
    bfsAction->setCheckable(true);
    dijkstraAction=new QAction("Dijkstra");
    dijkstraAction->setCheckable(true);
    acoAction=new QAction("蚁群算法");
    acoAction->setCheckable(true);
    gbfsAction=new QAction("GBFS");
    gbfsAction->setCheckable(true);
    //贝塞尔曲线
    bezierAction=new QAction("显示贝塞尔曲线");
    bezierAction->setCheckable(true);

    //dir actions
    QActionGroup *directionsAction=new QActionGroup(this);
    directionsAction->addAction(set4dir);
    directionsAction->addAction(set8dir);
    //hfunc actions
    QActionGroup *hfuncAction=new QActionGroup(this);
    hfuncAction->addAction(dfsAction); //“深搜”
    hfuncAction->addAction(bfsAction); //“广度优先”
    hfuncAction->addAction(dijkstraAction); //“Dijkstra”
    hfuncAction->addAction(acoAction); //ACO蚁群算法
    hfuncAction->addAction(gbfsAction); //GBFS最佳优先搜索
    hfuncAction->addAction(nAstarEudistance); //“传统A*”
    hfuncAction->addAction(nAstarMandistance);
    hfuncAction->addAction(nAstarDiadistance);
    hfuncAction->addAction(dAstarEudistance); //“双向A*”
    hfuncAction->addAction(dAstarMandistance);
    hfuncAction->addAction(dAstarDiadistance);
    hfuncAction->addAction(yAstarEudistance); //“优化A*”
    hfuncAction->addAction(yAstarMandistance);
    hfuncAction->addAction(yAstarDiadistance);
    hfuncAction->addAction(DstarEudistance); //D星算法
    hfuncAction->addAction(DstarMandistance);
    hfuncAction->addAction(DstarDiadistance);
    hfuncAction->addAction(LPAEudistance); //LPA*
    hfuncAction->addAction(LPAMandistance);
    hfuncAction->addAction(LPADiadistance);
    hfuncAction->addAction(DliteEudistance); //D*Lite算法
    hfuncAction->addAction(DliteMandistance);
    hfuncAction->addAction(DliteDiadistance);
    //factor actions
    QActionGroup *factorAction=new QActionGroup(this);
    factorAction->addAction(safeAction);
    factorAction->addAction(angleAction);
    factorAction->addAction(allAction);
    factorAction->addAction(nothingAction);

    //“文件”菜单选项
    fileMenu->addAction(fileopenAction); //打开地图文件
    fileMenu->addAction(filesaveAction); //保存地图文件
    fileMenu->addSeparator();  //设置分界线
    fileMenu->addAction(backAction); //打开背景图片
    fileMenu->addAction(clearbackAction); //清除背景图片
    //“帮助”菜单选项
    aboutMenu->addAction(aboutAction);
    //“选项”菜单选项
    settingMenu->addAction(setxyAction); //设置地图宽高
    settingMenu->addAction(bezierAction); //显示贝塞尔曲线
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(set4dir);
    settingMenu->addAction(set8dir);
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(dfsAction); //添加“深度优先”选项
    settingMenu->addAction(bfsAction); //添加“广度优先”选项
    settingMenu->addAction(dijkstraAction); //添加“Dijkstra”选项
    settingMenu->addAction(gbfsAction); //添加“最佳优先搜索”选项
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addMenu(normalAstarmenu); //添加“传统A*”选项
    settingMenu->addMenu(doubleAstarmenu); //添加“双向A*”选项
    settingMenu->addMenu(yydsAstarmenu); //添加“优化A*”选项
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addMenu(Dstarmenu); //添加“D星算法”选项
    settingMenu->addMenu(LPAstarmenu); //添加“LPA*”选项
    settingMenu->addMenu(Dlitemenu); //添加“D*Lite”选项
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(acoAction); //添加“蚁群算法”选项


    //“传统A*”子菜单栏
    normalAstarmenu->addAction(nAstarEudistance);
    normalAstarmenu->addAction(nAstarMandistance);
    normalAstarmenu->addAction(nAstarDiadistance);
    //“双向A*”子菜单栏
    doubleAstarmenu->addAction(dAstarEudistance);
    doubleAstarmenu->addAction(dAstarMandistance);
    doubleAstarmenu->addAction(dAstarDiadistance);
    //“优化A*”子菜单栏
    yydsAstarmenu->addAction(yAstarEudistance);
    yydsAstarmenu->addAction(yAstarMandistance);
    yydsAstarmenu->addAction(yAstarDiadistance);
    yydsAstarmenu->addSeparator();
    yydsAstarmenu->addAction(safeAction);
    yydsAstarmenu->addAction(angleAction);
    yydsAstarmenu->addAction(allAction);
    yydsAstarmenu->addAction(nothingAction);
    //“LPA*”子菜单栏
    LPAstarmenu->addAction(LPAEudistance);
    LPAstarmenu->addAction(LPAMandistance);
    LPAstarmenu->addAction(LPADiadistance);
    //“D*”子菜单栏
    Dstarmenu->addAction(DstarEudistance);
    Dstarmenu->addAction(DstarMandistance);
    Dstarmenu->addAction(DstarDiadistance);
    //“D*lite”子菜单栏
    Dlitemenu->addAction(DliteEudistance);
    Dlitemenu->addAction(DliteMandistance);
    Dlitemenu->addAction(DliteDiadistance);

    //设置各个Action对应的信号槽
    connect(setxyAction,SIGNAL(triggered(bool)),this,SLOT(openxyDialog())); //设置地图长宽
    connect(aboutAction,SIGNAL(triggered(bool)),this,SLOT(openAboutDialog())); //“关于我们”窗口
    connect(set4dir,SIGNAL(triggered(bool)),this,SLOT(setdir())); //四方向
    connect(set8dir,SIGNAL(triggered(bool)),this,SLOT(setdir())); //八方向
    //“传统A*”
    connect(nAstarEudistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(nAstarMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(nAstarDiadistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //“双向A*”
    connect(dAstarEudistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(dAstarMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(dAstarDiadistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //“优化A*”
    connect(yAstarEudistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(yAstarMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(yAstarDiadistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //扩展
    connect(safeAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“安全距离”信号槽
    connect(angleAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“考虑拐角”信号槽
    connect(allAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“考虑拐角”信号槽
    connect(nothingAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“不考虑其他因素”信号槽
    //深度优先
    connect(dfsAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //广度优先
    connect(bfsAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //Dijkstra
    connect(dijkstraAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //蚁群算法
    connect(acoAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //最佳优先搜索
    connect(gbfsAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //D星
    connect(DstarEudistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(DstarMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(DstarMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //LPA*
    connect(LPAEudistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(LPAMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(LPADiadistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    //D*Lite
    connect(DliteEudistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(DliteMandistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(DliteDiadistance,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));

    connect(bezierAction,SIGNAL(triggered(bool)),this,SLOT(setbezier())); //设置“贝塞尔曲线”信号槽
    connect(backAction,SIGNAL(triggered(bool)),this,SLOT(setbackpic())); //打开背景
    connect(clearbackAction,SIGNAL(triggered(bool)),this,SLOT(clearbackpic())); //清除背景
    connect(fileopenAction,SIGNAL(triggered(bool)),this,SLOT(loadmap())); //打开地图文件
    connect(filesaveAction,SIGNAL(triggered(bool)),this,SLOT(savemap())); //保存地图文件

    //设置默认初始模式
    set8dir->trigger();
    yAstarDiadistance->trigger();
    nothingAction->trigger();
}
//设置宽高窗口
void MainWindow::openxyDialog(){
    SetxyDialog *aa=new SetxyDialog(this,map->w,map->h);
    //qDebug()<<map;
    if(aa->exec()==QDialog::Accepted){
        delete map;
        map=new Astar("Map",this,aa->x,aa->y,rectl[setrectsize->currentIndex()]);
        setpainter();
        mainarea->setWidget(map);
        //设为默认模式状态
        setStatusBar(aa->x,aa->y,1,1);
        set8dir->trigger();
        yAstarDiadistance->trigger();
        nothingAction->trigger();
        if(isbgset) map->setbackground(pic);
    }
    delete aa;
    return;
}
//重置地图数据
void MainWindow::clearmap(){
    int tempw=map->w;
    int temph=map->h;
    delete map;
    map=new Astar("Map",this,tempw,temph,rectl[setrectsize->currentIndex()]);
    setpainter();
    mainarea->setWidget(map);
    if(isbgset) map->setbackground(pic);
    setdir();
    sethfunc();
}
//工具栏
void MainWindow::setToolBar(){
    QToolBar *barset=addToolBar("工具栏");
    //barset->setMovable(false);
    barset->setStyleSheet("QToolBar{background-color:white;}");
    QToolButton *clearBtn=new QToolButton(this);
    clearBtn->setIcon(QIcon(":/img/repositioning.png"));
    clearBtn->setText("重置数据变量");
    clearBtn->setFont(QFont("微软雅黑",12));
    clearBtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    clearBtn->setMinimumSize(80,65);

    barset->addWidget(clearBtn);
    connect(clearBtn,SIGNAL(clicked(bool)),this,SLOT(clearmap()));
    barset->addSeparator();
    btngroup=new QButtonGroup(this);

    QToolButton *randNewbtn = new QToolButton(this);
    randNewbtn->setText("随机地图");
    randNewbtn->setFont(QFont("微软雅黑",12));
    randNewbtn->setIcon(QIcon(":/img/question.png"));
    randNewbtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    barset->addWidget(randNewbtn);
    barset->addSeparator();
    connect(randNewbtn,SIGNAL(clicked(bool)),this,SLOT(CreateRandMap()));

    startbtn=new QToolButton(this);
    startbtn->setText("设置起点");
    startbtn->setFont(QFont("微软雅黑",12));
    startbtn->setCheckable(true);
    startbtn->setIcon(QIcon(":/img/local.png"));
    startbtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    endbtn=new QToolButton(this);
    endbtn->setText("设置终点");
    endbtn->setFont(QFont("微软雅黑",12));
    endbtn->setCheckable(true);
    endbtn->setIcon(QIcon(":/img/des.png"));
    endbtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    blocksbtn=new QToolButton(this);
    blocksbtn->setText("标注障碍");
    blocksbtn->setFont(QFont("微软雅黑",12));
    blocksbtn->setCheckable(true);
    blocksbtn->setIcon(QIcon(":/img/caution.png"));
    blocksbtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    //blocksbtn->setStyleSheet("QToolButton:pressed { background-color: blue; border: 2px solid white; }");

    btngroup->addButton(startbtn);
    btngroup->addButton(endbtn);
    btngroup->addButton(blocksbtn);

    barset->addWidget(startbtn);
    barset->addWidget(endbtn);
    barset->addWidget(blocksbtn);
    //barset->setStyleSheet("background-color:black");
    //barset->setStyleSheet("QToolBar { background-color: gray; border: 2px solid black; spacing: 10; }"
    //              "QToolBar::separator { width: 5px; background-color: black; }");
    connect(startbtn,SIGNAL(clicked(bool)),this,SLOT(setpainter()));
    connect(endbtn,SIGNAL(clicked(bool)),this,SLOT(setpainter()));
    connect(blocksbtn,SIGNAL(clicked(bool)),this,SLOT(setpainter()));

    barset->addSeparator();

    startbtn->click();

    QToolButton *startAstar=new QToolButton(this);
    startAstar->setText("润!");
    startAstar->setFont(QFont("微软雅黑",12));
    startAstar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    startAstar->setMinimumSize(75,66);
    startAstar->setIcon(QIcon(":/img/telegram.png"));
    barset->addWidget(startAstar);
    connect(startAstar,SIGNAL(clicked(bool)),this,SLOT(startA()));

    QToolButton *secondSearch=new QToolButton(this);
    secondSearch->setText("再润!");
    secondSearch->setFont(QFont("微软雅黑",12));
    secondSearch->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    secondSearch->setMinimumSize(75,66);
    secondSearch->setIcon(QIcon(":/img/telegram.png"));
    barset->addWidget(secondSearch);
    connect(secondSearch,SIGNAL(clicked(bool)),this,SLOT(secondSearch()));

    QToolButton *nextpath=new QToolButton(this);
    nextpath->setText("NEXTPATH");
    nextpath->setFont(QFont("微软雅黑",12));
    nextpath->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    nextpath->setIcon(QIcon(":/img/next.png"));
    barset->addWidget(nextpath);
    connect(nextpath,SIGNAL(clicked(bool)),this,SLOT(nextpath()));

    QToolButton *showmin=new QToolButton(this);
    showmin->setText("深搜最短");
    showmin->setFont(QFont("微软雅黑",12));
    showmin->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    showmin->setIcon(QIcon(":/img/min.png"));
    barset->addWidget(showmin);
    connect(showmin,SIGNAL(clicked(bool)),this,SLOT(showmin_dfs()));

    QToolButton *analysis=new QToolButton(this);
    analysis->setText("性能分析");
    analysis->setFont(QFont("微软雅黑",12));
    analysis->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    analysis->setIcon(QIcon(":/img/workbench.png"));
    barset->addWidget(analysis);
    connect(analysis,SIGNAL(clicked(bool)),this,SLOT(openAnalysis()));

    QToolButton *clearways=new QToolButton(this);
    clearways->setText("清除当前路径");
    clearways->setFont(QFont("微软雅黑",12));
    clearways->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    clearways->setIcon(QIcon(":/img/clear1.png"));
    barset->addWidget(clearways);
    connect(clearways,SIGNAL(clicked(bool)),this,SLOT(clearways()));

    barset->addSeparator();

    QLabel* rectsizelabel=new QLabel;
    setrectsize=new QComboBox;
    setrectsize->setFont(QFont("微软雅黑",12));
    setrectsize->addItem("小");
    setrectsize->addItem("中");
    setrectsize->addItem("大");
    setrectsize->addItem("特大");
    setrectsize->setCurrentIndex(0); //默认“小”状态
    QLabel* sizetext=new QLabel("单元格大小");
    sizetext->setFont(QFont("微软雅黑",12));
    setrectsize->setToolTip("设置节点大小");
    QVBoxLayout * rectsizelayout=new QVBoxLayout;
    rectsizelayout->setContentsMargins(0, 0, 0, 0);
    rectsizelayout->setAlignment(Qt::AlignCenter);
    //rectsizelayout->addSpacing(6);
    rectsizelayout->addWidget(setrectsize);
    rectsizelayout->addWidget(sizetext);
    rectsizelabel->setLayout(rectsizelayout);
    rectsizelabel->setFixedWidth(100);
    barset->addWidget(rectsizelabel);

    barset->addSeparator();
    connect(setrectsize,SIGNAL(currentIndexChanged(int)),this,SLOT(setRect(int)));

    barset->setIconSize(QSize(35,35));
}
void MainWindow::setpainter(){ //设置起点 终点 障碍
    if(startbtn->isChecked()) map->setpainterstatus(2);
    if(endbtn->isChecked()) map->setpainterstatus(3);
    if(blocksbtn->isChecked()) map->setpainterstatus(0);
}
//int startMap;
//int endMap;
bool israndOK;
void MainWindow::CreateRandMap(){
    clearmap();
    map->createRandmap();
}
//调用A*算法
void MainWindow::startA(){
    map->runAstar();
}
//增量搜索
void MainWindow::secondSearch(){
    map->rescanLPAstar();
}
//调用清除路径函数
void MainWindow::clearways(){
    map->clearways();
}
//下一条路径
void MainWindow::nextpath(){
    map->nextpath();
}
//显示最优路径
void MainWindow::showmin_dfs(){
    map->showmin_dfs();
    setStatusBar(map->w,map->h,2,6);
}
void MainWindow::setWindowSize(){ //设置窗口宽高
    int ww=this->width();
    int hh=this->height();
    //qDebug()<<ww<<hh;
    this->setMinimumSize(ww,hh);
    //qDebug()<<startbtn->size();
    timer->stop();
}
void MainWindow::openTime(){
    QProcess process(this);
    QString str = QApplication::applicationDirPath();
    str += "/time.exe";
    process.startDetached(str);
}
void MainWindow::openSearch(){
    QProcess process(this);
    QString str = QApplication::applicationDirPath();
    str += "/search.exe";
    process.startDetached(str);
}
extern int dfsPathNum;
void MainWindow::openAnalysis(){ //性能分析
    //先打开性能分析开关 用于控制一次弹窗
    map->setAnalykey(true);
    map->sethfunc(1); //热身
    map->runAstar();
    map->clearways();
    //优化A*
    map->sethfunc(2); //曼哈顿
    map->runAstar();
    map->clearways();
    map->sethfunc(3); //欧式
    map->runAstar();
    map->clearways();
    map->sethfunc(1); //切比雪夫
    map->runAstar();
    map->clearways();
    //传统A*
    map->sethfunc(7); //欧式
    map->runAstar();
    map->clearways();
    map->sethfunc(8); //曼哈顿
    map->runAstar();
    map->clearways();
    map->sethfunc(9); //切比雪夫
    map->runAstar();
    map->clearways();
    //双向A*
    map->sethfunc(10); //欧式
    map->runAstar();
    map->clearways();
    map->sethfunc(11); //曼哈顿
    map->runAstar();
    map->clearways();
    map->sethfunc(12); //切比雪夫
    map->runAstar();
    map->clearways();
    if(map->w < 20 and map->h < 20){
        //深搜
        map->sethfunc(4);
        map->runAstar();
        map->clearways();
    }else{
        dfsPathNum = -1;
    }
    //广度优先
    map->sethfunc(5);
    map->runAstar();
    map->clearways();
    //Dijkstra
    map->sethfunc(23);
    map->runAstar();
    map->clearways();
    //GBFS
    map->sethfunc(14);
    map->runAstar();
    map->clearways();
    //D*
    map->sethfunc(13);
//    map->runDstar();
    map->clearways();
    //LPA*
    map->sethfunc(15);
//    map->runLPAstar();
    map->clearways();
    //D*lite
    map->sethfunc(20);
//    map->runDlitestar();
    map->clearways();
    //默认初始模式
    set8dir->trigger();
    yAstarDiadistance->trigger();
    nothingAction->trigger();
    analysis *analy=new analysis("1",this);
    analy->show();
    map->setAnalykey(false); //分析结束将开关关闭
}
void MainWindow::openAboutDialog(){ //“关于我们”窗口
    AboutDialog *about=new AboutDialog(this);
    about->show();
}
void MainWindow::setbezier() //设置bezier开关
{
    if(bezierAction->isChecked()){ //开启贝塞尔
        map->setbezier(1);
    }
    else{ //关闭
        map->setbezier(0);
    }
}
//设置func的值
void MainWindow::sethfunc(){
    if(yAstarEudistance->isChecked()){ //优化A*
        map->sethfunc(3);
        setStatusBar(map->w,map->h,0,3);
    }else if(yAstarMandistance->isChecked()){
        map->sethfunc(2);
        setStatusBar(map->w,map->h,0,2);
    }else if(yAstarDiadistance->isChecked()){
        map->sethfunc(1);
        setStatusBar(map->w,map->h,0,1);
    }
    else if(dfsAction->isChecked()){ //深度优先 暂时只支持四方向
        map->sethfunc(4);
        setStatusBar(map->w,map->h,2,4);
        set4dir->trigger();
    }
    else if(bfsAction->isChecked()){ //广度优先 暂时只支持四方向
        map->sethfunc(5);
        setStatusBar(map->w,map->h,2,5);
        set4dir->trigger();
    }
    else if(acoAction->isChecked()){ //蚁群算法
        map->sethfunc(24);
        setStatusBar(map->w,map->h,0,24);
    }
    else if(dijkstraAction->isChecked()){ //Dijkstra
        map->sethfunc(23);
        setStatusBar(map->w,map->h,0,23);
    }
    else if(nAstarEudistance->isChecked()){ //传统A*
        map->sethfunc(7);
        setStatusBar(map->w,map->h,0,7);
    }
    else if(nAstarMandistance->isChecked()){
        map->sethfunc(8);
        setStatusBar(map->w,map->h,0,8);
    }
    else if(nAstarDiadistance->isChecked()){
        map->sethfunc(9);
        setStatusBar(map->w,map->h,0,9);
    }
    else if(dAstarEudistance->isChecked()){ //双向A*
        map->sethfunc(10);
        setStatusBar(map->w,map->h,0,10);
    }
    else if(dAstarMandistance->isChecked()){
        map->sethfunc(11);
        setStatusBar(map->w,map->h,0,11);
    }
    else if(dAstarDiadistance->isChecked()){
        map->sethfunc(12);
        setStatusBar(map->w,map->h,0,12);
    }
    else if(DstarEudistance->isChecked()){ //D*
        map->sethfunc(13);
        setStatusBar(map->w,map->h,0,13);
    }
    else if(DstarMandistance->isChecked()){
        map->sethfunc(18);
        setStatusBar(map->w,map->h,0,18);
    }
    else if(DstarDiadistance->isChecked()){
        map->sethfunc(19);
        setStatusBar(map->w,map->h,0,19);
    }
    else if(gbfsAction->isChecked()){ //GBFS
        map->sethfunc(14);
        setStatusBar(map->w,map->h,0,14);
    }
    else if(LPAEudistance->isChecked()){ //LPA*
        map->sethfunc(15);
        setStatusBar(map->w,map->h,0,15);
    }
    else if(LPAMandistance->isChecked()){
        map->sethfunc(16);
        setStatusBar(map->w,map->h,0,16);
    }
    else if(LPADiadistance->isChecked()){
        map->sethfunc(17);
        setStatusBar(map->w,map->h,0,17);
    }
    else if(DliteEudistance->isChecked()){ //D*Lite
        map->sethfunc(20);
        setStatusBar(map->w,map->h,0,20);
    }
    else if(DliteMandistance->isChecked()){
        map->sethfunc(21);
        setStatusBar(map->w,map->h,0,21);
    }
    else if(DliteDiadistance->isChecked()){
        map->sethfunc(22);
        setStatusBar(map->w,map->h,0,22);
    }
}
//设置方向参数
void MainWindow::setdir(){
    if(set4dir->isChecked()){
        map->setdir(2);
        setStatusBar(map->w,map->h,2,0);
    }else{
        if(dfsAction->isChecked()){
            QMessageBox::information(this,"Caution","当前版本深度优先只支持四方向哦~",QMessageBox::Ok);
            //map->setdir(2);
            set4dir->trigger();
            return;
        }else if(bfsAction->isChecked()){
            QMessageBox::information(this,"Caution","当前版本广度优先只支持四方向哦~",QMessageBox::Ok);
            //map->setdir(2);
            set4dir->trigger();
            return;
        }
        map->setdir(1);
        setStatusBar(map->w,map->h,1,0);
    }
}
//设置影响因子（拐角、障碍物）
void MainWindow::setfactor(){
    if(angleAction->isChecked()){ //自定义拐角权值
        bool ok;
        int num = QInputDialog::getInt(this, tr("提示"), tr("请输入拐角权值："), 6, 1, 20, 2, &ok);
        if (ok){
            map->setpenalty(num);
        }
        map->setfactor(2);
    }else if(safeAction->isChecked()){ //考虑障碍物问题 选择权值模式
        bool ok;
        int mode = QInputDialog::getInt(this, tr("障碍惩罚值alpha"), tr("请选择alpha计算模式：\n1.静态alpha\n2.考虑相邻节点\n3.动态alpha"), 1, 1, 3, 1, &ok);
        if (ok){
            map->setmode(mode);
        }
        map->setfactor(1);
    }else if(nothingAction->isChecked()){ //不考虑任何因素
        map->setfactor(0);
    }else if(allAction->isChecked()){ //同时考虑
        bool ok=false;
        int num= QInputDialog::getInt(this, tr("提示"), tr("请输入penalty："), 6, 1, 20, 2, &ok);
            if (ok){
                map->setpenalty(num);
            }
        ok=false;
        int mode = QInputDialog::getInt(this, tr("障碍惩罚值alpha"), tr("请选择alpha计算模式：\n1.静态alpha\n2.考虑相邻节点\n3.动态alpha"), 1, 1, 3, 1, &ok);
            if (ok){
                map->setmode(mode);
            }
        map->setfactor(3);
    }
}
//状态栏
void MainWindow::setStatusBar(){
    widthStatus=new QLabel("宽度：10");
    heightStatus=new QLabel("高度：10");
    dirStatus=new QLabel("方向:八方向");
    hfuncStatus=new QLabel("启发函数：切比雪夫距离");

    statusBar()->addPermanentWidget(dirStatus);
    statusBar()->addPermanentWidget(hfuncStatus);
    statusBar()->addWidget(widthStatus);
    statusBar()->addWidget(heightStatus);

    statusBar()->setFont(QFont("SimSun",11));
}
void MainWindow::setStatusBar(int x,int y,int dir,int hfunc){
    widthStatus->setText(QString("宽度：")+QString::number(x));
    heightStatus->setText(QString("高度：")+QString::number(y));
    switch(dir){
    case 1:
        dirStatus->setText(QString("方向：八方向"));
        break;
    case 2:
        dirStatus->setText(QString("方向：四方向"));
        break;
    }
    switch(hfunc){
    case 1:
        hfuncStatus->setText(QString("|算法：优化A* 切比雪夫距离"));
        break;
    case 2:
        hfuncStatus->setText(QString("|算法：优化A* 曼哈顿距离"));
        break;
    case 3:
        hfuncStatus->setText(QString("|算法：优化A* 欧式距离"));
        break;
    case 4:
        hfuncStatus->setText(QString("|算法：深度优先"));
        break;
    case 5:
        hfuncStatus->setText(QString("|算法：广度优先"));
        break;
    case 6:
        hfuncStatus->setText(QString("|深度优先最优路径"));
        break;
    case 7:
        hfuncStatus->setText(QString("|算法：传统A* 欧式距离"));
        break;
    case 8:
        hfuncStatus->setText(QString("|算法：传统A* 曼哈顿距离"));
        break;
    case 9:
        hfuncStatus->setText(QString("|算法：传统A* 切比雪夫距离"));
        break;
    case 10:
        hfuncStatus->setText(QString("|算法：双向A* 欧式距离"));
        break;
    case 11:
        hfuncStatus->setText(QString("|算法：双向A* 曼哈顿距离"));
        break;
    case 12:
        hfuncStatus->setText(QString("|算法：双向A* 切比雪夫距离"));
        break;
    case 13:
        hfuncStatus->setText(QString("|算法：D* 欧式距离"));
        break;
    case 14:
        hfuncStatus->setText(QString("|算法：GBFS"));
        break;
    case 15:
        hfuncStatus->setText(QString("|算法：LPA* 欧式距离"));
        break;
    case 16:
        hfuncStatus->setText(QString("|算法：LPA* 曼哈顿距离"));
        break;
    case 17:
        hfuncStatus->setText(QString("|算法：LPA* 切比雪夫距离"));
        break;
    case 18:
        hfuncStatus->setText(QString("|算法：D* 曼哈顿距离"));
        break;
    case 19:
        hfuncStatus->setText(QString("|算法：D* 切比雪夫距离"));
        break;
    case 20:
        hfuncStatus->setText(QString("|算法：D*lite 欧式距离"));
        break;
    case 21:
        hfuncStatus->setText(QString("|算法：D*lite 曼哈顿距离"));
        break;
    case 22:
        hfuncStatus->setText(QString("|算法：D*lite 切比雪夫距离"));
        break;
    case 23:
        hfuncStatus->setText(QString("|算法：Dijkstra"));
        break;
    case 24:
        hfuncStatus->setText(QString("|算法：ACO"));
        break;
    }
}
void MainWindow::setbackpic(){  //设置背景图片
    QFileDialog fd;
    QString imagename=fd.getOpenFileName(this,"Open Image","","图片(*.jpg *.bmp *.png)");

    if(imagename.isNull()) return;
    pic.load(imagename);
    if(!pic.isNull()){
        map->setbackground(pic);
        isbgset=true;
        map->setIsbgset(true);
    }else{
        QMessageBox::information(this,"qwq","不存在的图片！");
    }
}
void MainWindow::clearbackpic(){    //清除背景
    if(!isbgset) return;
    isbgset=false;
    map->clearbackground();
    map->setIsbgset(false);
}
void MainWindow::setRect(int t){    //设置单元格大小
    map->setRect(rectl[t]);
}
void MainWindow::savemap(){ //保存地图信息
    QFileDialog fd;
    QString mapname=fd.getSaveFileName(this,"保存地图","","地图(*.AMap)");
    if(mapname.isNull()) return;
    //qDebug()<<mapname;
    map->savemap(mapname);
}
void MainWindow::loadmap(){ //载入地图信息
    QFileDialog fd;
    QString mapname=fd.getOpenFileName(this,"打开地图","","地图(*.AMap)");
    if(mapname.isNull()) return;
    map->loadmap(mapname);
    setrectsize->setCurrentIndex(map->rsizeindex());
    //载入地图 更新状态栏
    setStatusBar(map->w,map->h,1,1);
    //设置默认模式
    set8dir->trigger();
    yAstarDiadistance->trigger();
    nothingAction->trigger();
}
