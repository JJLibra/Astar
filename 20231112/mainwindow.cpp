#include "mainwindow.h"
#include <QActionGroup>
#include <setxyDialog.h> //引入设置xy坐标的对话框类的头文件

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    //this->setProperty("windowOpacity", 0.9);
    setWindowTitle("无人机避障和路径搜索");
    setWindowIcon(QIcon(":/img/nwpu.png"));
    map=new Astar("Map",this,10,10);

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
    QMenu *fileMenu=menuBar()->addMenu("文件");
    QMenu *settingMenu=menuBar()->addMenu("选项");
    QMenu *aboutMenu=menuBar()->addMenu("帮助");
    QAction *setxyAction=new QAction("设置地图宽高");
    QAction *fileopenAction=new QAction("打开(Open)");
    QAction *filesaveAction=new QAction("保存(Save)");
    QAction *aboutAction=new QAction("关于我们");
    QAction *backAction=new QAction("打开背景");
    QAction *clearbackAction=new QAction("清除背景");

    //切换方向
    set4dir= new QAction("四方向");
    set4dir->setCheckable(true);
    set8dir= new QAction("八方向");
    set8dir->setCheckable(true);
    QActionGroup *directionsAction=new QActionGroup(this);
    directionsAction->addAction(set4dir);
    directionsAction->addAction(set8dir);

    //切换启发函数
    euAction=new QAction("欧几里得距离");
    euAction->setCheckable(true);
    manAction=new QAction("曼哈顿距离");
    manAction->setCheckable(true);
    diaAction=new QAction("切比雪夫距离");
    diaAction->setCheckable(true);
    dfsAction=new QAction("深度优先");
    dfsAction->setCheckable(true);
    mindfsAction=new QAction("显示最优");
    mindfsAction->setCheckable(true);
    bfsAction=new QAction("广度优先");
    bfsAction->setCheckable(true);
    normalAction=new QAction("传统A*");
    normalAction->setCheckable(true);
    QActionGroup *hfuncAction=new QActionGroup(this);
    hfuncAction->addAction(euAction);
    hfuncAction->addAction(manAction);
    hfuncAction->addAction(diaAction);
    hfuncAction->addAction(dfsAction);
    hfuncAction->addAction(mindfsAction);
    hfuncAction->addAction(bfsAction);
    hfuncAction->addAction(normalAction);

    safeAction=new QAction("避开障碍");
    safeAction->setCheckable(true);
    angleAction=new QAction("减少拐角");
    angleAction->setCheckable(true);
    allAction=new QAction("同时考虑");
    allAction->setCheckable(true);
    nothingAction=new QAction("不考虑其他因素");
    nothingAction->setCheckable(true);
    QActionGroup *factorAction=new QActionGroup(this);
    factorAction->addAction(safeAction);
    factorAction->addAction(angleAction);
    factorAction->addAction(allAction);
    factorAction->addAction(nothingAction);

    //fileMenu->addSeparator();
    fileMenu->addAction(fileopenAction);
    fileMenu->addAction(filesaveAction);

    //“帮助”菜单栏
    aboutMenu->addAction(aboutAction);

    //设置行列数
    settingMenu->addAction(setxyAction);
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(backAction);
    settingMenu->addAction(clearbackAction);
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(set4dir);
    settingMenu->addAction(set8dir);
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(euAction);
    settingMenu->addAction(manAction);
    settingMenu->addAction(diaAction);
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(dfsAction); //添加“深度优先”选项
    settingMenu->addAction(mindfsAction); //添加“显示最优”选项
    settingMenu->addAction(bfsAction); //添加“广度优先”选项
    settingMenu->addAction(normalAction);
    settingMenu->addSeparator();  //设置分界线
    settingMenu->addAction(safeAction); //添加“安全距离”选项
    settingMenu->addAction(angleAction); //添加“考虑拐角”选项
    settingMenu->addAction(allAction); //添加“同时考虑”选项
    settingMenu->addAction(nothingAction); //添加“不考虑其他因素”选项

    //设置各个选项对应的信号槽
    connect(setxyAction,SIGNAL(triggered(bool)),this,SLOT(openxyDialog()));
    connect(aboutAction,SIGNAL(triggered(bool)),this,SLOT(openAboutDialog()));//打开“关于我们”窗口

    connect(set4dir,SIGNAL(triggered(bool)),this,SLOT(setdir()));
    connect(set8dir,SIGNAL(triggered(bool)),this,SLOT(setdir()));
    connect(euAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(manAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(diaAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));
    connect(dfsAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));   //设置dfs函数信号槽
    connect(mindfsAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));   //设置dfs函数信号槽
    connect(bfsAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));   //设置bfs函数信号槽
    connect(normalAction,SIGNAL(triggered(bool)),this,SLOT(sethfunc()));   //设置传统A*信号槽
    connect(safeAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“安全距离”信号槽
    connect(angleAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“考虑拐角”信号槽
    connect(allAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“考虑拐角”信号槽
    connect(nothingAction,SIGNAL(triggered(bool)),this,SLOT(setfactor()));   //设置“不考虑其他因素”信号槽
    connect(backAction,SIGNAL(triggered(bool)),this,SLOT(setbackpic()));
    connect(clearbackAction,SIGNAL(triggered(bool)),this,SLOT(clearbackpic()));
    connect(fileopenAction,SIGNAL(triggered(bool)),this,SLOT(loadmap()));
    connect(filesaveAction,SIGNAL(triggered(bool)),this,SLOT(savemap()));

    //设置默认初始模式
    set8dir->trigger();
    diaAction->trigger();
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
        setStatusBar(aa->x,aa->y,0,0);
        if(isbgset) map->setbackground(pic);
    }
    //connect(aa, SIGNAL(sendPP(int)), this, SLOT(receivePP(int))); //连接SetxyDialog对象aa发出的sendPP信号和MainWindow对象this的receivePP槽函数
    delete aa;
    return;
}
//清除地图
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
    QToolBar *barset=addToolBar("工具");
    //barset->setMovable(false);
    barset->setStyleSheet("QToolBar{background-color:white;spacing:10;}");
    barset->addSeparator();

    QToolButton *clearBtn=new QToolButton(this);
    clearBtn->setIcon(QIcon(":/img/repositioning.png"));
    clearBtn->setText("重置数据变量");
    clearBtn->setFont(QFont("微软雅黑",12));
    clearBtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    clearBtn->setMinimumSize(81,66);

    barset->addWidget(clearBtn);
    connect(clearBtn,SIGNAL(clicked(bool)),this,SLOT(clearmap()));

    barset->addSeparator();

    btngroup=new QButtonGroup(this);

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
    endbtn->setIcon(QIcon(":/img/trace.png"));
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

    startAstar->setText("润！");
    startAstar->setFont(QFont("微软雅黑",12));
    startAstar->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    startAstar->setMinimumSize(81,66);

    startAstar->setIcon(QIcon(":/img/Telegram.png"));
    barset->addWidget(startAstar);
    connect(startAstar,SIGNAL(clicked(bool)),this,SLOT(startA()));

    QToolButton *nextpath=new QToolButton(this);

    nextpath->setText("NEXTPATH");
    nextpath->setFont(QFont("微软雅黑",12));
    nextpath->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    nextpath->setIcon(QIcon(":/img/go-on.png"));

    barset->addWidget(nextpath);
    connect(nextpath,SIGNAL(clicked(bool)),this,SLOT(nextpath()));

    QToolButton *clearways=new QToolButton(this);

    clearways->setText("清除当前路径");
    clearways->setFont(QFont("微软雅黑",12));
    clearways->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    clearways->setIcon(QIcon(":/img/clear.png"));

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
    setrectsize->setCurrentIndex(1);
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

void MainWindow::setpainter(){
    if(startbtn->isChecked()) map->setpainterstatus(2);
    if(endbtn->isChecked()) map->setpainterstatus(3);
    if(blocksbtn->isChecked()) map->setpainterstatus(0);
}
//调用A*算法
void MainWindow::startA(){
    map->runAstar();
}
//调用清除路径函数
void MainWindow::clearways(){
    map->clearways();
}
//下一条路径
void MainWindow::nextpath(){
    map->nextpath();
}
//设置窗口宽高
void MainWindow::setWindowSize(){
    int ww=this->width();
    int hh=this->height();
    //qDebug()<<ww<<hh;
    this->setMinimumSize(ww,hh);
    //qDebug()<<startbtn->size();
    timer->stop();
}
//打开“关于我们”窗口
void MainWindow::openAboutDialog(){
    AboutDialog *about=new AboutDialog(this);
    about->show();
}
//设置func的值
void MainWindow::sethfunc(){
    if(euAction->isChecked()){
        map->setnormal(0);
        map->sethfunc(3);
        setStatusBar(map->w,map->h,0,3);
    }else if(manAction->isChecked()){
        map->setnormal(0);
        map->sethfunc(2);
        setStatusBar(map->w,map->h,0,2);
    }else if(diaAction->isChecked()){
        map->setnormal(0);
        map->sethfunc(1);
        setStatusBar(map->w,map->h,0,1);
    }
    else if(dfsAction->isChecked()){
        map->sethfunc(4);
        setStatusBar(map->w,map->h,0,4);
    }
    else if(bfsAction->isChecked()){
        map->sethfunc(5);
        setStatusBar(map->w,map->h,0,5);
    }
    else if(normalAction->isChecked()){
        map->setnormal(1);
        map->sethfunc(7);
    }
    else if(mindfsAction->isChecked()){
        map->showmin_dfs();
        map->sethfunc(6);
    }
}
//设置方向参数
void MainWindow::setdir(){
    if(set4dir->isChecked()){
        map->setdir(2);
        setStatusBar(map->w,map->h,2,0);
    }else{
        map->setdir(1);
        setStatusBar(map->w,map->h,1,0);
    }
}
//设置影响因子（拐角、障碍物）
void MainWindow::setfactor(){
    if(angleAction->isChecked()){
        bool ok;
        int num = QInputDialog::getInt(this, tr("提示"), tr("请输入拐角权值："), 6, 1, 20, 2, &ok);
        if (ok){
            map->setpenalty(num);
        }
        //考虑拐角问题
        map->setfactor(2);
    }else if(safeAction->isChecked()){
        //考虑障碍物问题
        bool ok;
        int mode = QInputDialog::getInt(this, tr("障碍惩罚值alpha"), tr("请选择alpha计算模式：\n1.静态alpha\n2.考虑相邻节点\n3.动态alpha"), 1, 1, 3, 1, &ok);
        if (ok){
            map->setmode(mode);
        }
        map->setfactor(1);
    }else if(nothingAction->isChecked()){
        map->setfactor(0);
    }else if(allAction->isChecked()){
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
    dirStatus=new QLabel("方向：八方向");
    hfuncStatus=new QLabel("启发函数：切比雪夫距离");
    //issafeStatus=new QLabel("暂不考虑安全距离");

    statusBar()->addPermanentWidget(dirStatus);
    statusBar()->addPermanentWidget(hfuncStatus);
    //statusBar()->addPermanentWidget(issafeStatus);
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
        hfuncStatus->setText(QString("启发函数：切比雪夫距离|算法：A*"));
        break;
    case 2:
        hfuncStatus->setText(QString("启发函数：曼哈顿距离|算法：A*"));
        break;
    case 3:
        hfuncStatus->setText(QString("启发函数：欧几里得距离|算法：A*"));
        break;
    case 4:
        //qDebug()<<"dfs";
        hfuncStatus->setText(QString("|算法：深度优先"));
        break;
    case 5:
        hfuncStatus->setText(QString("|算法：广度优先"));
        break;
    case 6:
        hfuncStatus->setText(QString("|深度优先最优路径"));
        break;
    case 7:
        hfuncStatus->setText(QString("|算法：传统A*"));
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
    }else{
        QMessageBox::information(this,"qwq","不存在的图片！");
    }
}
void MainWindow::clearbackpic(){    //清除背景
    if(!isbgset) return;
    isbgset=false;
    map->clearbackground();
}
void MainWindow::setRect(int t){    //设置单元格大小
    //qDebug()<<t;
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
}
