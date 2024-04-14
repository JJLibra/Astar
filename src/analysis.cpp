//性能分析窗口
#include <analysis.h>
#include<QColor>
#include<QPainter>
#include<mainwindow.h>

analysis::analysis(const QString &text, QWidget *parent)
{
    this->setWindowTitle("性能分析");
    this->setWindowIcon(QIcon(":/img/workbench.png"));
    this->setWindowFlags(Qt::WindowCloseButtonHint);
    this->setFixedSize(800,600);

    //时间信息
    extern double BfsTime;
    extern double DijkstraTime;
    extern double GbfsTime;
    extern double normalAstarEuTime;
    extern double normalAstarManTime;
    extern double normalAstarDiaTime;
    extern double doubleAstarEuTime;
    extern double doubleAstarManTime;
    extern double doubleAstarDiaTime;
    extern double yydsAstarEuTime;
    extern double yydsAstarManTime;
    extern double yydsAstarDiaTime;
    extern double DAstarTime;
    extern double LPAstarTime;
    extern double DliteTime;
    //搜索范围
    extern int BfsExtend;
    extern int DijkstraExtend;
    extern int GbfsExtend;
    extern int normalAstarEuExtend;
    extern int normalAstarManExtend;
    extern int normalAstarDiaExtend;
    extern int doubleAstarEuExtend;
    extern int doubleAstarManExtend;
    extern int doubleAstarDiaExtend;
    extern int yydsAstarEuExtend;
    extern int yydsAstarManExtend;
    extern int yydsAstarDiaExtend;
    extern int DAstarExtend;
    extern int LPAstarExtend;
    extern int DliteExtend;
    //深搜路径条数
    extern int dfsPathNum;

    QToolBar *Analybarset = new QToolBar(this);
    Analybarset->setStyleSheet("QToolBar{background-color:white;border-radius:5px;}");
    Analybarset->resize(200,60);
    Analybarset->setIconSize(QSize(35,35));
    Analybarset->setOrientation(Qt::Vertical); // 设置工具栏为垂直方向添加

    //时间分析
    QToolButton *TimeBtn =new QToolButton(this);
    TimeBtn->setText("用时");
    TimeBtn->setIcon(QIcon(":/img/TimeAnaly.png"));
    TimeBtn->setFont(QFont("微软雅黑",12));
    TimeBtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
    TimeBtn->setMinimumSize(84,60);
    //搜索范围
    QToolButton *SearchBtn =new QToolButton(this);
    SearchBtn->setText("搜索范围");
    SearchBtn->setIcon(QIcon(":/img/search.png"));
    SearchBtn->setFont(QFont("微软雅黑",12));
    SearchBtn->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

    Analybarset->addWidget(TimeBtn);
    Analybarset->addSeparator();
    Analybarset->addWidget(SearchBtn);
    Analybarset->addSeparator();

    connect(TimeBtn,SIGNAL(clicked(bool)),parent,SLOT(openTime())); //各算法用时分析
    connect(SearchBtn,SIGNAL(clicked(bool)),parent,SLOT(openSearch())); //各算法搜索范围分析

    //深度优先
    QLabel *dfs=new QLabel("深度优先");
    dfs->setAlignment(Qt::AlignCenter);
    dfs->setFont(QFont("黑体",14));
    QLabel *dfspathnum=new QLabel("");
    if(dfsPathNum!=-1) dfspathnum->setText("存在 " + QString::number(dfsPathNum) + " 条可行路径");
    else dfspathnum->setText("当前地图不适合深度优先搜索");
    dfspathnum->setAlignment(Qt::AlignCenter);
    dfspathnum->setFont(QFont("楷体",12));

    //广度优先
    QLabel *bfs=new QLabel("广度优先");
    bfs->setAlignment(Qt::AlignCenter);
    bfs->setFont(QFont("黑体",14));
    QLabel *bfstime=new QLabel("用时：");
    bfstime->setText("用时：" + QString::number(BfsTime) + "ms");
    bfstime->setAlignment(Qt::AlignCenter);
    bfstime->setFont(QFont("楷体",12));
    QLabel *bfsextend=new QLabel("搜索范围：");
    bfsextend->setText("搜索范围：" + QString::number(BfsExtend) + "个扩展点");
    bfsextend->setAlignment(Qt::AlignCenter);
    bfsextend->setFont(QFont("楷体",12));

    //Dijkstra
    QLabel *dijkstra=new QLabel("Dijkstra");
    dijkstra->setAlignment(Qt::AlignCenter);
    dijkstra->setFont(QFont("黑体",14));
    QLabel *dijkstratime=new QLabel("用时：");
    dijkstratime->setText("用时：" + QString::number(DijkstraTime) + "ms");
    dijkstratime->setAlignment(Qt::AlignCenter);
    dijkstratime->setFont(QFont("楷体",12));
    QLabel *dijkstraextend=new QLabel("搜索范围：");
    dijkstraextend->setText("搜索范围：" + QString::number(DijkstraExtend) + "个扩展点");
    dijkstraextend->setAlignment(Qt::AlignCenter);
    dijkstraextend->setFont(QFont("楷体",12));

    //最佳优先搜索
    QLabel *gbfs=new QLabel("GBFS");
    gbfs->setAlignment(Qt::AlignCenter);
    gbfs->setFont(QFont("黑体",14));
    QLabel *gbfstime=new QLabel("用时：");
    gbfstime->setText("用时：" + QString::number(GbfsTime) + "ms");
    gbfstime->setAlignment(Qt::AlignCenter);
    gbfstime->setFont(QFont("楷体",12));
    QLabel *gbfsextend=new QLabel("搜索范围：");
    gbfsextend->setText("搜索范围：" + QString::number(GbfsExtend) + "个扩展点");
    gbfsextend->setAlignment(Qt::AlignCenter);
    gbfsextend->setFont(QFont("楷体",12));

    //传统A*
    QLabel *normalAstar=new QLabel("传统 A*");
    normalAstar->setFont(QFont("黑体",14));
    normalAstar->setAlignment(Qt::AlignCenter);
    QLabel *normalAstarEutime=new QLabel("欧式距离用时：");
    normalAstarEutime->setText("欧式距离用时：" + QString::number(normalAstarEuTime) +"ms");
    normalAstarEutime->setFont(QFont("楷体",12));
    normalAstarEutime->setAlignment(Qt::AlignCenter);
    QLabel *normalAstarEuextend=new QLabel("搜索范围：");
    normalAstarEuextend->setText("搜索范围：" + QString::number(normalAstarEuExtend) + "个扩展点");
    normalAstarEuextend->setFont(QFont("楷体",12));
    normalAstarEuextend->setAlignment(Qt::AlignCenter);
    QLabel *normalAstarMantime=new QLabel("曼哈顿距离用时：");
    normalAstarMantime->setText("曼哈顿距离用时：" + QString::number(normalAstarManTime) +"ms");
    normalAstarMantime->setFont(QFont("楷体",12));
    normalAstarMantime->setAlignment(Qt::AlignCenter);
    QLabel *normalAstarManextend=new QLabel("搜索范围：");
    normalAstarManextend->setText("搜索范围：" + QString::number(normalAstarManExtend) + "个扩展点");
    normalAstarManextend->setFont(QFont("楷体",12));
    normalAstarManextend->setAlignment(Qt::AlignCenter);
    QLabel *normalAstarDiatime=new QLabel("切比雪夫距离用时：");
    normalAstarDiatime->setText("切比雪夫距离用时：" + QString::number(normalAstarDiaTime) +"ms");
    normalAstarDiatime->setFont(QFont("楷体",12));
    normalAstarDiatime->setAlignment(Qt::AlignCenter);
    QLabel *normalAstarDiaextend=new QLabel("搜索范围：");
    normalAstarDiaextend->setText("搜索范围：" + QString::number(normalAstarDiaExtend) + "个扩展点");
    normalAstarDiaextend->setFont(QFont("楷体",12));
    normalAstarDiaextend->setAlignment(Qt::AlignCenter);

    //双向A*
    QLabel *doubleAstar=new QLabel("双向 A*");
    doubleAstar->setFont(QFont("黑体",14));
    doubleAstar->setAlignment(Qt::AlignCenter);
    QLabel *doubleAstarEutime=new QLabel("用时：");
    doubleAstarEutime->setText("欧式距离用时：" + QString::number(doubleAstarEuTime)+"ms");
    doubleAstarEutime->setFont(QFont("楷体",12));
    doubleAstarEutime->setAlignment(Qt::AlignCenter);
    QLabel *doubleAstarEuextend=new QLabel("搜索范围：");
    doubleAstarEuextend->setText("搜索范围：" + QString::number(doubleAstarEuExtend) + "个扩展点");
    doubleAstarEuextend->setFont(QFont("楷体",12));
    doubleAstarEuextend->setAlignment(Qt::AlignCenter);
    QLabel *doubleAstarMantime=new QLabel("用时：");
    doubleAstarMantime->setText("曼哈顿距离用时：" + QString::number(doubleAstarManTime)+"ms");
    doubleAstarMantime->setFont(QFont("楷体",12));
    doubleAstarMantime->setAlignment(Qt::AlignCenter);
    QLabel *doubleAstarManextend=new QLabel("搜索范围：");
    doubleAstarManextend->setText("搜索范围：" + QString::number(doubleAstarManExtend) + "个扩展点");
    doubleAstarManextend->setFont(QFont("楷体",12));
    doubleAstarManextend->setAlignment(Qt::AlignCenter);
    QLabel *doubleAstarDiatime=new QLabel("用时：");
    doubleAstarDiatime->setText("切比雪夫距离用时：" + QString::number(doubleAstarDiaTime)+"ms");
    doubleAstarDiatime->setFont(QFont("楷体",12));
    doubleAstarDiatime->setAlignment(Qt::AlignCenter);
    QLabel *doubleAstarDiaextend=new QLabel("搜索范围：");
    doubleAstarDiaextend->setText("搜索范围：" + QString::number(doubleAstarDiaExtend) + "个扩展点");
    doubleAstarDiaextend->setFont(QFont("楷体",12));
    doubleAstarDiaextend->setAlignment(Qt::AlignCenter);

    //优化改进A*
    QLabel *yydsAstar=new QLabel("优化 A*");
    yydsAstar->setFont(QFont("黑体",14));
    yydsAstar->setAlignment(Qt::AlignCenter);
    QLabel *yydsAstarEutime=new QLabel("用时：");
    yydsAstarEutime->setText("欧式距离用时：" + QString::number(yydsAstarEuTime)+"ms");
    yydsAstarEutime->setFont(QFont("楷体",12));
    yydsAstarEutime->setAlignment(Qt::AlignCenter);
    QLabel *yydsAstarEuextend=new QLabel("搜索范围：");
    yydsAstarEuextend->setText("搜索范围：" + QString::number(yydsAstarEuExtend) + "个扩展点");
    yydsAstarEuextend->setFont(QFont("楷体",12));
    yydsAstarEuextend->setAlignment(Qt::AlignCenter);
    QLabel *yydsAstarMantime=new QLabel("用时：");
    yydsAstarMantime->setText("曼哈顿距离用时：" + QString::number(yydsAstarManTime)+"ms");
    yydsAstarMantime->setFont(QFont("楷体",12));
    yydsAstarMantime->setAlignment(Qt::AlignCenter);
    QLabel *yydsAstarManextend=new QLabel("搜索范围：");
    yydsAstarManextend->setText("搜索范围：" + QString::number(yydsAstarManExtend) + "个扩展点");
    yydsAstarManextend->setFont(QFont("楷体",12));
    yydsAstarManextend->setAlignment(Qt::AlignCenter);
    QLabel *yydsAstarDiatime=new QLabel("用时：");
    yydsAstarDiatime->setText("切比雪夫距离用时：" + QString::number(yydsAstarDiaTime)+"ms");
    yydsAstarDiatime->setFont(QFont("楷体",12));
    yydsAstarDiatime->setAlignment(Qt::AlignCenter);
    QLabel *yydsAstarDiaextend=new QLabel("搜索范围：");
    yydsAstarDiaextend->setText("搜索范围：" + QString::number(yydsAstarDiaExtend) + "个扩展点");
    yydsAstarDiaextend->setFont(QFont("楷体",12));
    yydsAstarDiaextend->setAlignment(Qt::AlignCenter);

    //D*
    QLabel *Dstar=new QLabel("D*");
    Dstar->setFont(QFont("黑体",14));
    Dstar->setAlignment(Qt::AlignCenter);
    QLabel *Dstartime=new QLabel("第一次搜索用时：");
    Dstartime->setText("第一次搜索用时：" + QString::number(DAstarTime) + "ms");
    Dstartime->setFont(QFont("楷体",12));
    Dstartime->setAlignment(Qt::AlignCenter);
    QLabel *Dstarextend=new QLabel("第一次搜索范围：");
    Dstarextend->setText("第一次搜索范围：" + QString::number(DAstarExtend) + "个扩展点");
    Dstarextend->setFont(QFont("楷体",12));
    Dstarextend->setAlignment(Qt::AlignCenter);

    //LPA*
    QLabel *LPAstar=new QLabel("LPA*");
    LPAstar->setFont(QFont("黑体",14));
    LPAstar->setAlignment(Qt::AlignCenter);
    QLabel *LPAstartime=new QLabel("第一次搜索用时：");
    LPAstartime->setText("第一次搜索用时：" + QString::number(LPAstarTime) + "ms");
    LPAstartime->setFont(QFont("楷体",12));
    LPAstartime->setAlignment(Qt::AlignCenter);
    QLabel *LPAstarextend=new QLabel("第一次搜索范围：");
    LPAstarextend->setText("第一次搜索范围：" + QString::number(LPAstarExtend) + "个扩展点");
    LPAstarextend->setFont(QFont("楷体",12));
    LPAstarextend->setAlignment(Qt::AlignCenter);

    //D*lite
    QLabel *Dlitestar=new QLabel("D*lite");
    Dlitestar->setFont(QFont("黑体",14));
    Dlitestar->setAlignment(Qt::AlignCenter);
    QLabel *Dlitestartime=new QLabel("第一次搜索用时：");
    Dlitestartime->setText("第一次搜索用时：" + QString::number(DliteTime) + "ms");
    Dlitestartime->setFont(QFont("楷体",12));
    Dlitestartime->setAlignment(Qt::AlignCenter);
    QLabel *Dlitestarextend=new QLabel("第一次搜索范围：");
    Dlitestarextend->setText("第一次搜索范围：" + QString::number(DliteExtend) + "个扩展点");
    Dlitestarextend->setFont(QFont("楷体",12));
    Dlitestarextend->setAlignment(Qt::AlignCenter);

    //信息布局
    QVBoxLayout *layoutText = new QVBoxLayout;
    layoutText->setAlignment(Qt::AlignCenter);
    layoutText->setAlignment(Qt::AlignBottom);
    layoutText->addStretch();
    layoutText->addWidget(dfs);
    layoutText->addWidget(dfspathnum);
    layoutText->addStretch();
    layoutText->addWidget(bfs);
    layoutText->addWidget(bfstime);
    layoutText->addWidget(bfsextend);
    layoutText->addStretch();
    layoutText->addWidget(dijkstra);
    layoutText->addWidget(dijkstratime);
    layoutText->addWidget(dijkstraextend);
    layoutText->addStretch();
    layoutText->addWidget(gbfs);
    layoutText->addWidget(gbfstime);
    layoutText->addWidget(gbfsextend);
    layoutText->addStretch();

    QVBoxLayout *layoutAstar = new QVBoxLayout;
    layoutAstar->setAlignment(Qt::AlignCenter);
    layoutAstar->setAlignment(Qt::AlignBottom);
    layoutAstar->addStretch();
    layoutAstar->addWidget(normalAstar);
    layoutAstar->addWidget(normalAstarEutime);
    layoutAstar->addWidget(normalAstarEuextend);
    layoutAstar->addWidget(normalAstarMantime);
    layoutAstar->addWidget(normalAstarManextend);
    layoutAstar->addWidget(normalAstarDiatime);
    layoutAstar->addWidget(normalAstarDiaextend);
    layoutAstar->addStretch();
    layoutAstar->addWidget(doubleAstar);
    layoutAstar->addWidget(doubleAstarEutime);
    layoutAstar->addWidget(doubleAstarEuextend);
    layoutAstar->addWidget(doubleAstarMantime);
    layoutAstar->addWidget(doubleAstarManextend);
    layoutAstar->addWidget(doubleAstarDiatime);
    layoutAstar->addWidget(doubleAstarDiaextend);
    layoutAstar->addStretch();
    layoutAstar->addWidget(yydsAstar);
    layoutAstar->addWidget(yydsAstarEutime);
    layoutAstar->addWidget(yydsAstarEuextend);
    layoutAstar->addWidget(yydsAstarMantime);
    layoutAstar->addWidget(yydsAstarManextend);
    layoutAstar->addWidget(yydsAstarDiatime);
    layoutAstar->addWidget(yydsAstarDiaextend);
    layoutAstar->addStretch();

    QVBoxLayout *layoutExAstar = new QVBoxLayout;
    layoutExAstar->addStretch();
    layoutExAstar->addWidget(Dstar);
    layoutExAstar->addWidget(Dstartime);
    layoutExAstar->addWidget(Dstarextend);
    layoutExAstar->addStretch();
    layoutExAstar->addWidget(LPAstar);
    layoutExAstar->addWidget(LPAstartime);
    layoutExAstar->addWidget(LPAstarextend);
    layoutExAstar->addStretch();
    layoutExAstar->addWidget(Dlitestar);
    layoutExAstar->addWidget(Dlitestartime);
    layoutExAstar->addWidget(Dlitestarextend);
    layoutExAstar->addStretch();
    layoutExAstar->setAlignment(Qt::AlignTop);

    QHBoxLayout *analysislayout = new QHBoxLayout(this);
    //analysislayout->addStretch();
    analysislayout->addWidget(Analybarset);
    analysislayout->addStretch();
    analysislayout->addLayout(layoutText);
    //analysislayout->addSpacing(6);
    analysislayout->addStretch();
    analysislayout->addLayout(layoutAstar);
    analysislayout->addStretch();
    analysislayout->addLayout(layoutExAstar);
    analysislayout->addStretch();
    analysislayout->setAlignment(Qt::AlignBottom);
}

