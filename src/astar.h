#ifndef ASTAR_H
#define ASTAR_H

#include <mainwindow.h>
#include <maplabel.h>
#include <QList>
#include <QRadioButton>
#include <QMessageBox>
#include <QPropertyAnimation>
#include <cmath>
#include <analysis.h>
#include <queue>
#include <vector>
#include <random>

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

//深搜
extern int dfsPathNum;

//单元格具有的属性
struct Astarnode{
  double cost; //花费
  double costDA; //双向A*花费
  QPoint lastpoint; //前驱指针
  QPoint lastpointDA; //DA前驱指针
  bool isClosed;    //关闭列表标记
  bool isClosedDA;  //双向A*关闭列表标记
  int x;
  int y;
  int g;    //实际代价
  int gda;  //双向A*实际代价
  double h;    //预估代价
  double hda;  //双向A*预估代价
  bool isInOpenList;    //开放列表标记
  bool isInDAOpenList;  //双向A*开放列表标记
  bool visited;
  int blocks;   //障碍数
  int dfs;
  int pathflag;
  int glpa;
  int rhs; //LPA*预估实际代价
  int k1;
  int k2;
  double pheromone; // 信息素
};

struct point{ //贝塞尔曲线节点
  int x;
  int y;
};

struct CompareNode {
  bool operator()(const Astarnode& a, const Astarnode& b) const {
      // 假设我们根据cost来比较两个节点，需要的话可以根据你的具体需求来调整
      return a.cost > b.cost;
  }
};

// 蚂蚁结构
struct Ant {
  QVector<QPoint> path;
  double pathLength = 0;
};

class Astar : public MapLabel{
protected:
    //void runAstar();    //A*算法的主体函数
public:
    Astar(const QString &text, QWidget *parent=nullptr,int width=5,int height=5,int rectaa=30); //定义Astar类的属性：text用来接收地图信息、width，height分别表示阵列的宽高、rectaa表示单元格的边长
    void createRandmap();
    void initializeNode(Astarnode& node, int i, int j);
    double calculateHeuristic(int i, int j);
    void calculateBlocks(Astarnode& node, int x, int y);
    void runAstar();    //A*算法的主体函数
    void runDFS();
    void runBFS(Astarnode current);
    void runGBFS(Astarnode current);
    void runDijkstra(Astarnode current);
    void runTraditionalAStar(Astarnode current);
    void runOptimizeAstar(Astarnode current);
    void runDoubleAstar(Astarnode current, Astarnode currentDA);
    void runLPAstar();  //LPA*算法
    void runDstar();  //D*算法
    void runDlitestar();  //D*lite算法
    void initializeAnts(int numberOfAnts);
    void initializePheromones();
    QVector<QPoint> getNeighbors(const QPoint& current);
    void constructPath(Ant &ant);
    void updatePheromones();
    void evaporatePheromones();
    void removeDuplicates(QVector<QPoint>& path);
    void searchForShortestPath();
    void runACO(); //蚁群算法
    void nextpath();
    void clearways();
    void setdir(int a);
    void sethfunc(int a);
    void setAnalykey(bool isAnaly); //分析窗口开关
    void setIsbgset(bool isbg); //是否存在背景
    void setfactor(int a);
    void setbezier(int a);
    void showmin_dfs();
    void setpenalty(int a);
    void setmode(int a);
    void savemap(QString dir);
    void loadmap(QString dir);
private:
    QVector<Ant> ants;
    QVector<QVector<double>> pheromoneMatrix;
    const double initialPheromone = 1.0;
    const double evaporationRate = 0.5;
    const double pheromoneDeposit = 100.0;
    const int maxPathlen = 500; // 最大路径长度
    const int numAnts = 20; // 蚂蚁的数量
    const int maxIterations = 100; // 最大迭代次数
    std::default_random_engine generator;  // 随机数生成器

    Astarnode anode[120][120];  //用于记录阵列节点属性
    QList<Astarnode> openlist;  //开放列表，存放未访问节点
    QList<Astarnode> openlistDA; //双向A星终点开放列表
    QList<Astarnode> successlist;  //已访问列表
    QList<Astarnode> predecesslist;  //未访问列表
    QList<QList<Astarnode>> path; //dfs路径列表
    QList<point> controlPoints; //控制点列表
    QList<point> Astarpoints;
    int startx;
    int starty;
    int endx;
    int endy;
    int count=0; //用于记录搜索范围
    void testNewmap();
    void showMessage(const QString &message, const QString &iconPath);
    void dfs(int x ,int y ,QList<Astarnode>& p,int plen);
    void updateandpaint();
    void updateandpaintACO(const QVector<QPoint>& bestPath);
    void dfs_updateandpaint(QList<QList<Astarnode>> path,int index);
    long long comb(int n, int m);
    void putopenlist(QList<Astarnode> & list, Astarnode data,int isDA);  //将节点data放入列表list中
    void putLPAopenlist(QList<Astarnode> &list, Astarnode data);
    Astarnode getopenlist(QList<Astarnode> & list,int isDA);    //将开放列表中的首节点取出
    Astarnode getLPAopenlist(QList<Astarnode> &list);
    void resortopenlist(QList<Astarnode> &list, Astarnode data,int isDA);//存入并重排序openlist
    void Initialize(); //初始化
    bool CompareKey(QList<Astarnode> &list,Astarnode &n2); //比较两节点的key值大小 n1<n2返回true 否则false
    void calculateKey(Astarnode &n); //LPA*计算key中的k1,k2
    void updateRhs(Astarnode &n,int w,int h,int dir); //LPA*计算最小rhs
    void GofirstSearch(); //LPA*寻找最短路径 第一次搜索
    void AfterChangedSearch(); //第二次搜索
    bool intersect(QList<Astarnode>& openlist, QList<Astarnode>& openlistDA); //双向A* 查看是否有交集
    Astarnode findMinNode(QList<Astarnode>& openlist, QList<Astarnode>& openlistDA);
    double getBezier(QList<point> points,double t,int mode);
    bool issolved;
    bool isLPAsolved;
    bool isDstarsolved;
    bool isDlitesolved;
    bool isdfssolved;
    bool isacosolved;
    bool isoverflow;
    bool isAnalysis;
    bool isBgset;
    void notfound();
    int hfunc;
    int bezier;
    int dir;
    int factor;
    int penal; //用于存储拐角值
    int alpha;
    int beta;
    int alpha_third;
    int mode;
    int max_depth = 100;
    int index=0;
    double dynamic=1.0;  //动态权值
    double temp;
    int straight=0; //起点终点的直线距离
    int bezierNum=10;
};

#endif // ASTAR_H
