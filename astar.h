#ifndef ASTAR_H
#define ASTAR_H

#include <mainwindow.h>
#include <maplabel.h>
#include <QList>
#include <QRadioButton>
#include <QMessageBox>
#include <cmath>

//单元格具有的属性
struct Astarnode{
  double cost; //计算所得花费
  QPoint lastpoint; //前驱指针
  bool isClosed;    //关闭列表标记
  int x;
  int y;
  int g;    //实际代价
  int h;    //预估代价
  bool isInOpenList;    //开放列表标记
  bool visited;
  int blocks;   //障碍数
  int dfs;
  int pathflag;
};

class Astar : public MapLabel{
public:
    Astar(const QString &text, QWidget *parent=nullptr,int width=5,int height=5,int rectaa=30); //定义Astar类的属性：text用来接收地图信息、width，height分别表示阵列的宽高、rectaa表示单元格的边长
    void runAstar();    //A*算法的主体函数
    void nextpath();
    void clearways();
    void setdir(int a);
    void sethfunc(int a);
    void setfactor(int a);
    void setnormal(int a);
    void showmin_dfs();
    void setpenalty(int a);
    void setmode(int a);
    void savemap(QString dir);
    void loadmap(QString dir);
private:
    Astarnode anode[120][120];  //用于记录阵列节点属性
    QList<Astarnode> openlist;  //开放列表，存放未访问节点
    QList<QList<Astarnode>> path;
    int startx;
    int starty;
    int endx;
    int endy;
    void dfs(int x ,int y ,QList<Astarnode>& p);
    void updateandpaint();
    void dfs_updateandpaint(QList<QList<Astarnode>> path,int index);
    void putopenlist(QList<Astarnode> & list, Astarnode data);  //将节点data放入列表list中
    Astarnode getopenlist(QList<Astarnode> & list);             //将开放列表中的首节点取出
    void resortopenlist(QList<Astarnode> &list, Astarnode data);//存入并重排序openlist
    bool issolved;
    bool isdfssolved;
    int isnormal;
    void notfound();
    int hfunc;
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
};

#endif // ASTAR_H
