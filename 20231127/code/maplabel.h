//地图绘制头文件
#ifndef MAPLABEL_H
#define MAPLABEL_H

#define LEFT 1
#define RIGHT 2
#define NONE 0

#define SQUARE 20

#include<mainwindow.h>
#include<QMouseEvent>
#include<QDebug>
#include<QLabel>
#include<QList>
#include<QListIterator>
#include<QPicture>
#include<QPixmap>
#include<QRadioButton>
#include<QDataStream>
#include<QPainterPath>

struct MapData{ //存放地图信息，用于保存
    int sta[120][120];
    int w;
    int h;
    int rsize;
};

class MapLabel : public QLabel{
public:
    MapLabel(const QString &text, QWidget *parent=nullptr,int width=5,int height=5,int rectaa=30);  //rectaa表示单元格的边长
    void mousePressEvent(QMouseEvent *event);   //鼠标按下事件
    void mouseMoveEvent(QMouseEvent *event);    //鼠标移动事件
    void mouseReleaseEvent(QMouseEvent *event); //鼠标释放事件
    void init();    //地图初始化
    void setpainterstatus(int sta); //地图各单元格状态
    int w;
    int h;
    void setbackground(QImage &bg); //“打开背景”
    void clearbackground();         //“清除背景”
    QImage backpic;
    void setRect(int a);    //设置单元格大小
    int rsizeindex();
protected:
    int mousestatus;
    QPainterPath nullpath;
    QPainterPath brickspath;
    QPainterPath waypath;
    QPoint *start;
    QPoint *end;
    int status[120][120];   //status=0，1，2，3，4分别表示可通，障碍，已访问，起始点，最短路径节点
    void setMapPath();
    bool isinit=false;
    int aa; //当前鼠标所指向的第aa bb个块
    int bb;
    QPixmap mappic;
    int painterstatus;
    void paintnow();
    void paintnow(int x,int y,int sta,bool isUpdatenow=true);
    void setstartpoint(int x,int y);
    void setendpoint(int x,int y);
    bool shutevent;
    QLabel *bgl;
    QLabel *mapl;
    int recta;
    int square;
    int border;

};


#endif // MAPLABEL_H
