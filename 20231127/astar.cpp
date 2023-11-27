//A*算法实现
#include<mainwindow.h>
#include<astar.h>
#include<cmath>
#include<QFile>
#include<QTextStream>
#include<QIODevice>
#include<QFileInfo>
#include<QStack>
#include<QQueue>
#include<QDebug>
#include<setxyDialog.h>
#include<QVector>
#include<QColor>
#include<QElapsedTimer>
#include<QMessageBox>
#include<QThread>

//时间信息
double BfsTime;
double DijkstraTime;
double GbfsTime;
double normalAstarEuTime;
double normalAstarManTime;
double normalAstarDiaTime;
double doubleAstarEuTime;
double doubleAstarManTime;
double doubleAstarDiaTime;
double yydsAstarEuTime;
double yydsAstarManTime;
double yydsAstarDiaTime;
double DAstarTime;
double LPAstarTime;
double DliteTime;

//搜索范围
int BfsExtend;
int DijkstraExtend;
int GbfsExtend;
int normalAstarEuExtend;
int normalAstarManExtend;
int normalAstarDiaExtend;
int doubleAstarEuExtend;
int doubleAstarManExtend;
int doubleAstarDiaExtend;
int yydsAstarEuExtend;
int yydsAstarManExtend;
int yydsAstarDiaExtend;
int DAstarExtend;
int LPAstarExtend;
int DliteExtend;

//深搜
int dfsPathNum;

Astar::Astar(const QString &text, QWidget *parent,int width,int height,int rectaa) : MapLabel(text,parent,width,height,rectaa){
    //qDebug()<<w<<h;
    issolved=false;
    isLPAsolved=false;
    isdfssolved=false;
    isoverflow=false;
    isAnalysis=false;
    isBgset=false;
    dir=1;
    bezier=0;
    hfunc=1;
    dynamic=1;
    bezierNum=10;
}
//核心函数
void Astar::runAstar(){    
    if(issolved){
        QMessageBox::information(this,"Caution","请先清除当前路径",QMessageBox::Ok);
        return;
    }
    if(start->isNull() or end->isNull()){
        if(!isAnalysis) QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
        return;
    }

    //先清除扩展点列表
    openlist.clear();
    openlistDA.clear();
    //记录起点、终点的坐标信息
    startx=start->x();
    starty=start->y();
    endx=end->x();
    endy=end->y();

    //遍历阵列中的节点，初始化节点信息
    for(int i=1; i<=h; i++){
        for(int j=1; j<=w; j++){
            anode[i][j].g=0;
            anode[i][j].gda=0;
            anode[i][j].blocks=0;
            anode[i][j].dfs=0;
            anode[i][j].pathflag=0;

            switch(hfunc){  //选择预估距离计算公式
            case 1: //优化A* 切比雪夫
                anode[i][j].h=abs(abs(i-endx)-abs(j-endy))*10+(abs(i-endx)>abs(j-endy)?abs(j-endy)*14:abs(i-endx)*14)-14;
                break;
            case 2: //优化A* 曼哈顿
                anode[i][j].h=(abs(i-endx)+abs(j-endy))*10;
                break;
            case 3: //优化A* 欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            case 7: //传统A* 欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            case 8: //传统A* 曼哈顿
                anode[i][j].h=(abs(i-endx)+abs(j-endy))*10;
                break;
            case 9: //传统A* 切比雪夫
                anode[i][j].h=abs(abs(i-endx)-abs(j-endy))*10+(abs(i-endx)>abs(j-endy)?abs(j-endy)*14:abs(i-endx)*14)-14;
                break;
            case 10: //双向A* 欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                anode[i][j].hda=sqrt(((int)pow(i-startx,2)+(int)pow(j-starty,2)))*10;
                break;
            case 11: //双向A* 曼哈顿
                anode[i][j].h=(abs(i-endx)+abs(j-endy))*10;
                anode[i][j].hda=(abs(i-startx)+abs(j-starty))*10;
                break;
            case 12: //双向A* 切比雪夫
                anode[i][j].h=abs(abs(i-endx)-abs(j-endy))*10+(abs(i-endx)>abs(j-endy)?abs(j-endy)*14:abs(i-endx)*14)-14;
                anode[i][j].hda=abs(abs(i-startx)-abs(j-starty))*10+(abs(i-startx)>abs(j-starty)?abs(j-starty)*14:abs(i-startx)*14)-14;
                break;
            case 14: //GBFS 使用欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            case 23: //Dijkstra
                anode[i][j].h=0;
                break;
            }

            anode[i][j].cost=anode[i][j].h; //因为初始g都为0，所以不需要加，cost等于预估距离即可
            anode[i][j].costDA=anode[i][j].hda;
            anode[i][j].x=i;
            anode[i][j].y=j;
            anode[i][j].isInOpenList=false;
            anode[i][j].isInDAOpenList=false;
            if(factor==1){  //计算各点障碍数
                for(int op=1; op<=4; op++){
                    int optrx=i+((op+1)%2)*(op-3);
                    int optry=j+(op%2)*(op-2);
                    if(status[optrx][optry]==0) continue;
                    else if(status[optrx][optry]==1 or status[optrx][optry]==2){
                        anode[i][j].blocks++;
                    }
                }
            }
            if(status[i][j]==0){ //空白 表示可以正常通行
                anode[i][j].isClosed=false;
                anode[i][j].isClosedDA=false;
                anode[i][j].visited=false;  //初始化访问情况
            }
            if(status[i][j]==1 or status[i][j]==2){ //1代表障碍 2代表起点
                anode[i][j].isClosed=true;
                anode[i][j].visited=true;
                anode[i][j].dfs=1;
            }
            if(status[i][j]==1 or status[i][j]==3){ //3代表终点
                anode[i][j].isClosedDA=true;
            }
        }
    }
    straight=sqrt(((int)pow(startx-endx,2)+(int)pow(starty-endy,2)))*10;

    //现已初始化节点信息列表，开始获取最短路径
    Astarnode current,currentDA;  //表示当前节点
    int optrx;
    int optry;
    int DAoptrx;
    int DAoptry;
    current=anode[startx][starty];  //从起点开始
    currentDA=anode[endx][endy]; //从终点开始

    if(hfunc == 4){ //dfs
        path.clear(); //清空列表
        QList<Astarnode> p;
        isoverflow=false;
        dfs(startx, starty, p, 0); //初始路径长度为0
        isdfssolved=true;
        if(isdfssolved)
        {
            if(isoverflow){
                dfsPathNum=-1;
                return; //出现溢出 返回
            }
            dfsPathNum=path.size();
            dfs_updateandpaint(path,0);
        }
    }

    if(hfunc == 5){ //bfs
        QElapsedTimer time;
        time.start();
        count=0;
        QQueue<Astarnode> q; //定义一个队列
        bool visited[h+1][w+1];
        memset(visited, false, sizeof(visited));
        q.enqueue(anode[startx][starty]);
        visited[startx][starty] = true;
        while (!q.empty()) {
            current = q.front();
            q.dequeue();
            if (current.x == endx && current.y == endy)
            {
                int queuelen=q.size();
                for(int i=0;i<queuelen;i++)
                {
                    current = q.front();
                    q.dequeue();
                    status[current.x][current.y]=7; //队列中的待扩展点
                }
                break;
            }
            for (int i = 1; i <= 4; i++) {
                optrx = current.x + ((i + 1) % 2) * (i - 3);
                optry = current.y + (i % 2) * (i - 2);
                if (optrx <= 0 or optrx > h or optry <= 0 or optry > w) continue;
                if (anode[optrx][optry].isClosed) continue;
                if (!visited[optrx][optry]) {
                    q.enqueue(anode[optrx][optry]);
                    visited[optrx][optry] = true;
                    status[optrx][optry]=5;
                    count++;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                }
            }
        }
        double bfstime=time.nsecsElapsed();
        issolved=true;
        shutevent=true;
        updateandpaint();

        bfstime=bfstime/pow(10,6);
        BfsTime=bfstime;
        BfsExtend=count;
        //QMessageBox bfs_time;
        //double bfs=time.nsecsElapsed();
        //bfs_time.setWindowTitle("Dijkstra算法运行时间");
        //bfs_time.setText(QString("用时%1ms").arg(bfs));
        //bfs_time.setStandardButtons(QMessageBox::Ok);
        //bfs_time.exec();
    }

    if(hfunc==1 or hfunc==2 or hfunc==3) //优化A*
    {
        QElapsedTimer time;
        time.start();
        count=0;
        while(1){
            //dir=1为八方向，dir=2为四方向，八方向不用考虑拐角问题
            if(dir!=2){ //八方向，同四方向理
                for(int i=1; i<=4; i++){
                    optrx=current.x+(i>=3?1:-1);
                    optry=current.y+pow(-1,i);
                    if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                    if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue;
                    if(anode[optrx][optry].isClosed) continue;
                    if(anode[optrx][optry].g==0){
                        anode[optrx][optry].g=current.g+14; //对角走，距离为14
                        temp=double(anode[optrx][optry].h)/double(straight);
                        if(temp>0.5){
                            dynamic=1.3;
                        }
                        else{
                            dynamic=1;
                        }
                        anode[optrx][optry].cost=anode[optrx][optry].g+dynamic*anode[optrx][optry].h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        putopenlist(openlist,anode[optrx][optry],1);
                        anode[optrx][optry].isInOpenList=true;
                        //status[optrx][optry]=5;
                    }
                    else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                        anode[optrx][optry].g=current.g+14;
                        temp=double(anode[optrx][optry].h)/double(straight);
                        if(temp>0.5){
                            dynamic=1.3;
                        }
                        else{
                            dynamic=1;
                        }
                        anode[optrx][optry].cost=anode[optrx][optry].g+dynamic*anode[optrx][optry].h;  //f=g+dynamic*h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        resortopenlist(openlist,anode[optrx][optry],1);
                    }
                }
            }
            for(int i=1; i<=4; i++){ //四方向，i=1，2，3，4分别代表上右下左
                optrx=current.x+((i+1)%2)*(i-3);    //如果i是奇数，在x方向移动，y方向上不变
                optry=current.y+(i%2)*(i-2);        //如果i是偶数，在y方向移动，x方向上不变
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;    //判断是否到达边界
                if(anode[optrx][optry].isClosed) continue;
                int penalty = 0;
                int alpha = 0;
                int alpha_third =1;
                if(factor==2 || factor==1 || factor==3){
                    if(((factor == 2 || factor==3)and (current.lastpoint.x() - current.x) * (current.x - optrx) + (current.lastpoint.y() - current.y) * (current.y - optry) != 0)){
                        penalty = penal;
                    }
                    //选择障碍数因子alpha
                    if (factor == 1 || factor==3){
                        switch (mode) {
                        case 1: //静态alpha
                            switch(current.blocks){
                            case 0:
                                alpha = -10;
                                break;
                            case 1:
                                alpha = 1;
                                break;
                            case 2:
                                alpha = 5;
                                break;
                            case 3:
                                alpha = 10;
                                break;
                            case 4:
                                alpha = 20;
                                break;
                            }
                            break;
                        case 2: //考虑相邻节点
                            beta = current.blocks + anode[optrx][optry].blocks;
                            alpha = beta*10;
                            break;
                        case 3: //动态获取
                            alpha = 0;
                            alpha_third = current.blocks;
                            break;
                        }
                    }
                }
                if(anode[optrx][optry].g==0){ //新的扩展点
                    anode[optrx][optry].g=current.g+10; //计算它的实际距离g
                    temp=double(anode[optrx][optry].h)/double(straight);
                    if(temp>0.5){
                        dynamic=1.3;
                    }
                    else{
                        dynamic=1;
                    }
                    anode[optrx][optry].cost=anode[optrx][optry].g+alpha_third*dynamic*anode[optrx][optry].h-penalty+alpha;   //f=g+h
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1);  //将该相邻节点放入开放列表 开放列表存放可扩展节点
                    anode[optrx][optry].isInOpenList=true;
                }
                //如果当前节点的花费比相邻节点少，可以继续在当前节点的相邻节点查找
                else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+10;
                    temp=double(anode[optrx][optry].h)/double(straight);
                    if(temp>0.5){
                        dynamic=1.3;
                    }
                    else{
                        dynamic=1;
                    }
                    anode[optrx][optry].cost=anode[optrx][optry].g+alpha_third*dynamic*anode[optrx][optry].h-penalty+alpha;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);   //将该相邻节点放入openlist并重排序openlist
                }
            }
            if(openlist.isEmpty()){ //如果openlist为空，说明没有可行通路
                notfound();
                return;
            }
            if(anode[endx][endy].isInOpenList) //终点已被访问，说明已找到最短路径
            {
                int openlistlen=openlist.size();
                int i=0;
                while (i<openlistlen) {
                    if(openlist[i].x != endx or openlist[i].y != endy) status[openlist[i].x][openlist[i].y]=7; //标记待扩展点
                    i++;
                }
                break;
            }
            current=getopenlist(openlist,1);  //否则继续寻找，从openlist首节点开始，openlist是升序的
            status[current.x][current.y]=5;
            count++;
        }
        //前面已确保openlist不为空，因此至此问题必定解决
        issolved=true;
        shutevent=true;
        updateandpaint();   //最终输出阵列的函数
        double astar=time.nsecsElapsed();
        astar=astar/pow(10,6);
        if(hfunc==1){
            yydsAstarDiaTime=astar;
            yydsAstarDiaExtend=count;
        }else if(hfunc==2){
            yydsAstarManTime=astar;
            yydsAstarManExtend=count;
        }else if(hfunc==3){
            yydsAstarEuTime=astar;
            yydsAstarEuExtend=count;
        }
//        QMessageBox astar_time;
//        astar_time.setWindowTitle("优化A*运行时间");
//        astar_time.setText(QString("用时%1ms").arg(astar));
//        astar_time.setStandardButtons(QMessageBox::Ok);
//        astar_time.exec();
    }

    if(hfunc==7 or hfunc==8 or hfunc==9) //传统A*
    {
        QElapsedTimer time;
        time.start();
        count=0;
        while(1){
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    optrx=current.x+(i>=3?1:-1);
                    optry=current.y+pow(-1,i);
                    if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                    if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue;
                    if(anode[optrx][optry].isClosed) continue;
                    if(anode[optrx][optry].g==0){
                        anode[optrx][optry].g=current.g+14; //对角走，距离为14
                        anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        putopenlist(openlist,anode[optrx][optry],1);
                        anode[optrx][optry].isInOpenList=true;
                        //status[optrx][optry]=5;
                    }
                    else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                        anode[optrx][optry].g=current.g+14;
                        anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        resortopenlist(openlist,anode[optrx][optry],1);
                    }
                }
            }
            for(int i=1; i<=4; i++){ //四方向，i=1，2，3，4分别代表上右下左
                optrx=current.x+((i+1)%2)*(i-3);
                optry=current.y+(i%2)*(i-2);
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;    //判断是否到达边界
                if(anode[optrx][optry].isClosed) continue;
                if(anode[optrx][optry].g==0){ //新的扩展点
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;   //f=g+h
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1);
                    anode[optrx][optry].isInOpenList=true;
                }
                else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);
                }
            }
            if(openlist.isEmpty()){ //如果openlist为空，说明没有可行通路
                notfound();
                return;
            }
            if(anode[endx][endy].isInOpenList) //终点已被访问，说明已找到最短路径
            {
                int openlistlen=openlist.size();
                int i=0;
                while (i<openlistlen) {
                    if(openlist[i].x != endx or openlist[i].y != endy) status[openlist[i].x][openlist[i].y]=7; //标记待扩展点
                    i++;
                }
                break;
            }
            current=getopenlist(openlist,1);
            status[current.x][current.y]=5;
            count++;
        }
        issolved=true;
        shutevent=true;
        updateandpaint();
        double astar=time.nsecsElapsed();
        astar=astar/pow(10,6);
        if(hfunc==7){
            normalAstarEuTime=astar;
            normalAstarEuExtend=count;
        }else if(hfunc==8){
            normalAstarManTime=astar;
            normalAstarManExtend=count;
        }else if(hfunc==9){
            normalAstarDiaTime=astar;
            normalAstarDiaExtend=count;
        }
//        QMessageBox astar_time;
//        astar_time.setWindowTitle("传统A*运行时间");
//        astar_time.setText(QString("用时%1ms").arg(astar));
//        astar_time.setStandardButtons(QMessageBox::Ok);
//        astar_time.exec();
    }

    if(hfunc==10 or hfunc==11 or hfunc==12) //双向A*
    {
        QElapsedTimer time;
        time.start();
        count=0;
        while(1){ //从起点和从终点轮流进行扩展
            //从起点开始扩展
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    optrx=current.x+(i>=3?1:-1);
                    optry=current.y+pow(-1,i);
                    if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                    if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue;
                    if(anode[optrx][optry].isClosed) continue;
                    if(anode[optrx][optry].g==0){
                        anode[optrx][optry].g=current.g+14; //对角走，距离为14
                        anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        putopenlist(openlist,anode[optrx][optry],1);
                        anode[optrx][optry].isInOpenList=true;
                    }
                    else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                        anode[optrx][optry].g=current.g+14;
                        anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;  //f=g+dynamic*h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        resortopenlist(openlist,anode[optrx][optry],1);
                    }
                }
            }
            for(int i=1; i<=4; i++){
                optrx=current.x+((i+1)%2)*(i-3);
                optry=current.y+(i%2)*(i-2);
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                if(anode[optrx][optry].isClosed) continue;
                if(anode[optrx][optry].g==0){
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1);  //放入开放列表 等待扩展
                    anode[optrx][optry].isInOpenList=true;
                }
                else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);   //放入开放列表并重排序开放列表
                }
            }
            if(openlist.isEmpty()){
                notfound();
                return;
            }
            if(anode[endx][endy].isInOpenList)
            {
                int openlistlen=openlist.size();
                int i=0;
                while (i<openlistlen) {
                    if(openlist[i].x != endx or openlist[i].y != endy) status[openlist[i].x][openlist[i].y]=7; //标记待扩展点
                    i++;
                }
                break;
            }
            current=getopenlist(openlist,1);
            status[current.x][current.y]=5;
            count++;

            //从终点开始扩展
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    DAoptrx=currentDA.x+(i>=3?1:-1);
                    DAoptry=currentDA.y+pow(-1,i);
                    if(DAoptrx<=0 or DAoptrx>h or DAoptry<=0 or DAoptry>w) continue;
                    if(anode[currentDA.x][DAoptry].isClosedDA and anode[DAoptrx][currentDA.y].isClosedDA) continue;
                    if(anode[DAoptrx][DAoptry].isClosedDA) continue;
                    if(anode[DAoptrx][DAoptry].gda==0){
                        anode[DAoptrx][DAoptry].gda=currentDA.gda+14; //对角走，距离为14
                        anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                        anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                        anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                        putopenlist(openlistDA,anode[DAoptrx][DAoptry],2);
                        anode[DAoptrx][DAoptry].isInDAOpenList=true;
                    }
                    else if(anode[DAoptrx][DAoptry].gda>currentDA.gda+14 and anode[DAoptrx][DAoptry].isInDAOpenList){
                        anode[DAoptrx][DAoptry].gda=currentDA.gda+14;
                        anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;  //f=g+dynamic*h;
                        anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                        anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                        resortopenlist(openlistDA,anode[DAoptrx][DAoptry],2);
                    }
                }
            }
            for(int i=1; i<=4; i++){
                DAoptrx=currentDA.x+((i+1)%2)*(i-3);
                DAoptry=currentDA.y+(i%2)*(i-2);
                if(DAoptrx<=0 or DAoptrx>h or DAoptry<=0 or DAoptry>w) continue;
                if(anode[DAoptrx][DAoptry].isClosedDA) continue;
                if(anode[DAoptrx][DAoptry].gda==0){
                    anode[DAoptrx][DAoptry].gda=currentDA.gda+10;
                    anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                    anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                    anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                    putopenlist(openlistDA,anode[DAoptrx][DAoptry],2);  //放入开放列表 等待扩展
                    anode[DAoptrx][DAoptry].isInDAOpenList=true;
                }
                else if(anode[DAoptrx][DAoptry].gda>currentDA.gda+10 and anode[DAoptrx][DAoptry].isInDAOpenList){
                    anode[DAoptrx][DAoptry].gda=currentDA.gda+10;
                    anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                    anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                    anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                    resortopenlist(openlistDA,anode[DAoptrx][DAoptry],2);   //放入开放列表并重排序开放列表
                }
            }
            if(openlistDA.isEmpty()){
                notfound();
                return;
            }
            if(anode[startx][starty].isInDAOpenList)
            {
                int openlistDAlen=openlistDA.size();
                int i=0;
                while (i<openlistDAlen) {
                    if(openlistDA[i].x != startx or openlistDA[i].y != starty) status[openlistDA[i].x][openlistDA[i].y]=7; //标记待扩展点
                    i++;
                }
                break;
            }
            currentDA=getopenlist(openlistDA,2);
            status[currentDA.x][currentDA.y]=6;
            count++;

            if(intersect(openlist,openlistDA)) //判断openlist和openlistDA是否有交集 并将交集中cost最小的节点返回
            {
                Astarnode minNode;
                minNode=findMinNode(openlist,openlistDA); //找到交集中总代价最小的中间节点(交点)
                //qDebug("代价最小相交点:(%d,%d)",minNode.x,minNode.y);
                Astarnode temp=anode[minNode.x][minNode.y],temp1;
                while(1) //逆序openlistDA节点间的指针
                {
                    temp1=anode[temp.lastpointDA.x()][temp.lastpointDA.y()];
                    anode[temp1.x][temp1.y].lastpoint.setX(temp.x);
                    anode[temp1.x][temp1.y].lastpoint.setY(temp.y);
                    if(temp1.x==endx and temp1.y==endy) break;
                    temp=temp1;
                }
                int openlistlen=openlist.size();
                int openlistDAlen=openlistDA.size();
                int i=0;
                while (i<openlistlen) {
                    if(openlist[i].x != endx or openlist[i].y != endy) status[openlist[i].x][openlist[i].y]=7;
                    i++;
                }
                i=0;
                while (i<openlistDAlen) {
                    if(openlistDA[i].x != startx or openlistDA[i].y != starty) status[openlistDA[i].x][openlistDA[i].y]=7;
                    i++;
                }
                break;
            }
        }
        issolved=true;
        shutevent=true;
        updateandpaint();
        double astar=time.nsecsElapsed();
        astar=astar/pow(10,6);
        if(hfunc==10){
            doubleAstarEuTime=astar;
            doubleAstarEuExtend=count;
        }else if(hfunc==11){
            doubleAstarManTime=astar;
            doubleAstarManExtend=count;
        }else if(hfunc==12){
            doubleAstarDiaTime=astar;
            doubleAstarDiaExtend=count;
        }
//        QMessageBox astar_time;
//        astar_time.setWindowTitle("双向A*运行时间");
//        astar_time.setText(QString("用时%1ms").arg(astar));
//        astar_time.setStandardButtons(QMessageBox::Ok);
//        astar_time.exec();
    }

    if(hfunc==14) //最佳优先搜索GBFS
    {
        QElapsedTimer time;
        time.start();
        count=0;
        while(1){
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    optrx=current.x+(i>=3?1:-1);
                    optry=current.y+pow(-1,i);
                    if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                    if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue;
                    if(anode[optrx][optry].isClosed) continue;
                    if(anode[optrx][optry].g==0){
                        anode[optrx][optry].g=current.g+14; //对角走，距离为14
                        anode[optrx][optry].cost=anode[optrx][optry].h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        putopenlist(openlist,anode[optrx][optry],1);
                        anode[optrx][optry].isInOpenList=true;
                    }
                    else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                        anode[optrx][optry].g=current.g+14;
                        anode[optrx][optry].cost=anode[optrx][optry].h;  //f=h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        resortopenlist(openlist,anode[optrx][optry],1);
                    }
                }
            }
            for(int i=1; i<=4; i++){ //四方向
                optrx=current.x+((i+1)%2)*(i-3);
                optry=current.y+(i%2)*(i-2);
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                if(anode[optrx][optry].isClosed) continue;
                if(anode[optrx][optry].g==0){ //新的扩展点
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].h;   //f=h
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1);
                    anode[optrx][optry].isInOpenList=true;
                }
                else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].h;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);
                }
            }
            if(openlist.isEmpty()){
                notfound();
                return;
            }
            if(anode[endx][endy].isInOpenList) break;
            current=getopenlist(openlist,1);
            status[current.x][current.y]=5;
            count++;
        }
        double gbfstime=time.nsecsElapsed();
        gbfstime=gbfstime/pow(10,6);
        GbfsTime=gbfstime;
        GbfsExtend=count;
        issolved=true;
        shutevent=true;
        updateandpaint();
    }

    if(hfunc==23){ //Dijkstra算法
        QElapsedTimer time;
        time.start();
        count=0;
        while(1){
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    optrx=current.x+(i>=3?1:-1);
                    optry=current.y+pow(-1,i);
                    if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                    if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue;
                    if(anode[optrx][optry].isClosed) continue;
                    if(anode[optrx][optry].g==0){
                        anode[optrx][optry].g=current.g+14; //对角走，距离为14
                        anode[optrx][optry].cost=anode[optrx][optry].g;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        putopenlist(openlist,anode[optrx][optry],1);
                        anode[optrx][optry].isInOpenList=true;
                    }
                    else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                        anode[optrx][optry].g=current.g+14;
                        anode[optrx][optry].cost=anode[optrx][optry].g;  //f=g;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        resortopenlist(openlist,anode[optrx][optry],1);
                    }
                }
            }
            for(int i=1; i<=4; i++){ //四方向
                optrx=current.x+((i+1)%2)*(i-3);
                optry=current.y+(i%2)*(i-2);
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
                if(anode[optrx][optry].isClosed) continue;
                if(anode[optrx][optry].g==0){ //新的扩展点
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].g;   //f=g
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1);
                    anode[optrx][optry].isInOpenList=true;
                }
                else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+10;
                    anode[optrx][optry].cost=anode[optrx][optry].g;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);
                }
            }
            if(openlist.isEmpty()){
                notfound();
                return;
            }
            if(anode[endx][endy].isInOpenList) break;
            current=getopenlist(openlist,1);
            status[current.x][current.y]=5;
            count++;
        }
        double dijkstratime=time.nsecsElapsed();
        dijkstratime=dijkstratime/pow(10,6);
        DijkstraTime=dijkstratime;
        DijkstraExtend=count;
        issolved=true;
        shutevent=true;
        updateandpaint();
    }
}
void Astar::Initialize(){
    //先清除扩展点列表
    openlist.clear();
    //记录起点、终点的坐标信息
    startx=start->x();
    starty=start->y();
    endx=end->x();
    endy=end->y();
    //遍历阵列中的节点，初始化节点信息
    for(int i=1; i<=h; i++){
        for(int j=1; j<=w; j++){
            anode[i][j].g=10000; //将所有节点的g值设为无穷 先使用10000作为测试
            anode[i][j].rhs=10000;

            switch(hfunc){  //计算预估距离h
            case 15: //LPA* 欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            case 16: //LPA* 曼哈顿
                anode[i][j].h=(abs(i-endx)+abs(j-endy))*10;
                break;
            case 17: //LPA* 切比雪夫
                anode[i][j].h=abs(abs(i-endx)-abs(j-endy))*10+(abs(i-endx)>abs(j-endy)?abs(j-endy)*14:abs(i-endx)*14)-14;
                break;
            case 20: //Dlite 欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            case 21: //Dlite 曼哈顿
                anode[i][j].h=(abs(i-endx)+abs(j-endy))*10;
                break;
            case 22: //Dlite 切比雪夫
                anode[i][j].h=abs(abs(i-endx)-abs(j-endy))*10+(abs(i-endx)>abs(j-endy)?abs(j-endy)*14:abs(i-endx)*14)-14;
                break;
            }

            anode[i][j].cost=anode[i][j].g+anode[i][j].h;
            anode[i][j].x=i;
            anode[i][j].y=j;
            anode[i][j].isInOpenList=false;
            //anode[i][j].visited=false;

            if(status[i][j]==0){ //空白 表示可以正常通行
                anode[i][j].isClosed=false;
            }
            if(status[i][j]==1 or status[i][j]==2){ //1代表障碍 2代表起点
                anode[i][j].isClosed=true;
            }
        }
    }
    anode[startx][starty].rhs=0; //起点预估实际距离rhs为0
    calculateKey(anode[startx][starty]); //计算起点key值
    putLPAopenlist(openlist,anode[startx][starty]);
}
//LPA*计算节点的key
void Astar::calculateKey(Astarnode &n){
    n.k1=qMin(n.g,n.rhs)+n.h; //k1=f
    n.k2=qMin(n.g,n.rhs);     //k2=g
}
//LPA*更新节点的rhs值
void Astar::updateRhs(Astarnode &n,int w,int h,int dir){
    int minrhs=INT_MAX;
    int minx,miny; //最小的rhs值对应的前驱节点的坐标
    int optrx,optry;
    if(dir!=2){
        for(int i=1; i<=4; i++){ //八方向
            optrx=n.x+(i>=3?1:-1);
            optry=n.y+pow(-1,i);
            Astarnode anode[optrx][optry];
            if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
            if(anode[optrx][optry].g+14<minrhs){
                minrhs=anode[optrx][optry].g+14;
                minx=optrx;
                miny=optry;
            }
        }
    }
    for(int i=1; i<=4; i++){ //四方向，i=1，2，3，4分别代表上右下左
        optrx=n.x+((i+1)%2)*(i-3);
        optry=n.y+(i%2)*(i-2);
        Astarnode anode[optrx][optry];
        if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;
        if(anode[optrx][optry].g+10<minrhs){
            minrhs=anode[optrx][optry].g+10;
            minx=optrx;
            miny=optry;
        }
    }
    n.rhs=minrhs; //更新rhs值
    n.lastpoint.setX(minx); //更新前驱节点
    n.lastpoint.setY(miny);
}
bool Astar::CompareKey(QList<Astarnode> &list,Astarnode &n2){
    if(list[0].k1>n2.k1) return false;
    if(list[0].k1<n2.k1) return true;
    if(list[0].k1==n2.k1){
        if(list[0].k2<=n2.k2) return true;
        else return false;
    }
    return false;
}
void Astar::GofirstSearch(){
    Initialize(); //初始化
    QElapsedTimer time;
    time.start();
    count=0;
    Astarnode current;  //表示当前节点
    int optrx;
    int optry;
    current=anode[startx][starty];  //从起点开始
    while(1){
        if(dir!=2){ //八方向
            for(int i=1; i<=4; i++){
                optrx=current.x+(i>=3?1:-1);
                optry=current.y+pow(-1,i);
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue; //遇到边界
                if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue; //遇到交叉 防止穿墙
                if(anode[optrx][optry].isClosed) continue; //遇到障碍
                if(anode[optrx][optry].g==10000){ //新的扩展点
                    anode[optrx][optry].g=current.g+14; //对角走，距离为14
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    //注意 与A*不同 需先计算rhs 才能计算key
                    //calculateKey(anode[optrx][optry]); //计算当前节点的key
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1); //按key大小入队
                    anode[optrx][optry].isInOpenList=true;
                }
                else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+14;
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);
                }
            }
        }
        for(int i=1; i<=4; i++){ //四方向，i=1，2，3，4分别代表上右下左
            optrx=current.x+((i+1)%2)*(i-3);
            optry=current.y+(i%2)*(i-2);
            if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;    //判断是否到达边界
            if(anode[optrx][optry].isClosed) continue; //判断障碍
            if(anode[optrx][optry].g==10000){ //新的扩展点
                anode[optrx][optry].g=current.g+10;
                anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;   //f=g+h
                //注意 与A*不同 需先计算rhs 才能计算key
                //calculateKey(anode[optrx][optry]);
                anode[optrx][optry].lastpoint.setX(current.x);
                anode[optrx][optry].lastpoint.setY(current.y);
                putopenlist(openlist,anode[optrx][optry],1); //按key大小入队
                anode[optrx][optry].isInOpenList=true;
            }
            else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                anode[optrx][optry].g=current.g+10;
                anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                anode[optrx][optry].lastpoint.setX(current.x);
                anode[optrx][optry].lastpoint.setY(current.y);
                resortopenlist(openlist,anode[optrx][optry],1);
            }
        }
        if(openlist.isEmpty()){ //如果openlist为空，说明没有可行通路
            notfound();
            return;
        }
        if(anode[endx][endy].isInOpenList) //终点已被访问，说明已找到最短路径
        {
            int openlistlen=openlist.size();
            int i=0;
            while (i<openlistlen) {
                if(openlist[i].x != endx or openlist[i].y != endy) status[openlist[i].x][openlist[i].y]=7; //标记待扩展点
                i++;
            }
            break;
        }
        current=getLPAopenlist(openlist); //将队列首节点出队进行扩展
        status[current.x][current.y]=5; //出列 说明进行扩展 设置扩展点状态
        count++;
    }
    issolved=true;
    if(hfunc==15 or hfunc==16 or hfunc==17){
        isLPAsolved=true;
        isLPArunning=true;
        double Lpafirst=time.nsecsElapsed();
        LPAstarTime=Lpafirst/pow(10,6);
        LPAstarExtend=count;
    }
    else if(hfunc==20 or hfunc==21 or hfunc==22){
        isDlitesolved=true;
        isDliterunning=true;
        double Dlitefirst=time.nsecsElapsed();
        DliteTime=Dlitefirst/pow(10,6);
        DliteExtend=count;
    }
    updateandpaint();
}
void Astar::AfterChangedSearch(){
    //qDebug("Second search");
    Astarnode current;  //表示当前节点
    int optrx;
    int optry;
    current=anode[startx][starty];  //从起点开始
    while(1){
        if(dir!=2){ //八方向
            for(int i=1; i<=4; i++){
                optrx=current.x+(i>=3?1:-1);
                optry=current.y+pow(-1,i);
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue; //遇到边界
                if(anode[current.x][optry].isClosed and anode[optrx][current.y].isClosed) continue; //遇到交叉 防止穿墙
                if(anode[optrx][optry].isClosed) continue; //遇到障碍
                if(anode[optrx][optry].g==10000){ //新的扩展点
                    anode[optrx][optry].g=current.g+14; //对角走，距离为14
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    //注意 与A*不同 需先计算rhs 才能计算key
                    //calculateKey(anode[optrx][optry]); //计算当前节点的key
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry],1); //按key大小入队
                    anode[optrx][optry].isInOpenList=true;
                }
                else if(anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+14;
                    anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry],1);
                }
            }
        }
        for(int i=1; i<=4; i++){ //四方向，i=1，2，3，4分别代表上右下左
            optrx=current.x+((i+1)%2)*(i-3);
            optry=current.y+(i%2)*(i-2);
            if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;    //判断是否到达边界
            if(anode[optrx][optry].isClosed) continue; //判断障碍
            if(anode[optrx][optry].g==10000){ //新的扩展点
                anode[optrx][optry].g=current.g+10;
                anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;   //f=g+h
                //注意 与A*不同 需先计算rhs 才能计算key
                //calculateKey(anode[optrx][optry]);
                anode[optrx][optry].lastpoint.setX(current.x);
                anode[optrx][optry].lastpoint.setY(current.y);
                putopenlist(openlist,anode[optrx][optry],1); //按key大小入队
                anode[optrx][optry].isInOpenList=true;
            }
            else if(anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                anode[optrx][optry].g=current.g+10;
                anode[optrx][optry].cost=anode[optrx][optry].g+anode[optrx][optry].h;
                anode[optrx][optry].lastpoint.setX(current.x);
                anode[optrx][optry].lastpoint.setY(current.y);
                resortopenlist(openlist,anode[optrx][optry],1);
            }
        }
        if(openlist.isEmpty()){ //如果openlist为空，说明没有可行通路
            notfound();
            return;
        }
        if(anode[endx][endy].isInOpenList) //终点已被访问，说明已找到最短路径
        {
            int openlistlen=openlist.size();
            int i=0;
            if(hfunc==20 or hfunc==21 or hfunc==22){
                if(openlistlen>20) openlistlen-=10;
            }
            while (i<openlistlen) {
                if(openlist[i].x != endx or openlist[i].y != endy) status[openlist[i].x][openlist[i].y]=8; //标记待扩展点
                i++;
            }
            break;
        }
        current=getLPAopenlist(openlist); //将队列首节点出队进行扩展
        status[current.x][current.y]=5; //出列 说明进行扩展 设置扩展点状态
    }
    issolved=true;
    if(hfunc==15 or hfunc==16 or hfunc==17){
        isLPAsolved=true;
        isLPArunning=true;
        isLPAMapChanged=false;
    }else if(hfunc==20 or hfunc==21 or hfunc==22){
        isDlitesolved=true;
        isDliterunning=true;
        isDliteMapChanged=false;
    }
    updateandpaint();
}
void Astar::runLPAstar() //LPA*算法
{
    if(!isLPAMapChanged){
        //qDebug("check LPA*");
        isDstarrunning=false;
        isDliterunning=false;
        if(isLPAsolved) clearways(); //先清除地图
        if(issolved){
            QMessageBox::information(this,"Caution","请先清除当前路径",QMessageBox::Ok);
            return;
        }
        if(start->isNull() or end->isNull()){
            if(!isAnalysis) QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
            return;
        }
        GofirstSearch(); //第一次搜索
    }
    else{ //地图发生改变 第二次搜索
        //qDebug("The map of LPA* is changed.");
        if(isLPAsolved) clearways(); //先清除地图
        //qDebug("修改点:(%d,%d)",aa,bb);
        //for all directed edges(u,v) with changed edge costs
        //  Update the edge cost c(u,v);
        //  UpdateVertex(v);
        Initialize();
        AfterChangedSearch();
    }
}
void Astar::runDstar(){ //D*算法
    if(!isDMapChanged){
        qDebug("chack Dstar");
        isLPArunning=false;
        isDliterunning=false;
        if(issolved){
            QMessageBox::information(this,"Caution","请先清除当前路径",QMessageBox::Ok);
            return;
        }
        if(start->isNull() or end->isNull()){
            QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
            return;
        }
        //先清除扩展点列表
        openlistDA.clear();
        //记录起点、终点的坐标信息
        startx=start->x();
        starty=start->y();
        endx=end->x();
        endy=end->y();

        //遍历阵列中的节点，初始化节点信息
        for(int i=1; i<=h; i++){
            for(int j=1; j<=w; j++){
                anode[i][j].gda=0;

                switch(hfunc){  //选择预估距离计算公式
                case 13: //D* 欧式距离
                    anode[i][j].hda=sqrt(((int)pow(i-startx,2)+(int)pow(j-starty,2)))*10;
                    break;
                case 18: //D* 曼哈顿
                    anode[i][j].hda=(abs(i-startx)+abs(j-starty))*10;
                    break;
                case 19: //D* 切比雪夫
                    anode[i][j].hda=abs(abs(i-startx)-abs(j-starty))*10+(abs(i-startx)>abs(j-starty)?abs(j-starty)*14:abs(i-startx)*14)-14;
                    break;
                }

                anode[i][j].costDA=anode[i][j].hda;
                anode[i][j].x=i;
                anode[i][j].y=j;
                anode[i][j].isInDAOpenList=false;

                if(status[i][j]==0){ //空白 表示可以正常通行
                    anode[i][j].isClosedDA=false;
                }
                if(status[i][j]==1 or status[i][j]==3){ //3代表终点
                    anode[i][j].isClosedDA=true;
                }
            }
        }

        //现已初始化节点信息列表，开始获取最短路径
        Astarnode currentDA;  //表示当前节点
        int DAoptrx;
        int DAoptry;
        currentDA=anode[endx][endy]; //从终点开始
        QElapsedTimer time;
        time.start();
        count=0;
        while (1) {
            //从终点开始扩展
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    DAoptrx=currentDA.x+(i>=3?1:-1);
                    DAoptry=currentDA.y+pow(-1,i);
                    if(DAoptrx<=0 or DAoptrx>h or DAoptry<=0 or DAoptry>w) continue;
                    if(anode[currentDA.x][DAoptry].isClosedDA and anode[DAoptrx][currentDA.y].isClosedDA) continue;
                    if(anode[DAoptrx][DAoptry].isClosedDA) continue;
                    if(anode[DAoptrx][DAoptry].gda==0){
                        anode[DAoptrx][DAoptry].gda=currentDA.gda+14; //对角走，距离为14
                        anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                        anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                        anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                        putopenlist(openlistDA,anode[DAoptrx][DAoptry],2);
                        anode[DAoptrx][DAoptry].isInDAOpenList=true;
                    }
                    else if(anode[DAoptrx][DAoptry].gda>currentDA.gda+14 and anode[DAoptrx][DAoptry].isInDAOpenList){
                        anode[DAoptrx][DAoptry].gda=currentDA.gda+14;
                        anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;  //f=g+dynamic*h;
                        anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                        anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                        resortopenlist(openlistDA,anode[DAoptrx][DAoptry],2);
                    }
                }
            }
            for(int i=1; i<=4; i++){
                DAoptrx=currentDA.x+((i+1)%2)*(i-3);
                DAoptry=currentDA.y+(i%2)*(i-2);
                if(DAoptrx<=0 or DAoptrx>h or DAoptry<=0 or DAoptry>w) continue;
                if(anode[DAoptrx][DAoptry].isClosedDA) continue;
                if(anode[DAoptrx][DAoptry].gda==0){
                    anode[DAoptrx][DAoptry].gda=currentDA.gda+10;
                    anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                    anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                    anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                    putopenlist(openlistDA,anode[DAoptrx][DAoptry],2);  //放入开放列表 等待扩展
                    anode[DAoptrx][DAoptry].isInDAOpenList=true;
                }
                else if(anode[DAoptrx][DAoptry].gda>currentDA.gda+10 and anode[DAoptrx][DAoptry].isInDAOpenList){
                    anode[DAoptrx][DAoptry].gda=currentDA.gda+10;
                    anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                    anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                    anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                    resortopenlist(openlistDA,anode[DAoptrx][DAoptry],2);   //放入开放列表并重排序开放列表
                }
            }
            if(openlistDA.isEmpty()){
                notfound();
                return;
            }
            if(anode[startx][starty].isInDAOpenList)
            {
                int openlistDAlen=openlistDA.size();
                int i=0;
                while (i<openlistDAlen) {
                    if(openlistDA[i].x != startx or openlistDA[i].y != starty) status[openlistDA[i].x][openlistDA[i].y]=7; //标记待扩展点
                    i++;
                }
                break;
            }
            currentDA=getopenlist(openlistDA,2);
            status[currentDA.x][currentDA.y]=5;
            count++;
        }

        Astarnode temp=anode[startx][starty],temp1;
        while(1) //逆序openlistDA节点间的指针
        {
            temp1=anode[temp.lastpointDA.x()][temp.lastpointDA.y()];
            anode[temp1.x][temp1.y].lastpoint.setX(temp.x);
            anode[temp1.x][temp1.y].lastpoint.setY(temp.y);
            if(temp1.x==endx and temp1.y==endy) break;
            temp=temp1;
        }
        int openlistDAlen=openlistDA.size();
        int i=0;
        while (i<openlistDAlen) {
            if(openlistDA[i].x != startx or openlistDA[i].y != starty) status[openlistDA[i].x][openlistDA[i].y]=7;
            i++;
        }
        double Dstarfirst=time.nsecsElapsed();
        DAstarTime=Dstarfirst/pow(10,6);
        DAstarExtend=count;
        issolved=true;
        isDstarsolved=true;
        isDstarrunning=true;
        updateandpaint();
    }
    else{
        qDebug("The map of Dstar changed.");
        if(isDstarsolved){
            clearways();
            isDstarsolved=false;
        }
        //先清除扩展点列表
        openlistDA.clear();
        //记录起点、终点的坐标信息
        startx=start->x();
        starty=start->y();
        endx=end->x();
        endy=end->y();

        //遍历阵列中的节点，初始化节点信息
        for(int i=1; i<=h; i++){
            for(int j=1; j<=w; j++){
                anode[i][j].gda=0;

                switch(hfunc){  //选择预估距离计算公式
                case 13: //双向A* 欧式距离
                    anode[i][j].hda=sqrt(((int)pow(i-startx,2)+(int)pow(j-starty,2)))*10;
                    break;
                case 18: //双向A* 曼哈顿
                    anode[i][j].hda=(abs(i-startx)+abs(j-starty))*10;
                    break;
                case 19: //双向A* 切比雪夫
                    anode[i][j].hda=abs(abs(i-startx)-abs(j-starty))*10+(abs(i-startx)>abs(j-starty)?abs(j-starty)*14:abs(i-startx)*14)-14;
                    break;
                }

                anode[i][j].costDA=anode[i][j].hda;
                anode[i][j].x=i;
                anode[i][j].y=j;
                anode[i][j].isInDAOpenList=false;

                if(status[i][j]==0){ //空白 表示可以正常通行
                    anode[i][j].isClosedDA=false;
                }
                if(status[i][j]==1 or status[i][j]==3){ //3代表终点
                    anode[i][j].isClosedDA=true;
                }
            }
        }

        //现已初始化节点信息列表，开始获取最短路径
        Astarnode currentDA;  //表示当前节点
        int DAoptrx;
        int DAoptry;
        currentDA=anode[endx][endy]; //从终点开始
        while (1) {
            //从终点开始扩展
            if(dir!=2){ //八方向
                for(int i=1; i<=4; i++){
                    DAoptrx=currentDA.x+(i>=3?1:-1);
                    DAoptry=currentDA.y+pow(-1,i);
                    if(DAoptrx<=0 or DAoptrx>h or DAoptry<=0 or DAoptry>w) continue;
                    if(anode[currentDA.x][DAoptry].isClosedDA and anode[DAoptrx][currentDA.y].isClosedDA) continue;
                    if(anode[DAoptrx][DAoptry].isClosedDA) continue;
                    if(anode[DAoptrx][DAoptry].gda==0){
                        anode[DAoptrx][DAoptry].gda=currentDA.gda+14; //对角走，距离为14
                        anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                        anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                        anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                        putopenlist(openlistDA,anode[DAoptrx][DAoptry],2);
                        anode[DAoptrx][DAoptry].isInDAOpenList=true;
                    }
                    else if(anode[DAoptrx][DAoptry].gda>currentDA.gda+14 and anode[DAoptrx][DAoptry].isInDAOpenList){
                        anode[DAoptrx][DAoptry].gda=currentDA.gda+14;
                        anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;  //f=g+dynamic*h;
                        anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                        anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                        resortopenlist(openlistDA,anode[DAoptrx][DAoptry],2);
                    }
                }
            }
            for(int i=1; i<=4; i++){
                DAoptrx=currentDA.x+((i+1)%2)*(i-3);
                DAoptry=currentDA.y+(i%2)*(i-2);
                if(DAoptrx<=0 or DAoptrx>h or DAoptry<=0 or DAoptry>w) continue;
                if(anode[DAoptrx][DAoptry].isClosedDA) continue;
                if(anode[DAoptrx][DAoptry].gda==0){
                    anode[DAoptrx][DAoptry].gda=currentDA.gda+10;
                    anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                    anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                    anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                    putopenlist(openlistDA,anode[DAoptrx][DAoptry],2);  //放入开放列表 等待扩展
                    anode[DAoptrx][DAoptry].isInDAOpenList=true;
                }
                else if(anode[DAoptrx][DAoptry].gda>currentDA.gda+10 and anode[DAoptrx][DAoptry].isInDAOpenList){
                    anode[DAoptrx][DAoptry].gda=currentDA.gda+10;
                    anode[DAoptrx][DAoptry].costDA=anode[DAoptrx][DAoptry].gda+anode[DAoptrx][DAoptry].hda;
                    anode[DAoptrx][DAoptry].lastpointDA.setX(currentDA.x);
                    anode[DAoptrx][DAoptry].lastpointDA.setY(currentDA.y);
                    resortopenlist(openlistDA,anode[DAoptrx][DAoptry],2);   //放入开放列表并重排序开放列表
                }
            }
            if(openlistDA.isEmpty()){
                notfound();
                return;
            }
            if(anode[startx][starty].isInDAOpenList)
            {
                int openlistDAlen=openlistDA.size();
                int i=0;
                while (i<openlistDAlen) {
                    if(openlistDA[i].x != startx or openlistDA[i].y != starty) status[openlistDA[i].x][openlistDA[i].y]=8; //标记待扩展点
                    i++;
                }
                break;
            }
            currentDA=getopenlist(openlistDA,2);
        }

        Astarnode temp=anode[startx][starty],temp1;
        while(1) //逆序openlistDA节点间的指针
        {
            temp1=anode[temp.lastpointDA.x()][temp.lastpointDA.y()];
            anode[temp1.x][temp1.y].lastpoint.setX(temp.x);
            anode[temp1.x][temp1.y].lastpoint.setY(temp.y);
            if(temp1.x==endx and temp1.y==endy) break;
            temp=temp1;
        }
        issolved=true;
        isDstarsolved=true;
        isDstarrunning=true;
        isDMapChanged=false;
        updateandpaint();
    }
}
void Astar::runDlitestar(){ //D*lite算法
    if(!isDliteMapChanged){
        isDstarrunning=false;
        isLPArunning=false;
        if(isDlitesolved) clearways(); //先清除地图
        if(issolved){
            QMessageBox::information(this,"Caution","请先清除当前路径",QMessageBox::Ok);
            return;
        }
        if(start->isNull() or end->isNull()){
            if(!isAnalysis) QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
            return;
        }
        GofirstSearch(); //第一次搜索
    }
    else{ //地图发生改变 第二次搜索
        if(isDlitesolved) clearways(); //先清除地图
        Initialize();
        AfterChangedSearch();
    }
}
//检查两个开放列表是否有交集
bool Astar::intersect(QList<Astarnode>& openlist, QList<Astarnode>& openlistDA){
    for(int i=0; i<openlist.size(); i++){
        Astarnode node = openlist[i];
        for(int j=0; j<openlistDA.size(); j++){
            Astarnode nodeDA = openlistDA[j];
            if(node.x == nodeDA.x && node.y == nodeDA.y){
                return true;
            }
        }
    }
    return false;
}
Astarnode Astar::findMinNode(QList<Astarnode>& openlist, QList<Astarnode>& openlistDA){
    Astarnode minNode;
    int minCost = INT_MAX;
    for(int i=0; i<openlist.size(); i++){
        Astarnode node = openlist[i];
        for(int j=0; j<openlistDA.size(); j++){
            Astarnode nodeDA = openlistDA[j];
            if(node.x == nodeDA.x && node.y == nodeDA.y){ //相交点
                int cost = node.g + nodeDA.gda; //计算两个节点的总花费
                if(cost < minCost){ //如果总花费小于当前最小花费，更新最小节点和最小花费
                    minNode = node;
                    minCost = cost;
                }
            }
        }
    }
    return minNode; //返回最小节点
}
void Astar::dfs(int x ,int y ,QList<Astarnode>& p,int plen) {
    if(isoverflow) return;
    if(plen>150) return; //当前路径长度过长 丢弃
    if (x == endx && y == endy) {
        if(path.size()>10)
        {
            QMessageBox::information(this,"qwq","当前地图不适合使用深度优先!",QMessageBox::Ok);
            path.clear(); //清除路径列表 保护nextpath
            isoverflow=true;
            return;
        }
        path.push_back(p);
        return;
    }

    for (int i = 1; i <= 4; i++) {
        int nx = x + ((i + 1) % 2) * (i - 3);
        int ny = y + (i % 2) * (i - 2);
        int nh = anode[nx][ny].dfs;

        if (nx <= w && nx > 0 && ny <= h && ny > 0 && nh != -1 && nh != 1) {
            p.push_back(anode[nx][ny]);
            anode[nx][ny].dfs = -1;
            plen++;
            dfs(nx ,ny ,p ,plen);
            plen--;
            anode[nx][ny].dfs = 0;
            p.pop_back();
        }
    }
}
long long Astar::comb(int n, int m)
{
    if (m < n - m) m = n - m;
    long long ans = 1;
    for (int i = m + 1; i <= n; i++) ans *= i;
    for (int i = 1; i <= n - m; i++) ans /= i;
    return ans;
}
double Astar::getBezier(QList<point> points,double t,int mode)
{
    double bezier=0.0;
    int Bezierlen=controlPoints.size();
    if(mode==1) //mode为1 计算x坐标
    {
        for(int i=0;i<Bezierlen;i++)
        {
            bezier+=comb(Bezierlen-1,i)*points[i].x*pow(1-t,Bezierlen-1-i)*pow(t,i);
        }
    }
    else //计算y坐标
    {
        for(int i=0;i<Bezierlen;i++)
        {
            bezier+=comb(Bezierlen-1,i)*points[i].y*pow(1-t,Bezierlen-1-i)*pow(t,i);
        }
    }
    return bezier;
}
void Astar::dfs_updateandpaint(QList<QList<Astarnode>> path, int index){
    if (path.isEmpty()) {
        notfound();
        return;
    }
    QList<Astarnode> first_path = path[index]; //存储path列表中的第一条路径

    QPainter painter(&mappic);
    painter.setPen(Qt::black);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBrush(QBrush(QColor("#ffe100"))); //设置画刷颜色

    for (int i = 0; i < first_path.size(); i++) { //绘制最短路径单元格
        if(first_path[i].x==startx and first_path[i].y==starty){
            continue;
        }
        if(first_path[i].x==endx and first_path[i].y==endy){
            continue;
        }
        painter.drawRoundedRect(border+(first_path[i].y-1)*recta,border+(first_path[i].x-1)*recta,square,square,square/4,square/4); //在该节点上绘制一个矩形
        anode[first_path[i].x][first_path[i].y].pathflag=1;
    }
    if(bezier==1 && dir==2) //绘制贝塞尔曲线 路径平滑处理 暂时只支持四方向
    {
        //QList<Astarnode> controlPoints;
        painter.setPen(QPen(Qt::red, 2.5, Qt::SolidLine, Qt::RoundCap));
        int i=1;
        Astarnode Beginnode;
        Beginnode.x=startx;
        Beginnode.y=starty;
        first_path.prepend(Beginnode);
        int currentx = border + (starty - 1) * recta + square / 2;
        int currenty = border + (startx - 1) * recta + square / 2;
        while(i < first_path.size()) { //遍历所有的节点
            //qDebug("%d(%d,%d)",i,first_path[i].x,first_path[i].y);
            if(first_path[i].x==endx and first_path[i].y==endy){  //注意：dfs写的时候x和y轴反过来了
                if(controlPoints.size()!=0)
                {
                    //考虑到如果一直都是conner
                    for(int t=1;t<=bezierNum;t++)
                    {
                        double temp=double(t)/bezierNum;
                        double tempx=getBezier(controlPoints,temp,1);
                        double tempy=getBezier(controlPoints,temp,2);
                        //qDebug("%f,%f",tempx,tempy);
                        painter.drawLine(currentx,currenty,tempx,tempy);
                        currentx=tempx;
                        currenty=tempy;
                    }
                    controlPoints.clear();
                }
                int bendx = border + (endy - 1) * recta + square / 2;
                int bendy = border + (endx - 1) * recta + square / 2;
                painter.drawLine(currentx,currenty,bendx,bendy);
                break;
            }

            point p0,p1,p2;
            p0.x = border + (first_path[i-1].y - 1) * recta + square / 2; //计算第i-1个节点的中心点的x坐标
            p0.y = border + (first_path[i-1].x - 1) * recta + square / 2; //计算第i-1个节点的中心点的y坐标
            p1.x = border + (first_path[i].y - 1) * recta + square / 2; //计算第i个节点的中心点的x坐标
            p1.y = border + (first_path[i].x - 1) * recta + square / 2; //计算第i个节点的中心点的y坐标
            p2.x = border + (first_path[i + 1].y - 1) * recta + square / 2; //计算第i+1个节点的中心点的x坐标
            p2.y = border + (first_path[i + 1].x - 1) * recta + square / 2; //计算第i+1个节点的中心点的y坐标

            if((p0.x==p1.x && p1.x==p2.x) || (p0.y==p1.y && p1.y==p2.y))
            {
                if(controlPoints.size()==0)
                {
                    i++;
                    continue;
                }
                else
                {
                    //利用控制点绘制贝塞尔曲线
                    for(int t=1;t<=bezierNum;t++)
                    {
                        double temp=double(t)/bezierNum;
                        double tempx=getBezier(controlPoints,temp,1);
                        double tempy=getBezier(controlPoints,temp,2);
                        //qDebug("%f,%f",tempx,tempy);
                        painter.drawLine(currentx,currenty,tempx,tempy);
                        currentx=tempx;
                        currenty=tempy;
                    }
                    //绘制结束清空控制点列表
                    controlPoints.clear();
                    i++;
                    continue;
                }
            }
            else
            {
                point c1,c2;
                c1.x=(p0.x+p1.x)/2;
                c1.y=(p0.y+p1.y)/2;
                c2.x=(p1.x+p2.x)/2;
                c2.y=(p1.y+p2.y)/2;

                if(controlPoints.size()==0) //控制点列表为空
                {
                    painter.drawLine(currentx,currenty,p0.x,p0.y); //遇到conner首先连接到x0
                    currentx=p0.x;
                    currenty=p0.y;

                    //添加四个控制点
                    controlPoints.append(p0);
                    controlPoints.append(c1);
                    controlPoints.append(c2);
                    controlPoints.append(p2);
                }
                else
                {
                    controlPoints.removeLast(); //删除尾节点
                    //添加两个控制点
                    controlPoints.append(c2);
                    controlPoints.append(p2);
                }
            }
            i++;
        }
    }
    painter.end();
    mapl->setPixmap(mappic);
}
void Astar::nextpath(){  //下一条路径
    //qDebug()<<"nextpath";
    if(start->isNull() or end->isNull()){
        QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
        return;
    }
    if(!isdfssolved) return;
    for (int i = 1; i <= h; i++) { //先清除当前路径
        for (int j = 1; j <= w; j++) {
            if (anode[i][j].pathflag==1) {
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);  //清除路径
            }
            if(status[i][j]==2) paintnow(i,j,2,false);  //重绘起点
            if(status[i][j]==3) paintnow(i,j,3,false);  //重绘终点
        }
    }
    mapl->setPixmap(mappic);    //地图数据更新
    index++;
    //qDebug("index：%d",index);
    if(index<path.size()){
        dfs_updateandpaint(path,index);
    }
    else{
        index=index%path.size();
        dfs_updateandpaint(path,index);
    }
}
void Astar::showmin_dfs(){  //显示dfs最短路径
    if(path.isEmpty()){
        QMessageBox::information(this,"qwq","路径列表为空！",QMessageBox::Ok);
        return;
    }
    int min=1000;
    int num=0;
    for(int i=0;i<path.size();i++){
        if(path[i].size()<=min){
            min=path[i].size();
            num=i;
        }
    }
    for (int i = 1; i <= h; i++) { //先清除当前路径
        for (int j = 1; j <= w; j++) {
            if (anode[i][j].pathflag==1) {
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);
            }
        }
    }
    mapl->setPixmap(mappic);

    for (int i = 1; i <= h; i++) {  //先清除地图路径 否则路径连线会受影响
        for (int j = 1; j <= w; j++) {
            if (anode[i][j].pathflag==1) {
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);  //将对应的单元绘制为白色
            }
            if(status[i][j]==2) paintnow(i,j,2,false);  //将对应的单元绘制为白色
            if(status[i][j]==3) paintnow(i,j,3,false);
        }
    }
    mapl->setPixmap(mappic); //地图数据更新
    dfs_updateandpaint(path,num);
}
void Astar::updateandpaint(){
    Astarnode current=anode[endx][endy];    //从最后一个节点向前回溯输出
    if(current.lastpoint.x()==startx and current.lastpoint.y()==starty){ //判断两点是否相邻
        QMessageBox::information(this,"禁止贴贴","杰哥不要啦~",QMessageBox::Ok);
        clearways();
        return;
    }
    if(bezier==1 and dir!=2) //贝塞尔曲线暂时只支持四方向
    {
        QMessageBox::information(this,"qwq","当前版本贝塞尔曲线只支持四方向哦,等待更新...",QMessageBox::Ok);
        clearways();
        return;
    }
    waypath.clear(); //先清空结果列表 以免受上一次结果影响
    Astarpoints.clear(); //清空路径列表
    controlPoints.clear(); //清空控制点列表

    QPainter painter(&mappic); //绘制最短路径，这里使用QPainter类绘制
    painter.setPen(Qt::black);
    painter.setRenderHint(QPainter::Antialiasing);
    status[startx][starty]=2; //先设置起点状态，以免终点状态异常
    status[endx][endy]=3; //先设置终点状态，以免终点状态异常

    while(1){
        current=anode[current.lastpoint.x()][current.lastpoint.y()];
        if(current.x==startx and current.y==starty) break;  //已经回溯到起点 得到最短路径 结束
        point temp;
        temp.x=current.x;
        temp.y=current.y;
        Astarpoints.prepend(temp); //A星路径列表 便于后面输出贝塞尔曲线
        status[current.x][current.y]=4; //如果不是起点 则当前节点属于最短路径 将状态设为4
        waypath.addRoundedRect(border+(current.y-1)*recta,border+(current.x-1)*recta,square,square,square/4,square/4);  //在最短路径上添加一个带圆角的矩形
    }
    painter.setBrush(QBrush(QColor("#ffe100")));   //绘制最短路径，使用黄色标识
    painter.drawPath(waypath);

    if(!isBgset){ //如果有背景图片就不绘制扩展点
        //绘制扩展点
        painter.setBrush(QBrush(QColor("#c8e0ff"))); //设置画刷颜色#c8e0ff
        for (int i = 1; i <= h; i++) { //绘制扩展点
            for (int j = 1; j <= w; j++) {
                if (status[i][j]==5) { //扩展点状态为5
                    painter.drawRoundedRect(border+(j-1)*recta,border+(i-1)*recta,square,square,square/4,square/4); //绘制扩展点
                }
            }
        }
        //绘制待扩展点
        painter.setBrush(QBrush(QColor("#cdcdda"))); //设置画刷颜色
        status[endx][endy]=3;
        for (int i = 1; i <= h; i++) { //绘制扩展点
            for (int j = 1; j <= w; j++) {
                if (status[i][j]==7) { //待扩展点状态为7
                    painter.drawRoundedRect(border+(j-1)*recta,border+(i-1)*recta,square,square,square/4,square/4); //绘制扩展点
                }
            }
        }
        //绘制LPA*第二次搜索待扩展点
        painter.setBrush(QBrush(QColor("#eed0db"))); //设置画刷颜色
        status[endx][endy]=3;
        for (int i = 1; i <= h; i++) { //绘制扩展点
            for (int j = 1; j <= w; j++) {
                if (status[i][j]==8) { //待扩展点状态为8
                    painter.drawRoundedRect(border+(j-1)*recta,border+(i-1)*recta,square,square,square/4,square/4); //绘制扩展点
                }
            }
        }
    }

    point bstart,bend;
    bstart.x=startx;
    bstart.y=starty;
    bend.x=endx;
    bend.y=endy;
    Astarpoints.prepend(bstart); //头插起点
    Astarpoints.append(bend); //尾插终点
    if(hfunc==10 or hfunc==11 or hfunc==12) //绘制双向A*扩展点
    {
        painter.setBrush(QBrush(QColor("#dedbff"))); //设置画刷颜色
        for (int i = 1; i <= h; i++) {
            for (int j = 1; j <= w; j++) {
                if (status[i][j]==6) { //双向A*扩展点状态为6
                    painter.drawRoundedRect(border+(j-1)*recta,border+(i-1)*recta,square,square,square/4,square/4); //绘制扩展点
                }
            }
        }
    }
    //绘制贝塞尔曲线
    if(bezier==1 && dir==2)
    {
        painter.setPen(QPen(QColor("#ff1414"), 2.5, Qt::SolidLine, Qt::RoundCap));
        int currentx = border + (starty - 1) * recta + square / 2;
        int currenty = border + (startx - 1) * recta + square / 2;
        int i=1;
        while(i < Astarpoints.size()) { //遍历所有的节点
            //qDebug("%d(%d,%d)",i,Astarpoints[i].x,Astarpoints[i].y);
            if(Astarpoints[i].x==endx and Astarpoints[i].y==endy){  //注意：dfs写的时候x和y轴反过来了
                if(controlPoints.size()!=0)
                {
                    //考虑到如果一直都是conner
                    for(int t=1;t<=bezierNum;t++)
                    {
                        double temp=double(t)/bezierNum;
                        double tempx=getBezier(controlPoints,temp,1);
                        double tempy=getBezier(controlPoints,temp,2);
                        //qDebug("%f,%f",tempx,tempy);
                        painter.drawLine(currentx,currenty,tempx,tempy);
                        currentx=tempx;
                        currenty=tempy;
                    }
                    controlPoints.clear();
                }
                int bendx = border + (endy - 1) * recta + square / 2;
                int bendy = border + (endx - 1) * recta + square / 2;
                painter.drawLine(currentx,currenty,bendx,bendy);
                break;
            }

            point p0,p1,p2;
            p0.x = border + (Astarpoints[i-1].y - 1) * recta + square / 2; //计算第i-1个节点的中心点的x坐标
            p0.y = border + (Astarpoints[i-1].x - 1) * recta + square / 2; //计算第i-1个节点的中心点的y坐标
            p1.x = border + (Astarpoints[i].y - 1) * recta + square / 2; //计算第i个节点的中心点的x坐标
            p1.y = border + (Astarpoints[i].x - 1) * recta + square / 2; //计算第i个节点的中心点的y坐标
            p2.x = border + (Astarpoints[i + 1].y - 1) * recta + square / 2; //计算第i+1个节点的中心点的x坐标
            p2.y = border + (Astarpoints[i + 1].x - 1) * recta + square / 2; //计算第i+1个节点的中心点的y坐标

            if((p0.x==p1.x && p1.x==p2.x) || (p0.y==p1.y && p1.y==p2.y))
            {
                if(controlPoints.size()==0)
                {
                    i++;
                    continue;
                }
                else
                {
                    //利用控制点绘制贝塞尔曲线
                    for(int t=1;t<=bezierNum;t++)
                    {
                        double temp=double(t)/bezierNum;
                        double tempx=getBezier(controlPoints,temp,1);
                        double tempy=getBezier(controlPoints,temp,2);
                        //qDebug("%f,%f",tempx,tempy);
                        painter.drawLine(currentx,currenty,tempx,tempy);
                        currentx=tempx;
                        currenty=tempy;
                    }
                    //绘制结束清空控制点列表
                    controlPoints.clear();
                    i++;
                    continue;
                }
            }
            else
            {
                point c1,c2;
                c1.x=(p0.x+p1.x)/2;
                c1.y=(p0.y+p1.y)/2;
                c2.x=(p1.x+p2.x)/2;
                c2.y=(p1.y+p2.y)/2;

                if(controlPoints.size()==0) //控制点列表为空
                {
                    painter.drawLine(currentx,currenty,p0.x,p0.y); //遇到conner首先连接到x0
                    currentx=p0.x;
                    currenty=p0.y;

                    //添加四个控制点
                    controlPoints.append(p0);
                    controlPoints.append(c1);
                    controlPoints.append(c2);
                    controlPoints.append(p2);
                }
                else
                {
                    controlPoints.removeLast(); //删除尾节点
                    //添加两个控制点
                    controlPoints.append(c2);
                    controlPoints.append(p2);
                }
            }
            i++;
        }
    }
    painter.end();
    mapl->setPixmap(mappic);
}
void Astar::putopenlist(QList<Astarnode> &list, Astarnode data,int isDA){    //将data节点加入开放列表
    if(list.isEmpty()){
        list.append(data);
        return;
    }
    else{
        for(int i=0; i<list.size(); i++){   //需要保证openlist的升序性
            if(isDA==1 and list[i].cost>data.cost){
                list.insert(i,data);
                return;
            }
            if(isDA==2 and list[i].costDA>data.costDA)
            {
                list.insert(i,data);
                return;
            }
        }
        list.append(data);  //如果没有找到，就插到队列末尾
    }
}
void Astar::putLPAopenlist(QList<Astarnode> &list, Astarnode data){    //将data节点加入开放列表
    if(list.isEmpty()){
        list.append(data);
        return;
    }
    else{
        for(int i=0;i<list.size();i++){
            if(list[i].k1>data.k1){ //先判断k1
                list.insert(i,data);
                return;
            }
            if(list[i].k1==data.k1){
                if(list[i].k2>data.k2){ //再判断k2
                    list.insert(i,data);
                    return;
                }
                else{
                    list.insert(i+1,data);
                    return;
                }
            }
        }
        list.append(data);  //如果没有找到，就插到队列末尾
    }
}
Astarnode Astar::getopenlist(QList<Astarnode> &list,int isDA){   //首节点出队
    Astarnode a=list[0];
    if(isDA==1) anode[a.x][a.y].isInOpenList=false;
    else if(isDA==2) anode[a.x][a.y].isInDAOpenList=false;
    list.removeFirst();
    return a;
}
Astarnode Astar::getLPAopenlist(QList<Astarnode> &list){   //首节点出队
    Astarnode a=list[0];
    anode[a.x][a.y].isInOpenList=false;
    list.removeFirst();
    return a;
}
void Astar::resortopenlist(QList<Astarnode> &list, Astarnode data,int isDA){ //替换原来节点，重排并加入data节点
    for(int i=0; i<list.size(); i++){
        if(list[i].x==data.x and list[i].y==data.y){
            list.removeAt(i);
        }
    }
    putopenlist(list,data,isDA);
}
void Astar::clearways(){    //清除当前路径
    if(isdfssolved){
        for (int i = 1; i <= h; i++) { //遍历所有的节点
            for (int j = 1; j <= w; j++) {
                if (anode[i][j].pathflag==1) {
                    anode[i][j].pathflag=0;
                    paintnow(i,j,0,false);  //将对应的单元绘制为白色
                }
                if(status[i][j]==2) paintnow(i,j,2,false);  //将对应的单元绘制为白色
                if(status[i][j]==3) paintnow(i,j,3,false);
            }
        }
        isdfssolved=false;
        mapl->setPixmap(mappic);    //地图数据更新
    }
    if(!issolved) return;

    Astarnode current=anode[endx][endy];
    while(1){
        current=anode[current.lastpoint.x()][current.lastpoint.y()];
        if(current.x==startx and current.y==starty) break;
        if(status[current.x][current.y]==1) continue; //遇到路径上被修改为障碍的节点 跳过
        status[current.x][current.y]=0;
        paintnow(current.x,current.y,0,false);  //将对应的单元绘制为白色
    }
    status[startx][starty]=2;
    status[endx][endy]=3;
    for (int i = 1; i <= h; i++) { //遍历所有的节点
        for (int j = 1; j <= w; j++) {
            if (status[i][j]==5 or anode[i][j].pathflag==1 or status[i][j]==6 or status[i][j]==7 or status[i][j]==8) { //清除扩展点
                if((isDMapChanged or isLPAMapChanged) and (status[i][j]==5 or status[i][j]==6)) continue;
                status[i][j]=0;
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);  //将对应的单元绘制为白色
            }
            if(status[i][j]==2) paintnow(i,j,2,false); //重绘起点
            if(status[i][j]==3) paintnow(i,j,3,false); //重绘终点
        }
    }
    mapl->setPixmap(mappic);    //地图数据更新
    issolved=false;
    shutevent=false;
    isLPAsolved=false;
    //isLPArunning=false;
    openlist.clear();
}
void Astar::notfound(){
    QMessageBox::information(this,"qwq","无法找到通路!",QMessageBox::Ok);
}
void Astar::setdir(int a){
    dir=a;
}
void Astar::sethfunc(int a){
    hfunc=a;
}
void Astar::setAnalykey(bool isAnaly){
    isAnalysis=isAnaly;
}
void Astar::setIsbgset(bool isbg){
    isBgset=isbg;
}
void Astar::setbezier(int a){
    bezier=a;
}
void Astar::setpenalty(int a){
    penal=a;
}
void Astar::setmode(int a){
    mode = a;
}
void Astar::setfactor(int a){
    factor=a;
}
//保存地图方法
void Astar::savemap(QString dir){
    QFile mapf(dir);
    if(!mapf.open(QIODevice::WriteOnly)){
        QMessageBox::warning(this,"qwq","保存失败",QMessageBox::Ok);
        return;
    }

    MapData mapdata;
    mapdata.h=h;
    mapdata.w=w;

    for(int i=1; i<=h; i++){
        for(int j=1; j<=w; j++){
            if(status[i][j]==4) mapdata.sta[i][j]=0;
            else mapdata.sta[i][j]=status[i][j];
        }
    }
    mapdata.rsize=square;

    QDataStream w(&mapf);
    char temp[sizeof(MapData)];
    memcpy(temp,&mapdata,sizeof(MapData));
    w.writeBytes(temp,sizeof(MapData)); //QDataStream类内置函数，序列化地图信息

    mapf.close();
}
//载入地图方法
void Astar::loadmap(QString dir){
    qDebug()<<dir;
    QFile mapf(dir);
    MapData mapdata;
    if(!mapf.open(QIODevice::ReadOnly)){
        QMessageBox::warning(this,"qwq","打开失败",QMessageBox::Ok);
        return;
    }

    QDataStream r(&mapf);
    uint dataSize = sizeof(MapData);
    char* temp = new char[dataSize];
    r.readBytes(temp,dataSize);

    memcpy(&mapdata,temp,dataSize);
    mapf.close();
    delete [] temp;

    w=mapdata.w;
    h=mapdata.h;

    for(int i=1; i<=h; i++){
        for(int j=1; j<=w; j++){
            status[i][j]=mapdata.sta[i][j];
        }
    }
    issolved=false;
    setRect(mapdata.rsize);
    //setMapPath();
}
