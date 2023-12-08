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

Astar::Astar(const QString &text, QWidget *parent,int width,int height,int rectaa) : MapLabel(text,parent,width,height,rectaa){
    //qDebug()<<w<<h;
    issolved=false;
    isdfssolved=false;
    dir=1;
    hfunc=1;
    dynamic=1;
    isnormal=0;
}
//主要函数
void Astar::runAstar(){
    if(issolved){
        QMessageBox::information(this,"Caution","请先清除当前路径",QMessageBox::Ok);
        return;
    }

    //先清除扩展点列表
    openlist.clear();

    if(start->isNull() or end->isNull()){
        QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
        return;
    }

    //记录起点、终点的坐标信息
    startx=start->x();
    starty=start->y();
    endx=end->x();
    endy=end->y();

    //遍历阵列中的节点，初始化节点信息，此循环主要是计算h的值
    for(int i=1; i<=h; i++){
        for(int j=1; j<=w; j++){
            //首先初始化每一个节点对应的实际代价g和blocks
            anode[i][j].g=0;
            anode[i][j].blocks=0;
            anode[i][j].dfs=0;
            anode[i][j].pathflag=0;

            switch(hfunc){  //选择启发函数
            case 1: //对角距离
                anode[i][j].h=abs(abs(i-endx)-abs(j-endy))*10+(abs(i-endx)>abs(j-endy)?abs(j-endy)*14:abs(i-endx)*14)-14;
                break;
            case 2: //曼哈顿距离
                anode[i][j].h=(abs(i-endx)+abs(j-endy))*10;
                break;
            case 3: //欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            case 7: //欧式距离
                anode[i][j].h=sqrt(((int)pow(i-endx,2)+(int)pow(j-endy,2)))*10;
                break;
            }

            //anode[i][j].cost=anode[i][j].h+anode[i][j].g;
            anode[i][j].cost=anode[i][j].h; //因为初始g都为0，所以不需要加，cost等于预估距离即可
            anode[i][j].x=i;
            anode[i][j].y=j;
            anode[i][j].isInOpenList=false;
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
            if(status[i][j]==0){    //表示可以正常通行
                anode[i][j].isClosed=false;
                anode[i][j].visited=false;  //初始化访问情况
            }
            if(status[i][j]==1 or status[i][j]==2){
                anode[i][j].isClosed=true;
                anode[i][j].visited=true;
                anode[i][j].dfs=1;
            }
        }
    }
    straight=sqrt(((int)pow(startx-endx,2)+(int)pow(starty-endy,2)))*10;

    //现已初始化节点信息列表，开始获取最短路径
    Astarnode current;  //表示当前节点
    int optrx;
    int optry;
    current=anode[startx][starty];  //从起点开始

    if(hfunc == 4){  //dfs
        for(int i=0;i<path.size();i++){  //先清除列表数据
            path.removeAt(i);
            qDebug("%lld",path.size());
        }
        if(path.size()>0){
            path.removeAt(0);
        }
        QList<Astarnode> p;
        dfs(startx, starty, p);
        if(!isdfssolved) dfs_updateandpaint(path,0);
        isdfssolved=true;
    }

    if(hfunc == 5){ //bfs模式
        QElapsedTimer time;
        time.start();
        QQueue<Astarnode> q; //定义一个队列
        bool visited[h+1][w+1];
        memset(visited, false, sizeof(visited));
        q.enqueue(anode[startx][starty]);
        visited[startx][starty] = true;
        while (!q.empty()) {
            current = q.front();
            q.dequeue();
            if (current.x == endx && current.y == endy) break;
            for (int i = 1; i <= 4; i++) {
                optrx = current.x + ((i + 1) % 2) * (i - 3);
                optry = current.y + (i % 2) * (i - 2);
                if (optrx <= 0 or optrx > h or optry <= 0 or optry > w) continue;
                if (anode[optrx][optry].isClosed) continue;
                if (!visited[optrx][optry]) {
                    q.enqueue(anode[optrx][optry]);
                    visited[optrx][optry] = true;
                    status[optrx][optry]=5;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                }
            }
        }
        //qDebug()<<"运行时间："<<time.nsecsElapsed()<<"ns";
        issolved=true;
        shutevent=true;
        updateandpaint();
        QMessageBox bfs_time;
        int bfs=time.nsecsElapsed();
        //bfs_time.setIcon(QMessageBox::information);
        bfs_time.setWindowTitle("广度优先运行时间");
        bfs_time.setText(QString("用时%1ns").arg(bfs));
        bfs_time.setStandardButtons(QMessageBox::Ok);
        bfs_time.exec();
    }

    if(hfunc==1 || hfunc==2 || hfunc==3 || hfunc==7){   //A*算法
        QElapsedTimer time;
        time.start();
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
                        if(isnormal==0){
                            temp=double(anode[optrx][optry].h)/double(straight);
                            if(temp>0.5){
                                dynamic=2;
                            }
                            else{
                                dynamic=0.5+temp;
                            }
                        }
                        anode[optrx][optry].cost=anode[optrx][optry].g+dynamic*anode[optrx][optry].h;
                        qDebug("%f",dynamic);
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        putopenlist(openlist,anode[optrx][optry]);
                        anode[optrx][optry].isInOpenList=true;
                        status[optrx][optry]=5;
                    }
                    else if(anode[optrx][optry].g!=0 and anode[optrx][optry].g>current.g+14 and anode[optrx][optry].isInOpenList){
                        anode[optrx][optry].g=current.g+14;
                        if(isnormal==0){
                            temp=double(anode[optrx][optry].h)/double(straight);
                            if(temp>0.5){
                                dynamic=2;
                            }
                            else{
                                dynamic=0.5+temp;
                            }
                        }
                        anode[optrx][optry].cost=anode[optrx][optry].g+dynamic*anode[optrx][optry].h;  //f=g+dynamic*h;
                        anode[optrx][optry].lastpoint.setX(current.x);
                        anode[optrx][optry].lastpoint.setY(current.y);
                        resortopenlist(openlist,anode[optrx][optry]);
                        //status[optrx][optry]=5;
                    }
                }
            }
            for(int i=1; i<=4; i++){    //四方向，i=1，2，3，4分别代表上右下左
                //optrx，optry表示相邻节点坐标
                optrx=current.x+((i+1)%2)*(i-3);    //如果i是奇数，在x方向移动，y方向上不变
                optry=current.y+(i%2)*(i-2);        //如果i是偶数，在y方向移动，x方向上不变
                if(optrx<=0 or optrx>h or optry<=0 or optry>w) continue;    //判断是否到达边界
                if(anode[optrx][optry].isClosed) continue;  //如果相邻节点已在关闭列表中，跳过，关闭列表存放就是访问过的节点
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
                if(anode[optrx][optry].g==0){
                    anode[optrx][optry].g=current.g+10; //一条边的距离设为10
                    if(isnormal==0){
                        temp=double(anode[optrx][optry].h)/double(straight);
                        if(temp>0.5){
                            dynamic=2;
                        }
                        else{
                            dynamic=0.5+temp;
                        }
                    }
                    anode[optrx][optry].cost=anode[optrx][optry].g+alpha_third*dynamic*anode[optrx][optry].h-penalty+alpha;   //f=g+h
                    anode[optrx][optry].lastpoint.setX(current.x);  //并设置该相邻节点的前驱，因为之后输出最短路径通过回溯输出
                    anode[optrx][optry].lastpoint.setY(current.y);
                    putopenlist(openlist,anode[optrx][optry]);  //将该相邻节点放入开放列表，开放列表存放未访问过的节点
                    anode[optrx][optry].isInOpenList=true;
                    status[optrx][optry]=5;
                }
                //如果当前节点的花费比相邻节点少，可以继续在当前节点的相邻节点查找
                if(anode[optrx][optry].g!=0 and anode[optrx][optry].g>current.g+10 and anode[optrx][optry].isInOpenList){
                    anode[optrx][optry].g=current.g+10;
                    if(isnormal==0){
                        temp=double(anode[optrx][optry].h)/double(straight);
                        if(temp>0.5){
                            dynamic=2;
                        }
                        else{
                            dynamic=0.5+temp;
                        }
                    }
                    anode[optrx][optry].cost=anode[optrx][optry].g+alpha_third*dynamic*anode[optrx][optry].h-penalty+alpha;
                    anode[optrx][optry].lastpoint.setX(current.x);
                    anode[optrx][optry].lastpoint.setY(current.y);
                    resortopenlist(openlist,anode[optrx][optry]);   //将该相邻节点放入openlist并重排序openlist
                    //status[optrx][optry]=5;
                }
            }
            if(openlist.isEmpty()){ //如果openlist为空，说明没有可行通路
                notfound();
                //qDebug()<<"not found";
                return;
            }
            if(anode[endx][endy].isInOpenList) break;   //终点已被访问，说明已找到最短路径
            current=getopenlist(openlist);  //否则继续寻找，从openlist首节点开始，openlist是升序的
        }
        //前面已确保openlist不为空，因此至此问题必定解决
        issolved=true;
        shutevent=true;
        updateandpaint();   //最终输出阵列的函数
        int astar=time.nsecsElapsed();
        QMessageBox astar_time;
        astar_time.setWindowTitle("A*运行时间");
        astar_time.setText(QString("用时%1ns").arg(astar));
        astar_time.setStandardButtons(QMessageBox::Ok);
        astar_time.exec();
    }
}

void Astar::dfs(int x ,int y ,QList<Astarnode>& p) {
    if (x == endx && y == endy) {
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
            dfs(nx ,ny ,p);
            anode[nx][ny].dfs = 0;
            p.pop_back();
        }
    }
}

void Astar::dfs_updateandpaint(QList<QList<Astarnode>> path, int index){
    //qDebug("%lld",path.size());
    if (path.isEmpty()) {
        notfound();
        return;
    }
    QList<Astarnode> first_path = path[index]; //定义一个变量，用于存储path列表中的第一条路径

    QPainter painter(&mappic);
    painter.setPen(Qt::black);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setBrush(QBrush(QColor("#ffe100"))); //设置画刷颜色

    //qDebug("path[0]的长度：%lld",first_path.size());
    for (int i = 0; i < first_path.size(); i++) { //遍历所有的节点
        if(first_path[i].x==startx and first_path[i].y==starty){
            continue;
        }
        if(first_path[i].x==endx and first_path[i].y==endy){
            continue;
        }
        painter.drawRoundedRect(border+(first_path[i].y-1)*recta,border+(first_path[i].x-1)*recta,square,square,square/4,square/4); //在该节点上绘制一个矩形
        anode[first_path[i].x][first_path[i].y].pathflag=1;
        //qDebug("第一条：%d,%d",first_path[i].x,first_path[i].y);
    }

    painter.end();
    mapl->setPixmap(mappic);
}

void Astar::updateandpaint(){
    Astarnode current=anode[endx][endy];    //从最后一个节点向前回溯输出

    //判断两点是否相邻
    if(current.lastpoint.x()==startx and current.lastpoint.y()==starty){
        QMessageBox::information(this,"禁止贴贴","杰哥不要啦~",QMessageBox::Ok);
        clearways();
        return;
    }

    waypath.clear();    //先清空结果列表，以免受上一次结果影响

    while(1){
        current=anode[current.lastpoint.x()][current.lastpoint.y()];
        if(current.x==startx and current.y==starty) break;  //如果已经回溯到起点，得到最短路径，结束
        status[current.x][current.y]=4; //如果不是起点，说明当前节点属于最短路径，将当前节点状态设为4
        waypath.addRoundedRect(border+(current.y-1)*recta,border+(current.x-1)*recta,square,square,square/4,square/4);  //在最短路径上添加一个带圆角的矩形
    }
    //setMapPath();
    //绘制最短路径，这里使用QPainter类绘制
    QPainter painter(&mappic);
    painter.setPen(Qt::black);
    painter.setRenderHint(QPainter::Antialiasing);

    painter.setBrush(QBrush(QColor("#ffe100")));   //最短路径使用黄色标识
    painter.drawPath(waypath);

    //添加以下代码，用于绘制关闭列表中的节点
    painter.setBrush(QBrush(QColor("#ccbfb3"))); //设置画刷颜色
    status[endx][endy]=3;
    for (int i = 1; i <= h; i++) { //遍历所有的节点
        for (int j = 1; j <= w; j++) {
            if (status[i][j]==5) { //
                painter.drawRoundedRect(border+(j-1)*recta,border+(i-1)*recta,square,square,square/4,square/4); //在该节点上绘制一个矩形
            }
        }
    }

    painter.end();
    mapl->setPixmap(mappic);
}

void Astar::putopenlist(QList<Astarnode> &list, Astarnode data){    //将data节点加入开放列表
    if(list.isEmpty()){
        list.append(data);
        return;
    }
    else{
        for(int i=0; i<list.size(); i++){   //需要保证openlist的升序性
            if(list[i].cost>data.cost){
                list.insert(i,data);
                return;
            }
        }
        list.append(data);  //如果没有找到，就插到队列末尾
    }
}

Astarnode Astar::getopenlist(QList<Astarnode> &list){   //首节点出队
    Astarnode a=list[0];
    anode[a.x][a.y].isInOpenList=false;
    list.removeFirst();
    return a;
}
void Astar::resortopenlist(QList<Astarnode> &list, Astarnode data){ //替换原来节点，重排并加入data节点
    for(int i=0; i<list.size(); i++){
        if(list[i].x==data.x and list[i].y==data.y){
            list.removeAt(i);
        }
    }
    putopenlist(list,data);
}
void Astar::clearways(){    //清除当前路径
    if(isdfssolved){
        for (int i = 1; i <= h; i++) { //遍历所有的节点
            for (int j = 1; j <= w; j++) {
                if (anode[i][j].pathflag==1) {
                    anode[i][j].pathflag=0;
                    paintnow(i,j,0,false);  //将对应的单元绘制为白色
                }
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
        status[current.x][current.y]=0;
        paintnow(current.x,current.y,0,false);  //将对应的单元绘制为白色
    }
    status[startx][starty]=2;
    status[endx][endy]=3;
    for (int i = 1; i <= h; i++) { //遍历所有的节点
        for (int j = 1; j <= w; j++) {
            //5为扩展过的点，6为dfs路径上的点
            if (status[i][j]==5 || anode[i][j].pathflag==1) {
                status[i][j]=0;
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);  //将对应的单元绘制为白色
            }
        }
    }

    mapl->setPixmap(mappic);    //地图数据更新

    issolved=false;
    shutevent=false;
    openlist.clear();
}
void Astar::nextpath(){  //下一条路径
    qDebug()<<"nextpath";
    if(start->isNull() or end->isNull()){
        QMessageBox::information(this,"Caution","请先设置起点和终点！",QMessageBox::Ok);
        return;
    }
    //先清除当前路径
    for (int i = 1; i <= h; i++) { //遍历所有的节点
        for (int j = 1; j <= w; j++) {
            //5为扩展过的点，6为dfs路径上的点
            if (anode[i][j].pathflag==1) {
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);  //将对应的单元绘制为白色
            }
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
        QMessageBox::information(this,"提示","路径列表为空！",QMessageBox::Ok);
    }
    qDebug()<<"showmin";
    int min=100;
    int num=0;
    for(int i=0;i<path.size();i++){
        if(path[i].size()<=min){
            min=path[i].size();
            num=i;
        }
    }

    for (int i = 1; i <= h; i++) { //遍历所有的节点
        for (int j = 1; j <= w; j++) {
            if (anode[i][j].pathflag==1) {
                anode[i][j].pathflag=0;
                paintnow(i,j,0,false);  //将对应的单元绘制为白色
            }
        }
    }
    mapl->setPixmap(mappic);    //地图数据更新

    dfs_updateandpaint(path,num);
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
void Astar::setpenalty(int a){
    penal=a;
}
void Astar::setmode(int a){
    mode = a;
}
void Astar::setfactor(int a){
    factor=a;
}
void Astar::setnormal(int a){
    isnormal=a;
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
