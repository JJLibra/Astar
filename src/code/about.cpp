//"关于我们窗口设计"
#include<about.h>
AboutDialog::AboutDialog(QWidget *parent){
    //窗口
    this->setWindowTitle("关于NWPU-星G611-ikuns");
    this->setWindowIcon(QIcon(":/img/ikun.ico"));
    this->setWindowFlags(Qt::WindowCloseButtonHint);
    this->setFixedSize(460,240);

    //小组信息
    QLabel *logo=new QLabel();
    logo->setFixedSize(100,100);
    logo->setPixmap(QPixmap(":/img/ikun.ico").scaled(100,100,Qt::KeepAspectRatio,Qt::SmoothTransformation));
    logo->setAlignment(Qt::AlignCenter);

    //作者信息：ljj
    QLabel *profile=new QLabel();
    profile->setFixedSize(100,100);
    profile->setPixmap(QPixmap(":/img/luckycat.jpg").scaled(100,100,Qt::KeepAspectRatio,Qt::SmoothTransformation));
    profile->setAlignment(Qt::AlignCenter);

    QLabel *text1=new QLabel(" 星G -“ikun之家” 对门 awa");
    text1->setFont(QFont("SimSun",16));
    QLabel *text2=new QLabel("一款基于Qt开发的模拟无人机避障及路径搜索的算法演示器");
    text2->setFont(QFont("SimSun",12));
    text2->setWordWrap(true);
    QLabel *text3=new QLabel("作者:小旋风er");
    text3->setFont(QFont("SimSun",12));


    QLabel *text4=new QLabel();
    QLabel *text5=new QLabel();
    QLabel *text6=new QLabel();

    text4->setOpenExternalLinks(true);
    text4->setText("<a style='color:#66CCFF;' href=\"https://github.com/JJLibra\">Github:@LJJ@NWPU");
    text4->setFont(QFont("SimSun",12));

    text6->setOpenExternalLinks(true);
    text6->setText("<a style='color:#66CCFF;' href=\"https://space.bilibili.com/1327183873?spm_id_from=333.1007.0.0\">Bilibili:Rosssyyyy");
    text6->setFont(QFont("SimSun",12));

    text5->setOpenExternalLinks(true);
    text5->setText("Email:2021303114@mail.nwpu.edu.cn");
    text5->setFont(QFont("SimSun",12));

    QVBoxLayout *layoutPic = new QVBoxLayout;
    layoutPic->setAlignment(Qt::AlignRight);
    layoutPic->addWidget(logo);
    layoutPic->addStretch();
    layoutPic->addWidget(profile);

    QVBoxLayout *layoutText = new QVBoxLayout;

    layoutText->addWidget(text1);
    layoutText->addStretch();
    layoutText->addWidget(text2);
    layoutText->addStretch();
    layoutText->addWidget(text3);
    layoutText->addWidget(text4);
    layoutText->addWidget(text6);
    layoutText->addWidget(text5);

    layoutText->setAlignment(Qt::AlignTop);


    QHBoxLayout *aboutlayout = new QHBoxLayout(this);
    aboutlayout->addLayout(layoutPic);
    aboutlayout->addLayout(layoutText);

}
