//设置阵列行列数的窗口
#include <setxyDialog.h>
SetxyDialog::SetxyDialog(QWidget *parent, int xx,int yy):QDialog(parent){
    this->setWindowTitle("设置长宽");
    //this->setFixedSize(200,200);
    spx=new QSpinBox(this);
    spx->setMaximum(100);
    spx->setMinimum(5);
    spx->setValue(xx);
    spy=new QSpinBox(this);
    spy->setMaximum(100);
    spy->setMinimum(5);
    spy->setValue(yy);
    setbtn=new QPushButton("确定",this);

    xtext=new QLabel(this);
    xtext->setText("设定长度(5-100)");

    ytext=new QLabel(this);
    ytext->setText("设定高度(5-100)");

    box=new QGridLayout;

    box->addWidget(xtext,0,0,1,4);
    box->addWidget(spx,0,5,1,4);
    box->addWidget(ytext,3,0,1,4);
    box->addWidget(spy,3,5,1,4);
    box->addWidget(setbtn,5,5,1,4);

    box->setHorizontalSpacing(20);
    box->setVerticalSpacing(20);
    box->setContentsMargins(20,20,20,20);


    this->setLayout(box);

    setWindowFlags(Qt::Dialog|Qt::WindowCloseButtonHint);

    connect(setbtn,SIGNAL(clicked(bool)),this,SLOT(setxy()));
}
void SetxyDialog::setxy(){
    x=spx->value();
    y=spy->value();
    this->accept();
    return;
}


