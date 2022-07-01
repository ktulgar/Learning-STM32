#include "widget.h"
#include "ui_widget.h"

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    setFocusPolicy(Qt::StrongFocus);
    serialPort.setPortName("COM5");
    serialPort.setBaudRate(115200);
    serialPort.setDataBits(QSerialPort::Data8);
    serialPort.setStopBits(QSerialPort::OneStop);
    serialPort.setFlowControl(QSerialPort::NoFlowControl);
    serialPort.setParity(QSerialPort::NoParity);
    serialPort.open(QSerialPort::WriteOnly);
}

Widget::~Widget()
{
    delete ui;
}

void Widget::keyPressEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Left) {
        ui->pushButton->setStyleSheet("background-color : red");
        serialPort.write("Turn Left");
        while(!serialPort.waitForBytesWritten());
    }
    else if(event->key() == Qt::Key_Right) {
        ui->pushButton_2->setStyleSheet("background-color : red");
        serialPort.write("Turn Right");
        while(!serialPort.waitForBytesWritten());
    }
}

void Widget::keyReleaseEvent(QKeyEvent *event)
{
    if(event->key() == Qt::Key_Left) {
        ui->pushButton->setStyleSheet("");
    }
    else if(event->key() == Qt::Key_Right) {
        ui->pushButton_2->setStyleSheet("");
    }
}

