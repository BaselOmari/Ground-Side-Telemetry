#include "serialclass.h"
#include <QtSerialPort/QSerialPort>


serialclass::serialclass(QString portname, int baudrate, QSerialPort::StopBits stopbits,
                         QSerialPort::FlowControl flowcontrol, QSerialPort::DataBits databits)
{

    serial = new QSerialPort(this);


    QIODevice::connect(serial, &QSerialPort::readyRead, this, &serialclass::handleSerialRead);
    QIODevice::connect(serial, &QSerialPort::bytesWritten, this, &serialclass::handleSerialWrite);





    serial -> setBaudRate(baudrate);
    serial -> setPortName(portname);
    serial -> setStopBits(stopbits);
    serial -> setFlowControl(flowcontrol);
    serial -> setDataBits(databits);
    serial -> open(QIODevice::ReadWrite);

}

void serialclass::handleSerialRead(){

    serial -> open(QIODevice::ReadWrite);
    QByteArray byteArray = serial->readAll();
    qDebug("Read " + byteArray);

    serialdata.append(serial -> readAll());

}




void serialclass::write(QByteArray data){

    serial -> open(QIODevice::ReadWrite);
    qint64 bytes = serial -> write(data);
    if(bytes == -1) {
        qDebug("There was an error");
    }
    else {
        qDebug(QString::number(bytes).toStdString().c_str());
    }
    serial -> close();

}



void serialclass::handleSerialWrite(qint64 bytes) {
    //confirm that a message was sent
    qDebug("Message was Sent");
}




