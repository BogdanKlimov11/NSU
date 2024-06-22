#include <utility.h>
#include "tsani.h"
#include "ourBubl.h"

void Initial(int Adres_plat, int Adres_reg)
{
    portMask(0, 0xFF); //  настроили порт на запись
    portMask(1, 0xFF); //  настроили порт на запись
    portMask(2, 0x07); //  настроили порт на запись
    portOut(2, 0x06); // ALE=0 READ=1 WRITE=1 начальное состояние

    int a = (Adres_reg<<3)|Adres_plat; // формирование адреса

    unsigned char a0;
    unsigned char a1;

    a0 = (a&0xFF);
    a1 = (a>>8)&0xFF; // сформированный адресс

    portOut(0, a0);
    portOut(1, a1); // запись адреса на шину

    portOut(2, 0x07); // запись в ALE 1
    portOut(2, 0x06); // запись в ALE 0
}

void Write (int Adres_plat, int Adres_reg, int slovo)
{
    Initial(Adres_plat, Adres_reg);

    portMask(0, 0xFF); // нужно ли если в инициализации это уже делали?
    portMask(1, 0xFF);

    unsigned char w0;
    unsigned char w1;

    w0 = (slovo&0xFF);
    w1 = (slovo>>8)&0xFF;

    portOut(0, w0); // запись на шину
    portOut(1, w1);

    portOut(2, 0x2); // запись в write 0
    portOut(2, 0x6); // запись в write 1
}

int Read (int Adres_plat, int Adres_reg)
{
    Initial(Adres_plat, Adres_reg);

    portOut(2, 0x04); // записать 0 в READ
    portMask(0, 0x00); // настроили порт на чтение
    portMask(1, 0x00); // настроили порт на чтение
    Delay(0.006); // подождать 6 мкс

    unsigned char r0;
    unsigned char r1;

    portIn(0, &r0); // прочитать данные с шины
    portIn(1, &r1); // прочитать данные с шины

    portOut(2, 0x06); // запись в read 1
    portMask(0, 0xFF); // настроили порт на запись
    portMask(1, 0xFF); // перевести шину AD в режим записи

    return (r0 | (r1<<8));
}

void WriteDAC(double U)
{
    int Code;
    Code=256/3.3*U;
    if (Code<0) U=0;
    if (Code>255) U=255;
    Write(2,2,Code);
}

int ACP()
{
    int codeACP;
    unsigned char r=0x00;
    unsigned char s=0x00;

    Write(2, 0x10, 5); // управ.

    //обнуляем
    Write(2, 0x12, 0); // нач.
    Write(2, 0x13, 0); // конеч.
    Write(2, 0x14, 0); // таймер

    Snat();
    Delay(2);
    Write(2, 0x11, 1); // установка битов Start и IACK в регисторе команд

    // здесь опрос регистра команд

    while (s!=0x20)
    {
        portIn(2, &r);
        s = r&0x20;
    }
    Delay(1);
    Snat();
    codeACP=Read(2, 0x16);
    return codeACP;
}

void Prer()
{
    int reg;
    reg = Read(2, 0x11);
    reg = reg|0x4;
    Write(2, 0x11, reg);
}

void Snat()
{
    int reg;
    reg = Read(2, 0x11);
    reg = reg|0x2;
    Write(2, 0x11, reg);
}

int Define_Your_Functions_Here (int x)
{
    return x;
}