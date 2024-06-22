void DCP()
{
    int t, codeACP;
    double val;
    double PercentError, LSBerror, error0, errorMash, errorInte, errorTemp, errorDuff;
    double Xarakt[256];
    double Real[256];
    double Analog[256];

    // разрядность - кол-во выходных уровней, отстающих друг от друга на один элементарный шаг 8 бит = 256
    for (t=0; t<256; t++) { Real[t] = 3.3*t/256; }

    for (t=0; t<256; t++)
    {
        Write(2,2,t);
        analogIn(0, &val);
        Xarakt[t] = val;
    }

    // характеристика преобразования (передаточная функция) - зависимость между сигналом на выходе ЦАПа и поданным на вход кодом
    PlotY (panelHandle, PANEL_GRAPH_1, Xarakt, 256, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);

    // вычисление ошибки нуля
    Write(2,2,0);
    analogIn(0, &val);
    error0=val-0;
    PercentError=error0/3.3*100;
    LSBerror=error0/3.3*265;
    SetCtrlVal(PANEL, PANEL_NUMERIC_0,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_2,  LSBerror);

    // вычисленгие ошибки масштаба
    Write(2,2,255);
    analogIn(0, &val);
    errorMash=3.3-val;
    PercentError=errorMash/3.3*100;
    LSBerror=errorMash/3.3*256;
    SetCtrlVal(PANEL, PANEL_NUMERIC_1,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_3,  LSBerror);

    // вычисленгие интегральной нелин
    errorInte=0;
    for (t=0; t<256; t++)
    {
        errorTemp = Xarakt[t]- Real[t];
        if  (errorTemp<0)  errorTemp=errorTemp*(-1);
        if  (errorTemp>errorInte)  errorInte=errorTemp;
        Analog[t] = Xarakt[t] - Real[t];
    }
    PercentError=errorInte/3.3*100;
    LSBerror=errorInte/3.3*256;
    SetCtrlVal(PANEL, PANEL_NUMERIC_5,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_4,  LSBerror);

    // вычисление дифференциальной нелин
    errorDuff=0;
    for (t=0; t<255; t++)
    {
        errorTemp = (Xarakt[t] - Xarakt[t+1])- 3.3/256;
        if  (errorTemp<0)  errorTemp=errorTemp*(-1);
        if  (errorTemp>errorDuff)  errorDuff=errorTemp;
    }
    PercentError=errorDuff/3.3*100;
    LSBerror=errorDuff/3.3*256;
    SetCtrlVal(PANEL, PANEL_NUMERIC_7,  PercentError);
    SetCtrlVal(PANEL, PANEL_NUMERIC_6,  LSBerror);

    // график аналогивых ошибок
    for (t=0; t<256; t++)
    {
        Analog[t] = Xarakt[t] - Real[t] - error0 - errorMash;
    }
    PlotY (panelHandle, PANEL_GRAPH_2, Analog, 256, VAL_DOUBLE, VAL_FAT_LINE, VAL_SOLID_SQUARE, VAL_DOT, 1, VAL_GREEN);
}