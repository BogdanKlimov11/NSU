void ACPread()
{
    FILE *DataACP;
    int t = 0;
    int Xarakt[9000];
    double val = - 0.1;
    double PercentError, LSBerror, error0, errorMash, errorInte, errorTemp;

    DataACP = fopen ("Massiv.txt", "w");

    while (val < 2.56)
    {
        analogOut(0, val);
        val = val + (20 / (pow(2, 16) - 1));
        Xarakt[t] = ACP();
        fprintf(DataACP, "%d\n", Xarakt[t]);
        PercentError = 100 * (double) t / 9000;
        SetCtrlVal(PANEL, PANEL_NUMERICGAUGE, PercentError);
        t++;
    }

    fclose (DataACP);
}