#include <utility.h>
#include <ansi_c.h>

float A, B, C, pD, pValue1, pValue2, sammit;

void CalculateRoots(float A, float B, float C, float *pD,  float *pValue1, float *pValue2)
{
    *pD=B*B-4*A*C;
    if (*pD<0)
    {
        *pValue1= -B/2*A;
        *pValue2= sqrt(-*pD)/2*A;
    }
    else
    {
        *pValue1=(-B-sqrt(*pD))/2*A;
        *pValue2=(-B+sqrt(*pD))/2*A;
    }
}
void Parabola(float A, float B,  float *sammit)
{
    *sammit=-B/2*A;
}

static void usage (char *name)
{
    fprintf (stderr, "usage: %s <argument>\n", name);
    fprintf (stderr, "A short summary of the functionality.\n");
    fprintf (stderr, "    <argument>    is an argument\n");
    exit (1);
}

int main (int argc, char *argv[])
{

//	  double test=-1./0.;
//	  printf("%lf\n", test);

    int exit=0;

    while (1)
    {
        printf("Enter the coefficients A, B, C of the quadric equation:");
        exit=getchar();
        if (exit=='q') {break;}
        scanf("%f %f %f", &A, &B, &C);
        if (A==0)
        {
            if (B==0) {printf("This is not an equation");}
            else {printf("This is linear function: x=%f", -C/B); }
        }
        else
        {
            CalculateRoots(A, B, C, &pD, &pValue1, &pValue2);
            if (pD<0)
            {printf("x1=%f+i%f \nx2=%f-i%f ", pValue1, pValue2, pValue1, pValue2);}
            else
            {
                printf("x1=%f, x2=%f", pValue1, pValue2);
                Parabola(A, B, &sammit);
                printf("\nThe vertex of a parabola: %f", sammit);
            }
        }
        printf("\n\n");
    }
}