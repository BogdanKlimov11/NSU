#include <utility.h>
#include <ansi_c.h>

#include "quadratic-solver.h"

#define EXIT_KEY_Q 0x0051
#define EXIT_KEY_UPPER_Q 0x0071

int calculateRoots(float coefficientA, float coefficientB, float coefficientC, float *discriminantPtr, float *root1Ptr, float *root2Ptr)
{
    double a = coefficientA, b = coefficientB, c = coefficientC, discriminant = 0;
    int result;

    if (a == 0)
    {
        if (b == 0)
        {
            discriminant = 0;
            if (c != 0)
            {
                result = 5;
            }
            else
            {
                result = 6;
            }
        }
        else
        {
            discriminant = b * b;
            *root1Ptr = (float)(-c / b);
            result = 4;
        }
    }
    else
    {
        discriminant = b * b - 4 * a * c;
        if (discriminant > 0)
        {
            *root1Ptr = (float)((-b - sqrt(discriminant)) / (2 * a));
            *root2Ptr = (float)((-b + sqrt(discriminant)) / (2 * a));
            result = 2;
        }
        else if (discriminant == 0)
        {
            *root1Ptr = (float)(-b / (2 * a));
            result = 1;
        }
        else
        {
            *root1Ptr = (float)(-b / (2 * a));
            *root2Ptr = (float)(sqrt(-discriminant) / (2 * a));
            result = 3;
        }
    }
    *discriminantPtr = (float)discriminant;
    return result;
}

int main()
{
    float coefficientA, coefficientB, coefficientC, discriminant, root1, root2;
    int inputKey = 0, result;

    while ((inputKey != EXIT_KEY_Q) && (inputKey != EXIT_KEY_UPPER_Q))
    {
        printf("Enter A B C:\n");
        scanf("%f %f %f", &coefficientA, &coefficientB, &coefficientC);
        result = calculateRoots(coefficientA, coefficientB, coefficientC, &discriminant, &root1, &root2);
        switch (result)
        {
            case 1:
                printf("1 root:\nX = %f\n", root1);
                break;

            case 2:
                printf("2 real roots:\nX1 = %f\nX2 = %f\n", root1, root2);
                printf("Parabola's Vertex:\nX = %f\nY = %f\n", (-coefficientB / (2 * coefficientA)), ((4 * coefficientA * coefficientC - coefficientB * coefficientB) / (4 * coefficientA)));
                break;

            case 3:
                printf("2 complex roots:\nX1 = %f + i*%f\nX2 = %f - i*%f\n", root1, root2, root1, root2);
                break;

            case 4:
                printf("A linear equeation, 1 root:\nX = %f\n", root1);
                break;

            case 5:
                printf("No roots.\n");
                break;

            case 6:
                printf("Infinite number of roots.\n");
                break;

            default:
                printf("An error occured.");
        }
        printf("\nPress q or Q to quit.\n");
        inputKey = GetKey();
    }
    return 0;
}