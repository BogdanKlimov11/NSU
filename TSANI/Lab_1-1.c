#include <stdio.h>
#include <math.h>

void CalculateRoots(float A, float B, float C, float *pD, float *pValue1, float *pValue2) {
    *pD = B * B - 4 * A * C;
    if (*pD < 0) {
        *pValue1 = -B / (2 * A);
        *pValue2 = sqrt(-*pD) / (2 * A);
    } else {
        *pValue1 = (-B - sqrt(*pD)) / (2 * A);
        *pValue2 = (-B + sqrt(*pD)) / (2 * A);
    }
}

void Parabola(float A, float B, float *sammit) {
    *sammit = -B / (2 * A);
}

int main(int argc, char *argv[]) {
    float A, B, C, D, x1, x2, vertex;
    char input[256];

    while (1) {
        printf("Enter coefficients A, B, C (or 'q' to quit): ");
        if (fgets(input, sizeof(input), stdin) == NULL) break;
        if (input[0] == 'q' && input[1] == '\n') break;

        if (sscanf(input, "%f %f %f", &A, &B, &C) != 3) {
            printf("Invalid input. Please enter three numbers.\n\n");
            continue;
        }

        if (A == 0) {
            if (B == 0) {
                printf("Not an equation.\n");
            } else {
                printf("Linear equation: x = %g\n", -C / B);
            }
        } else {
            CalculateRoots(A, B, C, &D, &x1, &x2);
            if (D < 0) {
                printf("Complex roots: x1 = %g + i%g, x2 = %g - i%g\n", x1, x2, x1, x2);
            } else if (D == 0) {
                printf("Single root: x = %g\n", x1);
            } else {
                printf("Roots: x1 = %g, x2 = %g\n", x1, x2);
            }
            Parabola(A, B, &vertex);
            printf("Vertex of parabola: x = %g\n", vertex);
        }
        printf("\n");
    }
    return 0;
}
