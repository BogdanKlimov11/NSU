#include <stdio.h>
#include <stdlib.h>
#include <math.h>

double find_min(double nums[], int n) {
    double min_val = nums[0];
    for (int i = 1; i < n; i++) {
        if (nums[i] < min_val) {
            min_val = nums[i];
        }
    }
    return min_val;
}

double find_max(double nums[], int n) {
    double max_val = nums[0];
    for (int i = 1; i < n; i++) {
        if (nums[i] > max_val) {
            max_val = nums[i];
        }
    }
    return max_val;
}

double find_mean(double nums[], int n) {
    double sum = 0;
    for (int i = 0; i < n; i++) {
        sum += nums[i];
    }
    return sum / n;
}

double find_median(double nums[], int n) {
    double *nums_sorted = malloc(n * sizeof(double));
    for (int i = 0; i < n; i++) {
        nums_sorted[i] = nums[i];
    }
    // Сортировка массива
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (nums_sorted[i] > nums_sorted[j]) {
                double temp = nums_sorted[i];
                nums_sorted[i] = nums_sorted[j];
                nums_sorted[j] = temp;
            }
        }
    }
    double median;
    if (n % 2 == 1) {
        median = nums_sorted[n / 2];
    } else {
        median = (nums_sorted[n / 2 - 1] + nums_sorted[n / 2]) / 2;
    }
    free(nums_sorted);
    return median;
}

double find_std_deviation(double nums[], int n, double mean) {
    double variance = 0;
    for (int i = 0; i < n; i++) {
        variance += pow(nums[i] - mean, 2);
    }
    return sqrt(variance / n);
}

int find_max_consecutive(double nums[], int n) {
    int max_count = 1;
    int count = 1;
    for (int i = 1; i < n; i++) {
        if (nums[i] == nums[i - 1]) {
            count++;
            if (count > max_count) {
                max_count = count;
            }
        } else {
            count = 1;
        }
    }
    return max_count;
}

int find_max_monotonic(double nums[], int n) {
    int max_length = 1;
    int current_length = 1;

    // Поиск максимальной длины неубывающего участка
    for (int i = 1; i < n; i++) {
        if (nums[i] >= nums[i - 1]) {
            current_length++;
        } else {
            if (current_length > max_length) {
                max_length = current_length;
            }
            current_length = 1;
        }
    }
    if (current_length > max_length) {
        max_length = current_length;
    }

    current_length = 1;
    // Поиск максимальной длины невозрастающего участка
    for (int i = 1; i < n; i++) {
        if (nums[i] <= nums[i - 1]) {
            current_length++;
        } else {
            if (current_length > max_length) {
                max_length = current_length;
            }
            current_length = 1;
        }
    }
    if (current_length > max_length) {
        max_length = current_length;
    }

    return max_length;
}

int main() {
    int n;
    printf("Введите количество чисел: ");
    scanf("%d", &n);

    if (n <= 0) {
        printf("Массив пуст!\n");
        return 0;
    }

    double *nums = malloc(n * sizeof(double));
    printf("Введите числа с плавающей точкой: ");
    for (int i = 0; i < n; i++) {
        scanf("%lf", &nums[i]);
    }

    double min_val = find_min(nums, n);
    double max_val = find_max(nums, n);
    double mean_val = find_mean(nums, n);
    double median_val = find_median(nums, n);
    double std_dev_val = find_std_deviation(nums, n, mean_val);
    int max_consec_val = find_max_consecutive(nums, n);
    int max_monotonic_val = find_max_monotonic(nums, n);

    printf("Минимальное число: %.2lf\n", min_val);
    printf("Максимальное число: %.2lf\n", max_val);
    printf("Среднее арифметическое: %.2lf\n", mean_val);
    printf("Медиана: %.2lf\n", median_val);
    printf("Среднеквадратичное отклонение: %.2lf\n", std_dev_val);
    printf("Максимальное количество подряд одинаковых элементов: %d\n", max_consec_val);
    printf("Максимальная длина монотонного участка: %d\n", max_monotonic_val);

    free(nums);
    return 0;
}
