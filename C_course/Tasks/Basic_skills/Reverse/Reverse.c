#include "Reverse.h"

/* Оборачивание массива */
void reverse(int arr[], size_t count)
{
    int start = 0;
    int end = count - 1;
    
    while (start < end)
    {
        // Обмен значений между start и end
        int temp = arr[start];
        arr[start] = arr[end];
        arr[end] = temp;
        
        // Переход к следующим элементам
        start++;
        end--;
    }
}