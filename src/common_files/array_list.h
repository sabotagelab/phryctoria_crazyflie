#ifndef _ARRAY_LIST_H_
#define _ARRAY_LIST_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"

typedef void(*CallbackPrint)(void *);
typedef int(*CallbackCompare)(void *a, void *b);

typedef struct {
    int len;
    int max_len;
    int data_size;
    void *array;
    CallbackPrint callback_print;
    CallbackCompare callback_compare;
    xSemaphoreHandle sem;
} List;

void list_create(List *list, int limit, int data_size, void *ptr, CallbackPrint print_callback, CallbackCompare compare_callback);

void list_insert(List *list, void *data, int index);
void list_insert_sorted(List *list, void *data);
void list_append(List *list, void *data);

void list_remove(List *list, int index);
void list_take(List *list, void *store);

bool list_isempty(List *list);
void _list_print(List *list);


#endif