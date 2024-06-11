#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "semphr.h"

#include "array_list.h"

void list_create(List *list, int limit, int data_size, void *ptr, CallbackPrint print_callback, CallbackCompare compare_callback) {
    list->len = 0;
    list->max_len = limit;
    list->data_size = data_size;
    list->array = ptr;
    list->callback_print = print_callback;
    list->callback_compare = compare_callback;
    list->sem = xSemaphoreCreateBinary();
    xSemaphoreGive(list->sem);
}

static void _list_insert(List *list, void *data, int index) {
    assert((list->len+1) < list->max_len);
    assert(index >= 0);
    assert(index <= list->len);
    memmove(list->array+(index+1)*list->data_size, list->array+(index*list->data_size), (list->len - index)*list->data_size);
    memcpy(list->array + (index*list->data_size), data, list->data_size);
    list->len++;
}

static void _list_remove(List *list, int index) {
    // assert(index >= 0);
    // assert(index < list->len);

    if (list->len == 0 || index >= list->len)
        return;

    memcpy(list->array+index*list->data_size, list->array+(index+1)*list->data_size, (list->len-1-index)*list->data_size);
    memset(list->array + (list->len - 1)*list->data_size, 0, list->data_size);
    list->len--;
}

void list_insert(List *list, void *data, int index) {
    xSemaphoreTake(list->sem, portMAX_DELAY);
    _list_insert(list, data, index);
    xSemaphoreGive(list->sem);
}


void list_insert_sorted(List *list, void *data) {
    assert(list->callback_compare != NULL);
    xSemaphoreTake(list->sem, portMAX_DELAY);
    int i = 0;
    for (; i<list->len; i++) {
        if (list->callback_compare(data, list->array+(i*list->data_size)) < 0)
            break;
    }
    _list_insert(list, data, i);
    xSemaphoreGive(list->sem);
}


bool list_isempty(List *list) {
    return list->len == 0;
}

void list_append(List *list, void *data) {
    xSemaphoreTake(list->sem, portMAX_DELAY);
    _list_insert(list, data, list->len);
    xSemaphoreGive(list->sem);
}

void list_remove(List *list, int index) {
    xSemaphoreTake(list->sem, portMAX_DELAY);
    _list_remove(list, index);
    xSemaphoreGive(list->sem);
}

void list_take(List *list, void *store) {
    assert(store != NULL);
    xSemaphoreTake(list->sem, portMAX_DELAY);
    memcpy(store, list->array, list->data_size);
    _list_remove(list, 0);
    xSemaphoreGive(list->sem);
}

void _list_print(List *list) {
    xSemaphoreTake(list->sem, portMAX_DELAY);
    for (int i=0; i<list->len; i++) {
        printf("index: %d, ", i);
        list->callback_print(list->array+(i*list->data_size));
        printf("\n");
    }
    xSemaphoreGive(list->sem);
}