#pragma once

/*
  Simple generic List ADS like implementation
  Fabiano Salles
*/

#include <stdbool.h>

typedef void(*CallbackFree)(void *);
typedef int(*CallbackCompare)(void *a, void *b);
typedef void(*CallbackPrint)(void *);

typedef struct _node {
	void *data;
	struct _node *next;
} Node;

typedef struct {
	int count;
	int data_size;
	Node *head;
	Node *tail;
	CallbackFree callback_free;
	CallbackCompare callback_compare;
	CallbackPrint callback_print;
} List;	

/*
  callback to iteration functions.
  the list iteration will break if returns false
*/
typedef bool(*CallbackIterate)(int index, Node *node);

List* list_create(int dataSize, CallbackFree free_callback, CallbackCompare compare_callback, CallbackPrint print_callback);
void list_destroy(List *list);
void list_add(List *list, void *data);
void list_remove(List *list, int index);
bool list_contains(List *list, void *data);
Node* list_find(List *list, void *data);
Node* list_get_node(List *list, int index);
void* list_get_data(List *list, int index);
void list_iterate(List *list, CallbackIterate iterate_callback);

bool list_isempty(List *list);
void list_insert(List *list, void *data, int index);
void list_print(List *list);
