#include "generic_list.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "FreeRTOS.h"
#include "debug.h"

static void free_node_data(CallbackFree free_callback, void *data) {
	if (free_callback != NULL)
		free_callback(data);
	else
		free(data);
}

List *list_create(int dataSize, CallbackFree free_callback, CallbackCompare compare_callback, CallbackPrint print_callback) {
	assert(dataSize > 0);	

	List *list = (List *)malloc(sizeof(List));
	list->count = 0;
	list->data_size = dataSize;
	list->head = NULL;
	list->tail = NULL;
	list->callback_free = free_callback;
	list->callback_compare = compare_callback;
	list->callback_print = print_callback;
	return list;
}

void list_destroy(List *list){
	assert(list != NULL);
	
	if (list->head != NULL) {
		Node *node = list->head;
		while (node != NULL){
			list->head = node->next;
			free_node_data(list->callback_free, node->data);
			free(node);
			list->count--;
			node = list->head;
		}
	}
	free(list);
}

bool list_isempty(List *list) {
	return list->count == 0;
}

void list_add(List *list, void *data) {
	assert(list != NULL);
	assert(data != NULL);
	
	Node *newNode = calloc(1, sizeof(Node));
	newNode->data = malloc(list->data_size);
	memcpy(newNode->data, data, list->data_size);

	// list->callback_print(newNode->data);

	if (list->head == NULL) {
		list->head = newNode;
		list->tail = newNode;
	}
	else {
		list->tail->next = newNode;
		list->tail = newNode;
	}
	list->count++;
}

void list_insert(List *list, void *data, int index) {
	assert(list != NULL);
	assert(data != NULL);
	assert(index <= list->count);

	Node *newNode = calloc(1, sizeof(Node));
	newNode->data = malloc(list->data_size);
	memcpy(newNode->data, data, list->data_size);	

	if (list->head == NULL || index == list->count) {
		list_add(list, data);
	}
	else if (index == 0) {
		newNode->next = list->head;
		list->head = newNode;
		list->count++;
	}
	else {
		int i = 0;
		Node *cur = list->head;
		while (cur->next != NULL) {
			if (index == i) {
				newNode->next = cur->next;
				cur->next = newNode;
				break;
			}
			i++;
			cur = cur->next;
		}
		list->count++;
	}
}

void list_remove(List *list, int index) {
	assert(list != NULL);
	if (list_isempty(list))
		return;

	// assert(index >= 0);
	// assert(index <= list->count-1);
	
	int i = 0;
	Node *prev = (Node *)calloc(1, sizeof(Node));
	prev->next = list->head;
	while (i < index) {
		prev = prev->next;
		i++;
	}
	Node *target = prev->next;

	if (target == list->head) {
		list->head = list->head->next;
	}
	if (target == list->tail) {
		list->tail = prev;
		list->tail->next = NULL;
	}
	free_node_data(list->callback_free, target->data);
	// target->next = NULL;
	vPortFree(target);
	list->count--;

	// if (index == 0) {
	// 	Node *target = list->head;
	// 	list->head = list->head->next;
	// 	if (list->count == 1) {
	// 		list->tail = list->head; // both would be NULL
	// 	}
	// 	free_node_data(list->callback_free, target->data);
	// 	free(target);
	// 	list->count--;
	// 	return;
	// }

	// int i = 0;
	// Node *curr_node = list->head;
	// while (i < index-1){
	// 	curr_node = curr_node->next;
	// 	i++;
	// }

	// Node *target_node = curr_node->next;
	// if (target_node == NULL) {
	// 	list->tail = curr_node;
	// }
	// else {
	// 	curr_node->next = target_node->next;
	// 	if (target_node->next == NULL) 
	// 		list->tail = curr_node;

	// 	free_node_data(list->callback_free, target_node->data);		
	// 	free(target_node);
	// }	
	// list->count--;
}


/* makes a full list iteration since our list it is not sorted */
bool list_contains(List *list, void *data) {
	assert(list->callback_compare != NULL);
	
	Node *node = list->head;
	while (node->next != NULL)	{
		if (list->callback_compare(node->data, data) == 0)
			return true;		
		node = node->next;
	}
	return false;
}

Node* list_find(List *list, void *data) {
	assert(list->callback_compare != NULL);

	Node *node = list->head;
	while (node->next != NULL) {
		if (list->callback_compare(node->data, data) == 0)
			return node;
		node = node->next;
	}
	return NULL;
}

Node* list_get_node(List *list, int index) {
	assert(list != NULL);
	assert(index < list->count);
	
	Node *node = list->head;
	
	int i = 0;
	while (i < index && node != NULL) {
		if (index == i)
			return node;
		node = node->next;
		i++;
	}
	return NULL;
}

void* list_get_data(List *list, int index) {
	return list_get_node(list, index)->data;
}

void list_iterate(List *list, CallbackIterate iterate_callback) {
	assert(list != NULL);
	assert(iterate_callback != NULL);

	int i = 0;
	Node *node = list->head;
	while (node!= NULL){
		if (!iterate_callback(i, node))
			return;		
		node = node->next;
		i++;
	}
}

void list_print(List *list) {
	assert(list != NULL);
	
	DEBUG_PRINT("Print a list, the number of elements: %d\n", list->count);
	Node *node = list->head;
	while (node != NULL) {
		list->callback_print(node->data);
		node = node->next;
	}
}
