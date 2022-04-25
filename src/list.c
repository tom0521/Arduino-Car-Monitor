#include "list.h"

void
list_init (struct list * list) {
    list->head = 0;
    list->tail = 0;
}

bool
list_is_empty (struct list * list) {
    return list->head == 0;
}

void
list_enqueue (struct list * list, struct list_elem * elem) {
    elem->next = 0;
    elem->prev = list->tail;

    // If the list was empty
    if (list->head == 0) {
        list->head = elem;
    } else {
        list->tail->next = elem;
    }

    list->tail = elem;
}

struct list_elem *
list_dequeue (struct list * list) {
    struct list_elem * elem = list->head;
    list->head = elem->next;

    // If one element was in the list
    if (elem->next == 0) {
        list->tail = 0;
    } else {
        elem->next->prev = 0;
    }

    return elem;
}
