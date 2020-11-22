#ifndef LIST_H
#define LIST_H

/* List Element */
struct list_elem {
    struct list_elem * prev;
    struct list_elem * next;
};

/* List */
struct list {
    struct list_elem * head;
    struct list_elem * tail;
};

/* Converts pointer to list element LIST_ELEM into a pointer to
   the structure that LIST_ELEM is embedded inside. */
#define list_entry(LIST_ELEM, STRUCT, MEMBER)           \
        ((STRUCT *) ((uint8_t *) &(LIST_ELEM)->next     \
                     - offsetof (STRUCT, MEMBER.next)))


/* * * * * * * * * * * * * * * */
/*                             */
/*    Function Definitions     */
/*                             */
/* * * * * * * * * * * * * * * */
void list_init (struct list * list);
bool list_is_empty (struct list * list);
void list_enqueue (struct list * list, struct list_elem * elem);
struct list_elem * list_dequeue (struct list * list);

#endif // LIST_H