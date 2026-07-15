#ifndef __SCALER_LIST_H
#define __SCALER_LIST_H

#include "cn_codec_common.h"

#define _offsetof(TYPE, MEMBER)  ((size_t) &((TYPE *)0)->MEMBER)

/**
 * container_of - cast a member of a structure out to the containing structure
 * @ptr:	the pointer to the member.
 * @type:	the type of the container struct this is embedded in.
 * @member:	the name of the member within the struct.
 *
 */
#define container_of(ptr, type, member) ({			\
		const typeof( ((type *)0)->member ) *__mptr = (ptr);	\
		(type *)( (char *)__mptr - _offsetof(type,member) );})

#define LIST_HEAD_INIT(name)  { &name, &name }

#define LIST_HEAD(name)                           \
	listHead name = LIST_HEAD_INIT(name);

#define list_entry(ptr, type, member)             \
	container_of(ptr, type, member)

#define list_first_entry(ptr, type, member)       \
	container_of((ptr)->next, type, member)

#define list_last_entry(ptr, type, member)        \
	container_of((ptr)->prev, type, member)

#define list_next_entry(pos, member)              \
	list_entry((pos)->member.next, typeof(*(pos)), member)

#define list_prev_entry(pos, member)              \
	list_entry((pos)->member.prev, typeof(*(pos)), member)

#define list_for_each(pos, head)                  \
	for(pos=(head)->next; pos!=(head); pos=pos->next)

#define list_for_each_safe(pos, n, head)          \
	for(pos=(head)->next, n=pos->next; pos!=(head); pos=n, n=pos->next)

#define list_for_each_prev(pos, head)             \
	for(pos=(head)->prev; pos!=(head); pos=pos->prev)

#define list_for_each_entry(pos, head, member)    \
	for(pos=list_first_entry(head, typeof(*pos), member); &pos->member!=(head); pos=list_next_entry(pos, member))

#define list_for_each_entry_safe(pos, n, head, member)    \
	for(pos=list_first_entry(head, typeof(*pos), member), \
			n=list_next_entry(pos, member);                   \
			&pos->member!=(head);                             \
			pos=n, n=list_next_entry(n, member))

static inline void INIT_LIST_HEAD(listHead *list) {
	list->next = list;
	list->prev = list;
}

static inline void __list_add(listHead *new, listHead *prev, listHead *next) {
	next->prev = new;
	new->next = next;
	new->prev = prev;
	prev->next = new;
}

static inline void list_add(listHead *new, listHead *head) {
	__list_add(new, head, head->next);
}

static inline void list_add_tail(listHead *new, listHead *head) {
	__list_add(new, head->prev, head);
}

#if 0
/*
 * Delete a list entry by making the prev/next entries
 * point to each other.
 *
 * This is only for internal list manipulation where we know
 * the prev/next entries already!
 */
static inline void __list_del(listHead * prev, listHead * next)
{
	next->prev = prev;
	WRITE_ONCE(prev->next, next);
}

/**
 * list_del - deletes entry from list.
 * @entry: the element to delete from the list.
 * Note: list_empty() on entry does not return true after this, the entry is
 * in an undefined state.
 */
static inline void __list_del_entry(listHead *entry)
{
	__list_del(entry->prev, entry->next);
}

static inline void list_del(listHead *entry)
{
	__list_del(entry->prev, entry->next);
	entry->next = LIST_POISON1;
	entry->prev = LIST_POISON2;
}
#endif

/**
 * list_is_last - tests whether @list is the last entry in list @head
 * @list: the entry to test
 * @head: the head of the list
 */
static inline int list_is_last(const listHead *list,
		const listHead *head)
{
	return list->next == head;
}

/**
 * list_empty - tests whether a list is empty
 * @head: the list to test.
 */
static inline int list_empty(const listHead *head)
{
	if(!head)
		return 1;

	return head->next == head;
}

/**
 * list_empty_careful - tests whether a list is empty and not being modified
 * @head: the list to test
 *
 * Description:
 * tests whether a list is empty _and_ checks that no other CPU might be
 * in the process of modifying either member (next or prev)
 *
 * NOTE: using list_empty_careful() without synchronization
 * can only be safe if the only activity that can happen
 * to the list entry is list_del_init(). Eg. it cannot be used
 * if another CPU could re-list_add() it.
 */
static inline int list_empty_careful(const listHead *head)
{
	listHead *next = head->next;
	return (next == head) && (next == head->prev);
}
#endif
