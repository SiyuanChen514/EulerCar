/**
  * @copyright Copyright (c) 2023, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file      nos_list_external.h
  */
#ifndef NOS_LIST_EXTERNAL_H
#define NOS_LIST_EXTERNAL_H

#include "os_typedef.h"

struct TagListObject {
    struct TagListObject *prev;
    struct TagListObject *next;
};

/*
 * @ingroup SREList
 * 初始化链表(链表重用时的初始化)
 *
 * @param head [IN] 链表头结点的地址(The address of the head of a list )
 */
#define OS_LIST_INIT(head)     \
    do {                       \
        (head)->prev = (head); \
        (head)->next = (head); \
    } while (0)

#define LIST_OBJECT_INIT(object) { \
        &(object), &(object)       \
    }

#define LIST_OBJECT(object) (struct TagListObject (object) = LIST_OBJECT_INIT(object))

#define OS_LIST_FIRST(object) ((object)->next)

/* list action low level add */
INLINE void ListLowLevelAdd(struct TagListObject *newNode, struct TagListObject *prev,
    struct TagListObject *next)
{
    newNode->next = next;
    newNode->prev = prev;
    next->prev = newNode;
    prev->next = newNode;
}

/* list action add */
INLINE void ListAdd(struct TagListObject *newNode, struct TagListObject *listObject)
{
    ListLowLevelAdd(newNode, listObject, listObject->next);
}

/* list action tail add */
INLINE void ListTailAdd(struct TagListObject *newNode, struct TagListObject *listObject)
{
    ListLowLevelAdd(newNode, listObject->prev, listObject);
}

/* list action lowel delete */
INLINE void ListLowLevelDelete(struct TagListObject *prevNode, struct TagListObject *nextNode)
{
    if (prevNode == NULL || nextNode == NULL) {
        return;
    }

    nextNode->prev = prevNode;
    prevNode->next = nextNode;
}

/* list action delete */
INLINE void ListDelete(struct TagListObject *node)
{
    ListLowLevelDelete(node->prev, node->next);

    node->next = NULL;
    node->prev = NULL;
}

/* list action empty */
INLINE bool ListEmpty(const struct TagListObject *listObject)
{
    return (bool)(listObject->next == listObject);
}

#define OFFSET_OF_FIELD(type, field) ((uintptr_t)((uintptr_t)(&((type *)0x10)->field) - (uintptr_t)0x10))

#define COMPLEX_OF(ptr, type, field) ((type *)((uintptr_t)(ptr) - OFFSET_OF_FIELD(type, field)))

/* 根据成员地址得到控制块首地址, ptr成员地址, type控制块结构, field成员名 */
#define LIST_COMPONENT(ptrOfList, typeOfList, fieldOfList) COMPLEX_OF(ptrOfList, typeOfList, fieldOfList)

#define LIST_FOR_EACH(posOfList, listObject, typeOfList, field)                                                    \
    for ((posOfList) = LIST_COMPONENT((listObject)->next, typeOfList, field); &(posOfList)->field != (listObject); \
         (posOfList) = LIST_COMPONENT((posOfList)->field.next, typeOfList, field))

#endif /* NOS_LIST_EXTERNAL_H */
