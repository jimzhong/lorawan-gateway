#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "heap.h"
#include <stdio.h>

heap_t * heap_alloc(long capacity)
{
    heap_t *p;
    p = malloc(sizeof(heap_t));
    if (p == NULL)
        return NULL;
    p->capacity = capacity;
    p->size = 0;
    p->elements = calloc(capacity+1, sizeof(node_t));
    if(p->elements == NULL)
    {
        free(p);
        return NULL;
    }
    return p;
}

void heap_free(heap_t *heap)
{
    free(heap->elements);
    free(heap);
}

long heap_get_size(heap_t *heap)
{
    return heap->size;
}

long heap_get_capacity(heap_t *heap)
{
    return heap->capacity;
}

void heap_insert(heap_t *heap, long weight, void *data)
{
    long pos = ++(heap->size);
    assert(heap->size <= heap->capacity);    // abort if heap is full
    // move down if heavier than weight
    while (pos > 1 && (heap->elements[pos/2].weight > weight))
    {
        heap->elements[pos] = heap->elements[pos/2];
        pos /= 2;
    }
    fprintf(stdout, "Inserted %ld at %ld\n", weight, pos);
    // insert weight at pos
    heap->elements[pos].weight = weight;
    heap->elements[pos].data = data;
    // heap->elements[++(heap->size)].weight = weight;
    // heap->elements[heap->size].data = data;
}

node_t heap_peek(heap_t *heap)
{
    assert(heap->size > 0);
    return heap->elements[1];
}

node_t heap_pop(heap_t *heap)
{
    long pos;
    long min_child;
    node_t last = heap->elements[heap->size];

    // put the popping node to the end
    heap->elements[heap->size] = heap->elements[1];
    heap->size --;

    for(pos = 1;;)
    {
        min_child = 0;
        if (pos*2 <= heap->size)
            min_child = pos * 2;
        if ((pos*2+1 <= heap->size) && (heap->elements[pos*2+1].weight < heap->elements[min_child].weight))
        {
            min_child = pos*2+1;
        }
        if (min_child > 0 && heap->elements[min_child].weight < last.weight)
        {
            heap->elements[pos] = heap->elements[min_child];
            pos = min_child;
        }
        else
        {
            break;
        }
    }
    heap->elements[pos] = last;
    return heap->elements[heap->size + 1];
}
