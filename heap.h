#ifndef __HEAP_H__
#define __HEAP_H__

// Min heap
// heap[1] has the element with the smallest weight

typedef struct
{
    unsigned long weight;
    void *data; //associated data
} node_t;

typedef struct
{
    node_t *elements;
    long capacity;
    long size;
} heap_t;

typedef heap_t *Heap;
heap_t * heap_alloc(long capacity);
void heap_free(heap_t *heap);
long heap_get_size(heap_t *heap);
long heap_get_capacity(heap_t *heap);
void heap_insert(heap_t *heap, unsigned long weight, void *data);
node_t heap_peek(heap_t *heap);
node_t heap_pop(heap_t *heap);

#endif
