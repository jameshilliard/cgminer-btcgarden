#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <assert.h>

#include "btcg-vector.h"


struct BTCG_vec* vec_open( size_t elem_size) {
    struct BTCG_vec *v = malloc( sizeof( struct BTCG_vec));
    if (v == NULL) {
        return NULL;
    }

    v->buf = NULL;
    v->buf_size = 0;

    v->elem_size = elem_size;
    v->elem_num = 0;
    return v;
}

void vec_close( struct BTCG_vec *v) {
    free( v->buf);
    free(v);
}

/* Add a new element into the vector */
void vec_push_back( struct BTCG_vec *v, void *new_elem) {
    if (v->buf == NULL) {
        assert( v->buf_size == 0);
        v->buf_size = v->elem_size;
        v->buf = malloc( v->buf_size);
    }
    size_t offset = v->elem_size * v->elem_num;
    assert( offset <= v->buf_size);
    assert( v->buf_size % v->elem_size == 0);

    if ( offset == v->buf_size) {
        v->buf_size *= 2;
        v->buf = realloc( v->buf, v->buf_size);
    }
    memcpy( v->buf + offset, new_elem, v->elem_size);
    v->elem_num += 1;
}

void* vec_at( const struct BTCG_vec *v, size_t n) {
    assert( n < v->elem_num);
    return v->buf + v->elem_size * n;
}

/* Find element with size 4 */
static inline size_t __find_elem_4( const struct BTCG_vec *v, const void *elem) {
    assert( v->elem_size == 4);
    assert( sizeof(uint32_t) == 4);

    const uint32_t *p = (const uint32_t *)(v->buf);
    const uint32_t elem_4 = *((uint32_t*)elem);
    size_t i;
    for ( i = 0; i < v->elem_num; ++i) {
        assert( (void*)(p + i) <= v->buf + v->buf_size - 4);
        if ( *(p + i) == elem_4) {
            return i;
        }
    }
    return VEC_NPOS;
}

size_t vec_find_fst( const struct BTCG_vec *v, const void *elem) {
    if ( v->elem_size == 4) {
        return __find_elem_4( v, elem);
    }
    assert(0 && "Only support find elements whose element size is 4");
    return VEC_NPOS;
}

/* Get number of elements from the vector */
size_t vec_size( const struct BTCG_vec *v) {
    return v->elem_num;
}

/* Clear contents of the vector */
void vec_clear( struct BTCG_vec *v) {
    /* For our usage, no need to clear the space, to avoid too many realloc */
    v->elem_num = 0;
}
