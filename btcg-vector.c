#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "btcg-vector.h"


struct BTCG_vec* BTCG_vec_open( size_t elem_size) {
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

void BTCG_vec_close( struct BTCG_vec *v) {
    free( v->buf);
    free(v);
}

/* Add a new element into the vector */
void BTCG_vec_push_back( struct BTCG_vec *v, void *new_elem) {
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

void* BTCG_vec_at( const struct BTCG_vec *v, size_t n) {
    assert( n < v->elem_num);
    return v->buf + v->elem_size * n;
}

/* Get number of elements from the vector */
size_t BTCG_vec_size( const struct BTCG_vec *v) {
    return v->elem_num;
}

/* Clear contents of the vector */
void BTCG_vec_clear( struct BTCG_vec *v) {
    /* For our usage, no need to clear the space, to avoid too many realloc */
    v->elem_num = 0;
}
