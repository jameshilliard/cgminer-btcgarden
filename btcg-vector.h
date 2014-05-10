#ifndef BTCG_VECTOR_H
#define BTCG_VECTOR_H

struct BTCG_vec {
    void *buf;
    size_t buf_size;    // buffer size

    size_t elem_size;   // size of each element in the vector
    size_t elem_num;    // how many element in the vector
};

/* Return NULL on error */
struct BTCG_vec* vec_open( size_t elem_size);
void vec_close( struct BTCG_vec *v);

/* Add a new element into the vector */
void vec_push_back( struct BTCG_vec *v, void *new_elem);

/* Get an element from the vector */
void* vec_at( const struct BTCG_vec *v, size_t n);

/* Get number of elements from the vector */
size_t vec_size( const struct BTCG_vec *v);

/* Clear contents of the vector */
void vec_clear( struct BTCG_vec *v);

#endif
