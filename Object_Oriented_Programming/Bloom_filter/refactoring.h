#ifndef ALGORITHM_BLOOM_FILTER_HPP
#define ALGORITHM_BLOOM_FILTER_HPP

#ifdef __cplusplus
extern "C" {
#endif

    typedef struct _BloomFilter BloomFilter;
    typedef void* BloomFilterValue;
    typedef unsigned int (*BloomFilterHashFunc)(BloomFilterValue data);

    BloomFilter* bloom_filter_new(unsigned int table_size, BloomFilterHashFunc hash_func, unsigned int num_functions);
    void bloom_filter_free(BloomFilter* bloomfilter);
    void bloom_filter_insert(BloomFilter* bloomfilter, BloomFilterValue value);
    int bloom_filter_query(BloomFilter* bloomfilter, void* value);
    double bloom_filter_read(BloomFilter* bloomfilter, unsigned char* array);
    void bloom_filter_load(BloomFilter* bloomfilter, unsigned char* array);
    BloomFilter* bloom_filter_union(BloomFilter* filter1, BloomFilter* filter2);
    BloomFilter* bloom_filter_intersection(BloomFilter* filter1, BloomFilter* filter2);

#ifdef __cplusplus
}
#endif

#endif
