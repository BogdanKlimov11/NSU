#include <cstdlib>
#include <cstring>

#include "refactoring.h"

struct _BloomFilter {
    BloomFilterHashFunc hash_func;
    unsigned char* table;
    unsigned int table_size;
    unsigned int num_functions;
};

static const unsigned int salts[] = {
    0x1953c322, 0x588ccf17, 0x64bf600c, 0xa6be3f3d,
    0x341a02ea, 0x15b03217, 0x3b062858, 0x5956fd06,
    0x18b5624f, 0xe3be0b46, 0x20ffcd5c, 0xa35dfd2b,
    0x1fc4a9bf, 0x57c45d5c, 0xa8661c4a, 0x4f1b74d2,
    0x5a6dde13, 0x3b18dac6, 0x05a8afbf, 0xbbda2fe2,
    0xa2520d78, 0xe7934849, 0xd541bc75, 0x09a55b57,
    0x9b345ae2, 0xfc2d26af, 0x38679cef, 0x81bd1e0d,
    0x654681ae, 0x4b3d87ad, 0xd5ff10fb, 0x23b32f67,
    0xafc7e366, 0xdd955ead, 0xe7c34b1c, 0xfeace0a6,
    0xeb16f09d, 0x3c57a72d, 0x2c8294c5, 0xba92662a,
    0xcd5b2d14, 0x743936c8, 0x2489beff, 0xc6c56e00,
    0x74a4f606, 0xb244a94a, 0x5edfc423, 0xf1901934,
    0x24af7691, 0xf6c98b25, 0xea25af46, 0x76d5f2e6,
    0x5e33cdf2, 0x445eb357, 0x88556bd2, 0x70d1da7a,
    0x54449368, 0x381020bc, 0x1c0520bf, 0xf7e44942,
    0xa27e2a58, 0x66866fc5, 0x12519ce7, 0x437a8456,
};

BloomFilter* bloom_filter_new(unsigned int table_size, BloomFilterHashFunc hash_func, unsigned int num_functions) {
    BloomFilter* filter;

    if (num_functions > sizeof(salts) / sizeof(*salts)) {
        return nullptr;
    }

    filter = static_cast<BloomFilter*>(malloc(sizeof(BloomFilter)));
    if (filter == nullptr) {
        return nullptr;
    }

    filter->table = static_cast<unsigned char*>(calloc((table_size + 7) / 8, 1));
    if (filter->table == nullptr) {
        free(filter);
        return nullptr;
    }

    filter->hash_func = hash_func;
    filter->num_functions = num_functions;
    filter->table_size = table_size;
    return filter;
}

void bloom_filter_free(BloomFilter* bloomfilter) {
    free(bloomfilter->table);
    free(bloomfilter);
}

void bloom_filter_insert(BloomFilter* bloomfilter, BloomFilterValue value) {
    unsigned int hash = bloomfilter->hash_func(value);
    for (unsigned int i = 0; i < bloomfilter->num_functions; ++i) {
        unsigned int subhash = hash ^ salts[i];
        unsigned int index = subhash % bloomfilter->table_size;
        unsigned char b = static_cast<unsigned char>(1 << (index % 8));
        bloomfilter->table[index / 8] |= b;
    }
}

int bloom_filter_query(BloomFilter* bloomfilter, BloomFilterValue value) {
    unsigned int hash = bloomfilter->hash_func(value);
    for (unsigned int i = 0; i < bloomfilter->num_functions; ++i) {
        unsigned int subhash = hash ^ salts[i];
        unsigned int index = subhash % bloomfilter->table_size;
        unsigned char b = bloomfilter->table[index / 8];
        int bit = 1 << (index % 8);
        if ((b & bit) == 0) {
            return 0;
        }
    }
    return 1;
}

double bloom_filter_read(BloomFilter* bloomfilter, unsigned char* array) {
    unsigned int array_size = (bloomfilter->table_size + 7) / 8;
    memcpy(array, bloomfilter->table, array_size);
    /* Предполагается, что функция должна возвращать double, но в оригинале возврата нет — исправлено */
    return 0.0;
}

void bloom_filter_load(BloomFilter* bloomfilter, unsigned char* array) {
    unsigned int array_size = (bloomfilter->table_size + 7) / 8;
    memcpy(bloomfilter->table, array, array_size);
}

BloomFilter* bloom_filter_union(BloomFilter* filter1, BloomFilter* filter2) {
    if (filter1->table_size != filter2->table_size || filter1->num_functions != filter2->num_functions || filter1->hash_func != filter2->hash_func) {
        return nullptr;
    }

    BloomFilter* result = bloom_filter_new(filter1->table_size, filter1->hash_func, filter1->num_functions);
    if (result == nullptr) {
        return nullptr;
    }

    unsigned int array_size = (filter1->table_size + 7) / 8;
    for (unsigned int i = 0; i < array_size; ++i) {
        result->table[i] = filter1->table[i] | filter2->table[i];
    }
    return result;
}

BloomFilter* bloom_filter_intersection(BloomFilter* filter1, BloomFilter* filter2) {
    if (filter1->table_size != filter2->table_size || filter1->num_functions != filter2->num_functions || filter1->hash_func != filter2->hash_func) {
        return nullptr;
    }

    BloomFilter* result = bloom_filter_new(filter1->table_size, filter1->hash_func, filter1->num_functions);
    if (result == nullptr) {
        return nullptr;
    }

    unsigned int array_size = (filter1->table_size + 7) / 8;
    for (unsigned int i = 0; i < array_size; ++i) {
        result->table[i] = filter1->table[i] & filter2->table[i];
    }
    return result;
}
