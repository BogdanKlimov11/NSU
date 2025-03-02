#pragma once

#include <array>
#include <vector>
#include <bitset>
#include <numeric>

inline constexpr std::size_t number_of_unique_hashes = 64;

template<typename T,
         typename Hash = std::hash<T>,
         size_t TableSize = 50,
         size_t NumFunctions = number_of_unique_hashes,
         typename Dummy = std::enable_if_t<NumFunctions <= number_of_unique_hashes, bool>>
class BloomFilterCpp final {
    using hashfunc_t = Hash;
    hashfunc_t hash_func{};
    std::bitset<TableSize> _table{};
    const std::array<size_t, number_of_unique_hashes> salts = {
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

    std::bitset<TableSize>& table() { return _table; }

public:
    BloomFilterCpp() = default;

    void insert(const T& value) noexcept;

    bool query(const T& value) noexcept;

    const std::bitset<TableSize>& read() const noexcept { return _table; }

    void load(std::bitset<TableSize>& array) noexcept { _table = array; }

    [[nodiscard]] std::size_t table_size() const noexcept { return TableSize; }

    [[nodiscard]] std::size_t num_of_functions() const noexcept { return NumFunctions; }

    BloomFilterCpp<T, Hash, TableSize, NumFunctions> operator&(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) const noexcept;

    BloomFilterCpp<T, Hash, TableSize, NumFunctions> operator|(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) const noexcept;

    BloomFilterCpp<T, Hash, TableSize, NumFunctions>& operator&=(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) noexcept;

    BloomFilterCpp<T, Hash, TableSize, NumFunctions>& operator|=(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) noexcept;
};

template<typename T, typename Hash, size_t TableSize, size_t NumFunctions, typename Dummy>
void BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>::insert(const T& value) noexcept {
    const auto hash = hash_func(value);
    for (size_t i = 0; i < num_of_functions(); i++) {
        const auto idx = (hash ^ salts[i]) % table_size();
        table().set(idx);
    }
}

template<typename T, typename Hash, size_t TableSize, size_t NumFunctions, typename Dummy>
bool BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>::query(const T& value) noexcept {
    const auto hash = hash_func(value);
    for (size_t i = 0; i < num_of_functions(); i++) {
        const auto idx = (hash ^ salts[i]) % table_size();
        if (!table()[idx]) {
            return false;
        }
    }
    return true;
}

template<typename T, typename Hash, size_t TableSize, size_t NumFunctions, typename Dummy>
BloomFilterCpp<T, Hash, TableSize, NumFunctions>
BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>::operator|(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) const noexcept {
    auto res = BloomFilterCpp<T, Hash, TableSize, NumFunctions>();
    std::bitset<TableSize> new_table = read() | other.read();
    res.load(new_table);
    return res;
}

template<typename T, typename Hash, size_t TableSize, size_t NumFunctions, typename Dummy>
BloomFilterCpp<T, Hash, TableSize, NumFunctions>
BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>::operator&(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) const noexcept {
    auto res = BloomFilterCpp<T, Hash, TableSize, NumFunctions>();
    std::bitset<TableSize> new_table = read() & other.read();
    res.load(new_table);
    return res;
}

template<typename T, typename Hash, size_t TableSize, size_t NumFunctions, typename Dummy>
BloomFilterCpp<T, Hash, TableSize, NumFunctions>& BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>::operator&=(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) noexcept {
    auto other_table = other.read();
    table() &= other_table;
    return *this;
}

template<typename T, typename Hash, size_t TableSize, size_t NumFunctions, typename Dummy>
BloomFilterCpp<T, Hash, TableSize, NumFunctions>& BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>::operator|=(const BloomFilterCpp<T, Hash, TableSize, NumFunctions, Dummy>& other) noexcept {
    auto other_table = other.read();
    table() |= other_table;
    return *this;
}
