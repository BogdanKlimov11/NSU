#pragma once

#include <vector>
#include <memory>
#include <utility>
#include <stdexcept>

template<typename T>
class ObjectPool final {
private:
    std::unique_ptr<uint8_t[]> data;
    size_t size;
    std::vector<bool> is_empty;

public:
    ObjectPool() = delete;
    explicit ObjectPool(size_t size) 
        : data(std::make_unique<uint8_t[]>(size * sizeof(T))), 
          size(size), 
          is_empty(size, true) {}

    ~ObjectPool() override {
        for (size_t i = 0; i < size; ++i) {
            if (!is_empty[i]) {
                reinterpret_cast<T*>(data.get() + i * sizeof(T))->~T();
            }
        }
    }

    template <typename... Args>
    T& alloc(Args&&... args) {
        for (size_t i = 0; i < size; ++i) {
            if (is_empty[i]) {
                auto p = new (data.get() + i * sizeof(T)) T(std::forward<Args>(args)...);
                is_empty[i] = false;
                return *p;
            }
        }
        throw std::runtime_error("ObjectPool: No available objects for allocation.");
    }

    void free(T& obj) {
        for (size_t i = 0; i < size; ++i) {
            if (!is_empty[i] && (data.get() + i * sizeof(T)) == static_cast<uint8_t*>(static_cast<void*>(&obj))) {
                is_empty[i] = true;
                obj.~T();
                return;
            }
        }
        throw std::runtime_error("ObjectPool: Attempted to free an invalid object.");
    }
};
