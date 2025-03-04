#pragma once

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <stack>

template<typename Key, typename Value, typename Compare = std::less<Key>>
class btree {
public:
    using value_type = std::pair<const Key, Value>;
    using size_type = std::size_t;
    using reference = std::add_lvalue_reference_t<value_type>;
    using const_reference = std::add_const_t<reference>;
    using pointer = std::add_pointer_t<value_type>;
    using const_pointer = std::add_const_t<pointer>;

private:
    class TreeNode final {
    private:
        pointer data;
        std::shared_ptr<TreeNode> left, right;
        std::weak_ptr<TreeNode> parent;

    public:
        explicit TreeNode(const Key& key, const Value& value) {
            data = new std::pair<const Key, Value>(key, value);
        }

        explicit TreeNode(TreeNode* another) {
            data = another->data;
            left = another->getLeft();
            right = another->getRight();
            parent = another->getParent();
        }

        std::shared_ptr<TreeNode> getRight() { return right; }
        std::shared_ptr<TreeNode> getLeft() { return left; }
        void setLeft(std::shared_ptr<TreeNode> new_left) { left = new_left; }
        void setRight(std::shared_ptr<TreeNode> new_right) { right = new_right; }
        std::weak_ptr<TreeNode> getParent() { return parent; }
        void setParent(std::weak_ptr<TreeNode> new_parent) { parent = new_parent; }
        pointer getData() { return data; }
    };

    template<bool IsConst>
    struct TreeIterator {
    protected:
        std::weak_ptr<TreeNode> node_{};
        friend class btree;

    public:
        TreeIterator() = default;
        explicit TreeIterator(std::shared_ptr<TreeNode> node) { node_ = node; }

        using value_type = std::conditional_t<IsConst, const value_type, value_type>;
        using reference = std::conditional_t<IsConst, const_reference, reference>;
        using pointer = std::conditional_t<IsConst, const_pointer, pointer>;
        using iterator_category = std::bidirectional_iterator_tag;
        using difference_type = std::ptrdiff_t;
        using distance_type = difference_type;

        TreeIterator& operator++() {
            std::shared_ptr<TreeNode> node = node_.lock();
            if (!node) {
                throw std::out_of_range("btree iterator out of range");
            }
            if (node->getRight()) {
                node = node->getRight();
                while (node->getLeft()) {
                    node = node->getLeft();
                }
                node_ = node;
                return *this;
            }
            auto data = node->getData();
            auto parent = node->getParent().lock();
            if (!parent) {
                node = nullptr;
                node_ = node;
                return *this;
            }
            auto parentData = parent->getData();
            if (compare(data->first, parentData->first)) {
                node = parent;
            } else {
                auto cur = parent;
                auto curData = cur->getData();
                while (compare(curData->first, data->first)) {
                    auto curParent = cur->getParent().lock();
                    if (curParent) {
                        cur = curParent;
                    } else {
                        node = nullptr;
                        node_ = node;
                        return *this;
                    }
                }
                node = cur;
            }
            node_ = node;
            return *this;
        }

        TreeIterator operator++(int) {
            auto tmp = *this;
            ++(*this);
            return tmp;
        }

        TreeIterator& operator--() {
            std::shared_ptr<TreeNode> node = node_.lock();
            if (!node) {
                throw std::out_of_range("btree iterator out of range");
            }
            if (node->getLeft()) {
                node = node->getLeft();
                while (node->getRight()) {
                    node = node->getRight();
                }
                node_ = node;
                return *this;
            }
            auto data = node->getData();
            auto parent = node->getParent().lock();
            if (!parent) {
                node = nullptr;
                node_ = node;
                return *this;
            }
            auto parentData = parent->getData();
            if (compare(parentData->first, data->first)) {
                node = parent;
            } else {
                auto cur = parent;
                auto curData = cur->getData();
                while (compare(data->first, curData->first)) {
                    auto curParent = cur->getParent().lock();
                    if (curParent) {
                        cur = curParent;
                    } else {
                        node = nullptr;
                        node_ = node;
                        return *this;
                    }
                }
                node = cur;
            }
            node_ = node;
            return *this;
        }

        TreeIterator operator--(int) {
            auto tmp = *this;
            --this;
            return tmp;
        }

        reference operator*() const {
            std::shared_ptr<TreeNode> node = node_.lock();
            if (!node) {
                throw std::out_of_range("btree iterator out of range");
            }
            return *(node->getData());
        }

        pointer operator->() const {
            std::shared_ptr<TreeNode> node = node_.lock();
            if (!node) {
                throw std::out_of_range("btree iterator out of range");
            }
            return node->getData();
        }

        bool operator==(const TreeIterator& another) const { return node_.lock() == another.node_.lock(); }
        bool operator!=(const TreeIterator& another) const { return !(*this == another); }
    };

    std::shared_ptr<TreeNode> root{};
    size_type size_{};
    inline static Compare compare = Compare();

    std::shared_ptr<TreeNode> leftest() const {
        auto current = root;
        if (!current) {
            return current;
        }
        while (current->getLeft()) {
            current = current->getLeft();
        }
        return current;
    }

    std::shared_ptr<TreeNode> findByKey(const Key& key) const {
        auto current = root;
        if (current) {
            while (current->getData()->first != key) {
                auto curKey = current->getData()->first;
                if (compare(key, curKey)) {
                    current = current->getLeft();
                } else if (!compare(key, curKey)) {
                    current = current->getRight();
                }
                if (!current) {
                    break;
                }
            }
        }
        return current;
    }

public:
    using iterator = TreeIterator<false>;
    using const_iterator = TreeIterator<true>;

    btree() = default;
    explicit btree(const Compare& comp) { compare = comp; }
    btree(const btree& another) { *this = another; }
    btree(btree&& another) noexcept { *this = std::move(another); }

    btree& operator=(const btree& another) {
        if (this != &another) {
            clear();
            for (auto& elem : another) {
                insert(elem);
            }
        }
        return *this;
    }

    btree& operator=(btree&& another) noexcept {
        std::swap(root, another.root);
        std::swap(size_, another.size_);
        std::swap(compare, another.compare);
        return *this;
    }

    iterator begin() {
        auto node = leftest();
        if (!node) {
            return end();
        }
        return iterator(node);
    }

    const_iterator begin() const {
        auto node = leftest();
        if (!node) {
            return end();
        }
        return const_iterator(node);
    }

    iterator end() { return iterator{}; }
    const_iterator end() const { return const_iterator{}; }

    [[nodiscard]] bool empty() const { return size_ == 0; }
    [[nodiscard]] size_type size() const { return size_; }

    Value& operator[](const Key& key) { return insert(std::make_pair(key, Value())).first->second; }
    const Value& operator[](const Key& key) const { return (*this).at(key); }

    Value& at(const Key& key) {
        auto it = find(key);
        if (it == end()) {
            throw std::out_of_range("key not found");
        }
        return (*this)[key];
    }

    const Value& at(const Key& key) const {
        auto it = find(key);
        if (it == end()) {
            throw std::out_of_range("key not found");
        }
        return (*this)[key];
    }

    std::pair<iterator, bool> insert(const value_type& val) {
        auto new_node = std::make_shared<TreeNode>(Key(val.first), Value(val.second));
        if (!root) {
            root = new_node;
            ++size_;
            return std::make_pair(iterator(root), true);
        }
        auto current = root;
        std::shared_ptr<TreeNode> prev;
        while (current) {
            prev = current;
            auto curKey = current->getData()->first;
            if (compare(val.first, curKey)) {
                current = current->getLeft();
            } else {
                current = current->getRight();
            }
        }
        if (!prev) {
            prev = new_node;
        } else if (val.first == prev->getData()->first) {
            return std::make_pair(iterator(prev), false);
        } else if (compare(val.first, prev->getData()->first)) {
            prev->setLeft(new_node);
        } else {
            prev->setRight(new_node);
        }
        new_node->setParent(prev);
        ++size_;
        return std::make_pair(iterator(new_node), true);
    }

    void erase(iterator position) {
        if (position == end()) {
            return;
        }
        erase(position->first);
    }

    size_type erase(const Key& key) {
        auto it = find(key);
        if (it == end()) {
            return 0;
        }
        auto node = it.node_.lock();
        auto state = -1;
        auto parent = node->getParent().lock();
        if (!parent) {
            state = 0;
        } else if (parent->getRight() == node) {
            state = 1;
        }
        if (node->getRight()) {
            auto leftmostOfRight = node->getRight();
            while (leftmostOfRight->getLeft()) {
                leftmostOfRight = leftmostOfRight->getLeft();
            }
            auto left = node->getLeft();
            if (left) {
                left->setParent(leftmostOfRight);
            }
            leftmostOfRight->setLeft(left);
            auto right = node->getRight();
            if (state != 0) {
                right->setParent(node->getParent());
                if (state == -1) {
                    parent->setLeft(right);
                } else {
                    parent->setRight(right);
                }
            } else {
                root = right;
            }
        } else if (node->getLeft()) {
            auto left = node->getLeft();
            if (state != 0) {
                left->setParent(node->getParent());
                if (state == -1) {
                    parent->setLeft(left);
                } else {
                    parent->setRight(left);
                }
            } else {
                root = left;
            }
        } else {
            if (state == -1) {
                parent->setLeft(nullptr);
            } else if (state == 1) {
                parent->setRight(nullptr);
            } else {
                root = nullptr;
            }
        }
        --size_;
        return 1;
    }

    void erase(iterator first, iterator last) {
        for (auto i = first; i != last;) {
            auto next = ++i;
            erase(i);
            i = next;
        }
    }

    void swap(btree& another) noexcept {
        auto tmp = std::move(*this);
        *this = std::move(another);
        another = std::move(tmp);
    }

    void clear() {
        size_ = {};
        root = std::make_shared<TreeNode>(Key(), Value());
    }

    iterator find(const Key& key) {
        auto current = findByKey(key);
        if (!current) {
            return end();
        }
        return iterator(current);
    }

    const_iterator find(const Key& key) const {
        auto current = findByKey(key);
        if (!current) {
            return end();
        }
        return const_iterator(current);
    }
};

template<typename K, typename V, typename C>
bool operator==(const btree<K, V, C>& x, const btree<K, V, C>& y) {
    if (x.size() != y.size()) {
        return false;
    }
    for (auto i = x.begin(), j = y.begin(); i != x.end(); ++i, ++j) {
        if (*i != *j) {
            return false;
        }
    }
    return true;
}

template<typename K, typename V, typename C>
bool operator!=(const btree<K, V, C>& x, const btree<K, V, C>& y) {
    return !(x == y);
}
