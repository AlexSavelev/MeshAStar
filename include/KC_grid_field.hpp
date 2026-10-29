#pragma once

#include "KC_searching.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

enum class GridQueueMode { Heap, Buckets };

// Adapted from Gleb Zavyalov's bucket-ablation/grid_field.hpp (2026-10-28).
// Only the exact heap/bucket alternatives are retained. No sweeping, new
// heuristic or change to MeshA*'s OPEN. Both queues share the same relaxation.
namespace grid_field {
using Item = std::pair<double, int>;
class TimeBuckets {
    std::array<std::vector<Item>, 4> bins_;
    uint64_t current_ = 0;
    size_t size_ = 0;
public:
    bool empty() const { return size_ == 0; }
    void push(Item item) {
        // Distances stay double. Only scheduling uses floor(d). With weights
        // 1 and sqrt(2), vertices in [k,k+1) cannot improve each other.
        if (!std::isfinite(item.first) || item.first < 0)
            throw std::logic_error("Invalid bucket distance");
        const uint64_t bin = static_cast<uint64_t>(item.first);
        if (bin < current_ || bin - current_ >= bins_.size())
            throw std::logic_error("Grid bucket range violated");
        bins_[bin % bins_.size()].push_back(item);
        ++size_;
    }
    Item pop() {
        if (empty()) throw std::logic_error("Empty grid bucket queue");
        while (bins_[current_ % bins_.size()].empty()) ++current_;
        auto &bin = bins_[current_ % bins_.size()];
        Item item = bin.back(); bin.pop_back(); --size_;
        return item;
    }
};
struct HeapQueue {
    std::priority_queue<Item, std::vector<Item>, std::greater<Item>> queue;
    bool empty() const { return queue.empty(); }
    void push(Item item) { queue.push(item); }
    Item pop() { Item item = queue.top(); queue.pop(); return item; }
};

template<class Queue>
void build_with(Map &map, int gi, int gj, float radius, std::vector<double> &d) {
    d.assign(map.height * map.width, std::numeric_limits<double>::infinity());
    Queue queue;
    // Preserve the previous heap implementation's source order and tolerance.
    for (int i = 0; i < map.height; ++i) for (int j = 0; j < map.width; ++j) {
        float d2 = float((i-gi)*(i-gi) + (j-gj)*(j-gj));
        if (d2 <= radius*radius && map.traversable(i,j)) {
            int id = i*map.width+j; d[id] = 0.0; queue.push({0.0,id});
        }
    }
    const double diagonal = std::sqrt(2.0);
    while (!queue.empty()) {
        Item item = queue.pop();
        if (item.first != d[item.second]) continue;
        int i = item.second/map.width, j = item.second%map.width;
        for (int di=-1; di<=1; ++di) for (int dj=-1; dj<=1; ++dj) {
            if (di==0 && dj==0) continue;
            int ni=i+di, nj=j+dj;
            if (!map.in_bounds(ni,nj) || !map.traversable(ni,nj)) continue;
            int next=ni*map.width+nj;
            double nd=item.first+((di && dj)?diagonal:1.0);
            if (nd < d[next]) { d[next]=nd; queue.push({nd,next}); }
        }
    }
}
inline void build(Map &map, int gi, int gj, float radius, GridQueueMode mode,
                  std::vector<double> &d) {
    if (mode == GridQueueMode::Buckets) build_with<TimeBuckets>(map,gi,gj,radius,d);
    else build_with<HeapQueue>(map,gi,gj,radius,d);
}
} // namespace grid_field
