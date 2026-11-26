#pragma once
#include "LocalOrthogonalRouter.h"
#include <random>
struct Topology {
    std::vector<localroute::Block> blocks;
    std::vector<localroute::Conn> conns;
    unsigned side;
};
inline Topology generate(unsigned n) {
    using namespace localroute;
    std::mt19937 rng(42);
    Topology t;
    unsigned s = unsigned(std::ceil(std::sqrt(n)));
    t.side = s;
    for (Id i = 0; i < n; ++i) {
        unsigned col = i % s, row = i / s;
        double x = col * 170. + (col / 8) * 35 + (rng() % 21), y = row * 140. + (row / 8) * 30 + (rng() % 19);
        t.blocks.push_back({i,
                            {x, y, x + 80 + rng() % 41, y + 50 + rng() % 21},
                            {{1, 1, .3, Right}, {2, 0, .3, Left}, {3, 1, .7, Right}, {4, 0, .7, Left}}});
    }
    for (Id i = 0; i < n; ++i)
        for (unsigned e = 0; e < 2; ++e) {
            Id dst;
            if ((2 * i + e) % 20 < 3) {
                dst = (i + n / 3 + rng() % (n / 3)) % n;
            } else {
                dst = i + 1 + rng() % 4;
                if (e && i + s < n)
                    dst = i + s;
                if (dst >= n)
                    dst = i - 1;
            }
            if (dst == i)
                dst = (dst + 1) % n;
            t.conns.push_back({2 * i + e, i, e ? 3u : 1u, dst, e ? 4u : 2u});
        }
    return t;
}
