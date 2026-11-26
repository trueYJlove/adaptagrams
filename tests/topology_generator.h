#ifndef LOCALROUTE_TESTS_TOPOLOGY_GENERATOR_H_
#define LOCALROUTE_TESTS_TOPOLOGY_GENERATOR_H_

#include <cmath>
#include <random>
#include <vector>

#include "localroute/local_orthogonal_router.h"

namespace localroute {

struct Topology {
  std::vector<Block> blocks;
  std::vector<Conn> conns;
  unsigned side = 0;
};

inline Topology GenerateTopology(unsigned n) {
  std::mt19937 rng(42);
  Topology t;
  const unsigned s = static_cast<unsigned>(std::ceil(std::sqrt(n)));
  t.side = s;
  for (Id i = 0; i < n; ++i) {
    const unsigned col = i % s;
    const unsigned row = i / s;
    const double x = col * 170.0 + (col / 8) * 35.0 + (rng() % 21);
    const double y = row * 140.0 + (row / 8) * 30.0 + (rng() % 19);
    t.blocks.push_back(
        {i,
         {x, y, x + 80.0 + (rng() % 41), y + 50.0 + (rng() % 21)},
         {{1, 1.0, 0.3, kRight},
          {2, 0.0, 0.3, kLeft},
          {3, 1.0, 0.7, kRight},
          {4, 0.0, 0.7, kLeft}}});
  }
  for (Id i = 0; i < n; ++i) {
    for (unsigned e = 0; e < 2; ++e) {
      Id dst = 0;
      if ((2 * i + e) % 20 < 3) {
        dst = (i + n / 3 + rng() % (n / 3)) % n;
      } else {
        dst = i + 1 + rng() % 4;
        if (e && i + s < n) {
          dst = i + s;
        }
        if (dst >= n) {
          dst = i - 1;
        }
      }
      if (dst == i) {
        dst = (dst + 1) % n;
      }
      t.conns.push_back({2 * i + e, i, e ? 3u : 1u, dst, e ? 4u : 2u});
    }
  }
  return t;
}

}  // namespace localroute

#endif  // LOCALROUTE_TESTS_TOPOLOGY_GENERATOR_H_
