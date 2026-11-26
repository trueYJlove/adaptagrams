# libavoid Release benchmark

Seed 42; same input per scale. Native quality is not validated (NA). Peak is per process, sampled using Windows peak working set.

| Round | Blocks | Mode | Scene | Process | Threads | Samples | P99 ms | Max ms | Valid / display / failed | Unexpected frames | Peak MiB |
|---:|---:|---|---|---|---:|---:|---:|---:|---|---:|---:|
| 1 | 10000 | legacy4 | D_cold | OK | 4 | 1 | 46815.5 | 46815.5 | 19624/376/0 | 0 | 123.67 |
| 1 | 10000 | profile4 | D_cold | OK | 4 | 1 | 73046.9 | 73046.9 | 19624/376/0 | 0 | 123.15 |
