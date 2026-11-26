# libavoid Release benchmark

Seed 42; same input per scale. Native quality is not validated (NA). Peak is per process, sampled using Windows peak working set.

| Round | Blocks | Mode | Scene | Process | Threads | Samples | P99 ms | Max ms | Valid / display / failed | Unexpected frames | Peak MiB |
|---:|---:|---|---|---|---:|---:|---:|---:|---|---:|---:|
| 1 | 10000 | async | first_interactive_proxy | OK | 1 | 1 | 16.3734 | 16.3734 | 0/0/0 | 0 | 86.77 |
| 1 | 10000 | async | async_full | OK | 1 | 1 | 132201 | 132201 | 19624/376/0 | 0 | 86.77 |
| 1 | 10000 | async | owner_A_proxy | OK | 1 | 100 | 0.254 | 0.319 | 0/0/0 | 0 | 86.77 |
| 1 | 10000 | async | settled_A | OK | 1 | 1 | 15.181 | 15.181 | 0/0/0 | 0 | 86.77 |
| 1 | 10000 | async | owner_B20_proxy | OK | 1 | 100 | 0.1886 | 0.2034 | 0/0/0 | 0 | 86.77 |
| 1 | 10000 | async | settled_B20 | OK | 1 | 1 | 460.33 | 460.33 | 0/0/0 | 0 | 86.77 |
