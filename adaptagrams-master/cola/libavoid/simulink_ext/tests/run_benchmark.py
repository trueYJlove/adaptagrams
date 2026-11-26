"""Run the Release matrix with process-isolated timeouts; write CSV and a readable report."""
import argparse
import csv
import ctypes as ct
import io
import subprocess
import time
from pathlib import Path
from ctypes import wintypes as wt
import os

FIELDS = 'solver_name,blocks,scene,threads,samples,avg_ms,p50_ms,p95_ms,p99_ms,max_ms,valid,display_only,failed,solver_calls,retries,window_p99,window_max,unexpected_frames,reused,simple,pending,query_ms,setup_ms,graph_ms,search_ms,validation_ms,other_ms,invalid_attempts,collision_attempts,capped_attempts,coincident,buried,invalid_candidate,collision,budget'.split(',')

if os.name == 'nt':
    class Counters(ct.Structure):
        _fields_ = [('cb', wt.DWORD), ('faults', wt.DWORD), ('peak', ct.c_size_t),
                    ('working', ct.c_size_t)] + [(f'n{i}', ct.c_size_t) for i in range(6)]

    info = ct.WinDLL('psapi').GetProcessMemoryInfo
    info.argtypes = [wt.HANDLE, ct.POINTER(Counters), wt.DWORD]
    info.restype = wt.BOOL


def run(args, limit, log):
    peak = 0
    with log.open('w', encoding='utf-8') as err, log.with_suffix('.stdout').open('w+', encoding='utf-8') as out:
        with subprocess.Popen(args, stdout=out, stderr=err, cwd=log.parent) as proc:
            start = time.monotonic()
            while proc.poll() is None:
                if os.name == 'nt':
                    m = Counters()
                    m.cb = ct.sizeof(m)
                    if info(wt.HANDLE(int(proc._handle)), ct.byref(m), m.cb):
                        peak = max(peak, m.peak)
                if time.monotonic() - start >= limit:
                    proc.kill()
                    proc.wait()
                    status = 'TIMEOUT'
                    break
                time.sleep(.05)
            else:
                status = 'OK' if not proc.returncode else f'ERROR({proc.returncode})'
        out.seek(0)
        rows = list(csv.DictReader(io.StringIO(out.read()), fieldnames=FIELDS))
    return status, round(peak / 2**20, 2) if peak else '', rows


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('exe', type=Path)
    p.add_argument('output', type=Path)
    p.add_argument('--rounds', type=int, default=1, choices=range(1, 11))
    p.add_argument('--scales', type=int, nargs='+', default=[1000, 3000, 5000, 10000])
    p.add_argument('--modes', nargs='+', choices=['default', 'tuned', 'local1', 'local4', 'async', 'legacy4', 'profile4'],
                   default=['default', 'tuned', 'local1', 'local4'])
    a = p.parse_args()
    out = a.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    with (out / 'benchmark_results.csv').open('w', newline='', encoding='utf-8') as f:
        cols = ['round', 'mode', 'process_status', 'peak_working_mib'] + FIELDS
        w = csv.DictWriter(f, cols)
        w.writeheader()
        report = ['# libavoid Release benchmark', '',
                  'Seed 42; same input per scale. Native quality is not validated (NA). Peak is per process, sampled using Windows peak working set.', '',
                  '| Round | Blocks | Mode | Scene | Process | Threads | Samples | P99 ms | Max ms | Valid / display / failed | Unexpected frames | Peak MiB |',
                  '|---:|---:|---|---|---|---:|---:|---:|---:|---|---:|---:|']
        for rnd in range(1, a.rounds + 1):
            stop = set()
            for n in sorted(a.scales):
                for mode in a.modes:
                    if mode in stop:
                        status, peak, rows = 'SKIPPED', '', []
                    else:
                        prefix = out / f'{rnd}-{n}-{mode}.log'
                        status, peak, rows = run([str(a.exe.resolve()), mode, str(n), str(out)],
                                                 15 if mode in ('default', 'tuned') else 900, prefix)
                        if status == 'TIMEOUT' and mode in ('default', 'tuned'):
                            stop.add(mode)
                    print(rnd, n, mode, status, flush=True)
                    for row in rows or [{'blocks': n, 'scene': 'no completed scene'}]:
                        w.writerow({'round': rnd, 'mode': mode, 'process_status': status,
                                    'peak_working_mib': peak, **row})
                        counts = '/'.join(str(row.get(k, '-')) for k in ['valid', 'display_only', 'failed'])
                        report.append(f"| {rnd} | {n} | {mode} | {row['scene']} | {status} | "
                                      f"{row.get('threads','-')} | {row.get('samples','-')} | {row.get('p99_ms','-')} | "
                                      f"{row.get('max_ms','-')} | {counts} | {row.get('unexpected_frames','-')} | {peak} |")
                    f.flush()
                    (out / 'benchmark_results.md').write_text('\n'.join(report) + '\n', encoding='utf-8')


if __name__ == '__main__':
    main()
