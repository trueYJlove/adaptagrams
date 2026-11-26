#include "localroute/ILocalRouteSolver.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace localroute {
namespace {

using Clock = std::chrono::steady_clock;

constexpr std::uint8_t BLK_OTHER = 1;
constexpr std::uint8_t BLK_SRC = 2;
constexpr std::uint8_t BLK_DST = 4;
constexpr std::uint8_t BLK_ANY = BLK_OTHER | BLK_SRC | BLK_DST;
constexpr std::uint8_t EDGE_BORDER = 8;

constexpr int DIR_DX[4] = {0, 0, -1, 1};
constexpr int DIR_DY[4] = {-1, 1, 0, 0};
constexpr unsigned DIR_MASK[4] = {Up, Down, Left, Right};
constexpr int OPP_IDX[4] = {1, 0, 3, 2};

Point pinPosition(Rect a, const Pin &p) {
    return {p.x == 1 ? a.r : a.l + p.x * (a.r - a.l), p.y == 1 ? a.b : a.t + p.y * (a.b - a.t)};
}

const Pin *findPin(const Block &b, Id id) {
    for (const auto &p : b.pins)
        if (p.id == id)
            return &p;
    return nullptr;
}

bool crosses(Point a, Point b, Rect r) {
    auto s = box(a, b);
    return s.r > r.l && s.l < r.r && s.b > r.t && s.t < r.b;
}

unsigned orthoDir(Point a, Point b) {
    if (a == b)
        return 0;
    if (a.x == b.x)
        return a.y < b.y ? Down : Up;
    if (a.y == b.y)
        return a.x < b.x ? Right : Left;
    return 0;
}

void dedupPreservePins(std::vector<double> &v, double pinA, double pinB) {
    std::sort(v.begin(), v.end());
    if (v.empty())
        return;
    constexpr double eps = 1e-9;
    size_t write = 0;
    for (size_t i = 0; i < v.size(); ++i) {
        double x = v[i];
        if (write == 0) {
            v[write++] = x;
            continue;
        }
        double &prev = v[write - 1];
        if (std::abs(x - prev) <= eps) {
            bool xIsPin = (x == pinA || x == pinB);
            bool prevIsPin = (prev == pinA || prev == pinB);
            if (xIsPin && !prevIsPin) {
                prev = x;
            } else if (xIsPin && prevIsPin && x != prev) {
                v[write++] = x;
            }
        } else {
            v[write++] = x;
        }
    }
    v.resize(write);
}

void injectChannelMidlines(std::vector<double> &v, double buffer) {
    if (v.size() >= 200 || v.size() < 2)
        return;
    double minGap = std::max(buffer * 2.5, 16.0);
    constexpr size_t maxCoords = 240;
    std::vector<std::pair<double, double>> gaps;
    gaps.reserve(v.size());
    for (size_t i = 0; i + 1 < v.size(); ++i) {
        double w = v[i + 1] - v[i];
        if (w > minGap)
            gaps.emplace_back(w, 0.5 * (v[i] + v[i + 1]));
    }
    if (gaps.empty())
        return;
    size_t budget = maxCoords - v.size();
    if (gaps.size() > budget) {
        std::nth_element(gaps.begin(), gaps.begin() + budget, gaps.end(),
                         [](const auto &a, const auto &b) { return a.first > b.first; });
        gaps.resize(budget);
    }
    for (const auto &g : gaps)
        v.push_back(g.second);
    std::sort(v.begin(), v.end());
}

int findCoordIndex(const std::vector<double> &v, double target) {
    auto it = std::lower_bound(v.begin(), v.end(), target);
    if (it != v.end() && *it == target)
        return int(it - v.begin());
    if (it != v.begin() && std::abs(*(it - 1) - target) <= 1e-12)
        return int((it - 1) - v.begin());
    if (it != v.end() && std::abs(*it - target) <= 1e-12)
        return int(it - v.begin());
    return -1;
}

int minBendsToEnterRight(double dx, double dy, int curDir) {
    if (curDir == 3) { // Right
        if (dx > 0.0)
            return dy == 0.0 ? 0 : 2;
        return 4;
    }
    if (curDir == 2) { // Left
        return dy == 0.0 ? 4 : 2;
    }
    // Up (0) or Down (1)
    bool yOk = (dy == 0.0) || (dy > 0.0 && curDir == 1) || (dy < 0.0 && curDir == 0);
    return (dx > 0.0 && yOk) ? 1 : 3;
}

int minRemainingBends(double x, double y, Point b, int dirIdx, unsigned dpDirs) {
    double dx = b.x - x;
    double dy = b.y - y;
    if (dx == 0.0 && dy == 0.0)
        return 0;
    // Rotate coordinate frame so each allowed entry direction dEnter maps to +X (Right=3).
    // Grid indices: 0=Up(0,-1), 1=Down(0,+1), 2=Left(-1,0), 3=Right(+1,0).
    int best = 4;
    for (int dOut = 0; dOut < 4; ++dOut) {
        if ((DIR_MASK[dOut] & dpDirs) == 0)
            continue;
        int dEnter = OPP_IDX[dOut];
        int bends = 4;
        if (dEnter == 3) { // Right (+x, +y)
            bends = minBendsToEnterRight(dx, dy, dirIdx);
        } else if (dEnter == 2) { // Left: rotate 180 deg (-x, -y), 0<->1, 2<->3
            bends = minBendsToEnterRight(-dx, -dy, OPP_IDX[dirIdx]);
        } else if (dEnter == 1) { // Down (+y -> +x'): x' = dy, y' = -dx; Up->Left(2), Down->Right(3), Left->Down(1), Right->Up(0)
            constexpr int rotCW[4] = {2, 3, 1, 0};
            bends = minBendsToEnterRight(dy, -dx, rotCW[dirIdx]);
        } else { // Up (-y -> +x'): x' = -dy, y' = dx; Up->Right(3), Down->Left(2), Left->Up(0), Right->Down(1)
            constexpr int rotCCW[4] = {3, 2, 0, 1};
            bends = minBendsToEnterRight(-dy, dx, rotCCW[dirIdx]);
        }
        if (bends < best)
            best = bends;
    }
    return best;
}

struct HeapNode {
    double f;
    double g;
    std::int32_t state;
};

struct HeapCmp {
    bool operator()(const HeapNode &a, const HeapNode &b) const {
        if (a.f != b.f)
            return a.f > b.f;
        return a.g < b.g; // Break exact f-ties toward goal (larger g = smaller h)
    }
};

struct EntryTarget {
    int ix;
    int iy;
    int dEnterIdx;
    int dOutIdx;
    double cost;
};

struct SolverWorkspace {
    std::vector<double> xs;
    std::vector<double> ys;
    std::vector<std::uint8_t> edgeH;
    std::vector<std::uint8_t> edgeV;
    std::vector<double> gCost;
    std::vector<std::int32_t> parent;
    std::vector<std::uint32_t> visitedStamp;
    std::vector<std::uint32_t> closedStamp;
    std::vector<HeapNode> heap;
    std::vector<Point> rawPts;
    std::uint32_t runStamp = 1;

    void prepareStates(size_t numStates) {
        if (gCost.size() < numStates) {
            gCost.resize(numStates);
            parent.resize(numStates);
            visitedStamp.resize(numStates, 0);
            closedStamp.resize(numStates, 0);
        }
        if (runStamp == std::numeric_limits<std::uint32_t>::max()) {
            std::fill(visitedStamp.begin(), visitedStamp.end(), 0);
            std::fill(closedStamp.begin(), closedStamp.end(), 0);
            runStamp = 1;
        } else {
            ++runStamp;
        }
        heap.clear();
    }
};

struct LazyHananAStarSolver final : ILocalRouteSolver {
    const char *name() const override { return "LazyHananAStarSolver"; }

    std::vector<Point> solve(const Request &q) const override {
        auto tStart = q.profile ? Clock::now() : Clock::time_point();

        const Block *src = nullptr;
        const Block *dst = nullptr;
        for (const Block *bObs : q.obstacles) {
            if (bObs->id == q.conn.src)
                src = bObs;
            if (bObs->id == q.conn.dst)
                dst = bObs;
        }
        if (!src || !dst)
            return {};
        const Pin *sp = findPin(*src, q.conn.sp);
        const Pin *dp = findPin(*dst, q.conn.dp);
        if (!sp || !dp)
            return {};

        const Point a = pinPosition(src->rect, *sp);
        const Point b = pinPosition(dst->rect, *dp);
        if (a == b)
            return {};

        const Rect aa = src->rect.pad(q.cfg.buffer);
        const Rect bb = dst->rect.pad(q.cfg.buffer);

        // Straight 2-point fast check (also handles overlapping aa/bb straight connections).
        if (a.x == b.x || a.y == b.y) {
            unsigned d = orthoDir(a, b);
            unsigned dOpp = DIR_MASK[OPP_IDX[(d == Up ? 0 : d == Down ? 1 : d == Left ? 2 : 3)]];
            if ((d & sp->dirs) && (dOpp & dp->dirs)) {
                bool ok = !aa.inside(b) && (src->rect.inside(a) || !crosses(a, b, src->rect)) &&
                          !bb.inside(a) && (dst->rect.inside(b) || !crosses(b, a, dst->rect));
                if (ok) {
                    for (const Block *bObs : q.obstacles) {
                        if (bObs->id == q.conn.src || bObs->id == q.conn.dst)
                            continue;
                        if (crosses(a, b, bObs->rect.pad(q.cfg.buffer))) {
                            ok = false;
                            break;
                        }
                    }
                }
                if (ok) {
                    if (q.profile)
                        q.profile->graphMs += std::chrono::duration<double, std::milli>(Clock::now() - tStart).count();
                    return {a, b};
                }
            }
        }

        thread_local SolverWorkspace ws;
        auto &xs = ws.xs;
        auto &ys = ws.ys;
        xs.clear();
        ys.clear();
        xs.reserve(q.obstacles.size() * 2 + 8);
        ys.reserve(q.obstacles.size() * 2 + 8);

        xs.push_back(a.x);
        xs.push_back(b.x);
        ys.push_back(a.y);
        ys.push_back(b.y);

        double minX = std::min(a.x, b.x), maxX = std::max(a.x, b.x);
        double minY = std::min(a.y, b.y), maxY = std::max(a.y, b.y);
        for (const Block *bObs : q.obstacles) {
            Rect r = bObs->rect.pad(q.cfg.buffer);
            xs.push_back(r.l);
            xs.push_back(r.r);
            ys.push_back(r.t);
            ys.push_back(r.b);
            minX = std::min(minX, r.l);
            maxX = std::max(maxX, r.r);
            minY = std::min(minY, r.t);
            maxY = std::max(maxY, r.b);
        }

        const double padOut = std::max(q.cfg.buffer * 2.0, 32.0);
        xs.push_back(minX - padOut);
        xs.push_back(maxX + padOut);
        ys.push_back(minY - padOut);
        ys.push_back(maxY + padOut);

        dedupPreservePins(xs, a.x, b.x);
        dedupPreservePins(ys, a.y, b.y);
        injectChannelMidlines(xs, q.cfg.buffer);
        injectChannelMidlines(ys, q.cfg.buffer);

        const int W = int(xs.size());
        const int H = int(ys.size());
        if (W < 2 || H < 2)
            return {};

        const int sx = findCoordIndex(xs, a.x);
        const int sy = findCoordIndex(ys, a.y);
        const int gx = findCoordIndex(xs, b.x);
        const int gy = findCoordIndex(ys, b.y);
        if (sx < 0 || sy < 0 || gx < 0 || gy < 0)
            return {};

        auto &edgeH = ws.edgeH;
        auto &edgeV = ws.edgeV;
        edgeH.assign(size_t(H) * size_t(W - 1), 0);
        edgeV.assign(size_t(H - 1) * size_t(W), 0);

        constexpr double borderEps = 1e-9;
        for (const Block *bObs : q.obstacles) {
            std::uint8_t bit = 0;
            if (bObs->id == q.conn.src)
                bit |= BLK_SRC;
            if (bObs->id == q.conn.dst)
                bit |= BLK_DST;
            if (bObs->id != q.conn.src && bObs->id != q.conn.dst)
                bit |= BLK_OTHER;

            Rect r = bObs->rect.pad(q.cfg.buffer);

            // Mark horizontal micro-segments
            int hx0 = std::max(0, int(std::lower_bound(xs.begin(), xs.end(), r.l) - xs.begin()) - 1);
            int hx1 = std::min(W - 1, int(std::upper_bound(xs.begin(), xs.end(), r.r) - xs.begin()));
            int hy0 = int(std::upper_bound(ys.begin(), ys.end(), r.t) - ys.begin());
            int hy1 = int(std::lower_bound(ys.begin(), ys.end(), r.b) - ys.begin());
            for (int iy = hy0; iy < hy1; ++iy) {
                if (!(r.t < ys[iy] && ys[iy] < r.b))
                    continue;
                std::uint8_t *rowPtr = edgeH.data() + size_t(iy) * size_t(W - 1);
                for (int ix = hx0; ix < hx1; ++ix)
                    if (xs[ix + 1] > r.l && xs[ix] < r.r)
                        rowPtr[ix] |= bit;
            }
            // Border bias on r.t and r.b
            int bt0 = std::max(0, int(std::lower_bound(ys.begin(), ys.end(), r.t - borderEps) - ys.begin()));
            int bt1 = std::min(H, int(std::upper_bound(ys.begin(), ys.end(), r.t + borderEps) - ys.begin()));
            for (int iy = bt0; iy < bt1; ++iy) {
                std::uint8_t *rowPtr = edgeH.data() + size_t(iy) * size_t(W - 1);
                for (int ix = hx0; ix < hx1; ++ix)
                    if (xs[ix + 1] > r.l && xs[ix] < r.r)
                        rowPtr[ix] |= EDGE_BORDER;
            }
            int bb0 = std::max(0, int(std::lower_bound(ys.begin(), ys.end(), r.b - borderEps) - ys.begin()));
            int bb1 = std::min(H, int(std::upper_bound(ys.begin(), ys.end(), r.b + borderEps) - ys.begin()));
            for (int iy = bb0; iy < bb1; ++iy) {
                std::uint8_t *rowPtr = edgeH.data() + size_t(iy) * size_t(W - 1);
                for (int ix = hx0; ix < hx1; ++ix)
                    if (xs[ix + 1] > r.l && xs[ix] < r.r)
                        rowPtr[ix] |= EDGE_BORDER;
            }

            // Mark vertical micro-segments
            int vy0 = std::max(0, int(std::lower_bound(ys.begin(), ys.end(), r.t) - ys.begin()) - 1);
            int vy1 = std::min(H - 1, int(std::upper_bound(ys.begin(), ys.end(), r.b) - ys.begin()));
            int vx0 = int(std::upper_bound(xs.begin(), xs.end(), r.l) - xs.begin());
            int vx1 = int(std::lower_bound(xs.begin(), xs.end(), r.r) - xs.begin());
            for (int iy = vy0; iy < vy1; ++iy) {
                if (!(ys[iy + 1] > r.t && ys[iy] < r.b))
                    continue;
                std::uint8_t *rowPtr = edgeV.data() + size_t(iy) * size_t(W);
                for (int ix = vx0; ix < vx1; ++ix)
                    if (r.l < xs[ix] && xs[ix] < r.r)
                        rowPtr[ix] |= bit;
            }
            // Border bias on r.l and r.r
            int bl0 = std::max(0, int(std::lower_bound(xs.begin(), xs.end(), r.l - borderEps) - xs.begin()));
            int bl1 = std::min(W, int(std::upper_bound(xs.begin(), xs.end(), r.l + borderEps) - xs.begin()));
            for (int iy = vy0; iy < vy1; ++iy) {
                if (!(ys[iy + 1] > r.t && ys[iy] < r.b))
                    continue;
                std::uint8_t *rowPtr = edgeV.data() + size_t(iy) * size_t(W);
                for (int ix = bl0; ix < bl1; ++ix)
                    rowPtr[ix] |= EDGE_BORDER;
            }
            int br0 = std::max(0, int(std::lower_bound(xs.begin(), xs.end(), r.r - borderEps) - xs.begin()));
            int br1 = std::min(W, int(std::upper_bound(xs.begin(), xs.end(), r.r + borderEps) - xs.begin()));
            for (int iy = vy0; iy < vy1; ++iy) {
                if (!(ys[iy + 1] > r.t && ys[iy] < r.b))
                    continue;
                std::uint8_t *rowPtr = edgeV.data() + size_t(iy) * size_t(W);
                for (int ix = br0; ix < br1; ++ix)
                    rowPtr[ix] |= EDGE_BORDER;
            }
        }

        auto edgeFlags = [&](int x0, int y0, int dIdx) -> std::uint8_t {
            if (dIdx == 0) // Up
                return edgeV[size_t(y0 - 1) * size_t(W) + size_t(x0)];
            if (dIdx == 1) // Down
                return edgeV[size_t(y0) * size_t(W) + size_t(x0)];
            if (dIdx == 2) // Left
                return edgeH[size_t(y0) * size_t(W - 1) + size_t(x0 - 1)];
            return edgeH[size_t(y0) * size_t(W - 1) + size_t(x0)];
        };

        auto stepLength = [&](int x0, int y0, int x1, int y1) -> double {
            return std::abs(xs[x1] - xs[x0]) + std::abs(ys[y1] - ys[y0]);
        };

        // Precompute valid entry points on/outside bb boundary along dp.dirs.
        EntryTarget entries[4];
        int numEntries = 0;
        for (int dOutIdx = 0; dOutIdx < 4; ++dOutIdx) {
            if ((DIR_MASK[dOutIdx] & dp->dirs) == 0)
                continue;
            int cx = gx, cy = gy;
            double cost = 0.0;
            int steps = 0;
            for (;;) {
                int nx = cx + DIR_DX[dOutIdx];
                int ny = cy + DIR_DY[dOutIdx];
                if (nx < 0 || nx >= W || ny < 0 || ny >= H)
                    break;
                if (steps == 0 && !dst->rect.inside(b) &&
                    crosses({xs[cx], ys[cy]}, {xs[nx], ys[ny]}, dst->rect))
                    break;
                std::uint8_t f = edgeFlags(cx, cy, dOutIdx);
                if (f & (BLK_OTHER | BLK_SRC))
                    break;
                cost += stepLength(cx, cy, nx, ny);
                cx = nx;
                cy = ny;
                ++steps;
                if (!bb.inside({xs[cx], ys[cy]})) {
                    entries[numEntries++] = {cx, cy, OPP_IDX[dOutIdx], dOutIdx, cost};
                    break;
                }
            }
        }
        if (numEntries == 0) {
            if (q.profile)
                q.profile->graphMs += std::chrono::duration<double, std::milli>(Clock::now() - tStart).count();
            return {};
        }

        const size_t numStates = size_t(W) * size_t(H) * 4;
        auto &heap = ws.heap;
        HeapCmp cmp;

        const double abx = b.x - a.x;
        const double aby = b.y - a.y;
        const double abLen = std::hypot(abx, aby);
        const double lineNx = abLen > 1.0 ? -aby / abLen : 0.0;
        const double lineNy = abLen > 1.0 ? abx / abLen : 0.0;
        const bool truncatedWindow = q.obstacles.size() >= q.cfg.budget;
        auto perpDist = [&](double x, double y) -> double {
            return abLen > 1.0 ? std::abs((x - a.x) * lineNx + (y - a.y) * lineNy) : 0.0;
        };

        auto runAStar = [&](double heuristicWeight, double corridorWeight, unsigned maxPops,
                            std::int32_t &bestGoalParent) -> bool {
            ws.prepareStates(numStates);
            const std::uint32_t stamp = ws.runStamp;

            auto pushStart = [&](std::int32_t s, double g, double f) {
                if (ws.visitedStamp[s] != stamp || g < ws.gCost[s]) {
                    ws.visitedStamp[s] = stamp;
                    ws.gCost[s] = g;
                    ws.parent[s] = -1;
                    heap.push_back({f, g, s});
                    std::push_heap(heap.begin(), heap.end(), cmp);
                }
            };

            // Precompute valid exit rays leaving aa boundary along sp.dirs.
            for (int d0 = 0; d0 < 4; ++d0) {
                if ((DIR_MASK[d0] & sp->dirs) == 0)
                    continue;
                int cx = sx, cy = sy;
                double cost = 0.0;
                int steps = 0;
                for (;;) {
                    int nx = cx + DIR_DX[d0];
                    int ny = cy + DIR_DY[d0];
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H)
                        break;
                    if (steps == 0 && !src->rect.inside(a) &&
                        crosses({xs[cx], ys[cy]}, {xs[nx], ys[ny]}, src->rect))
                        break;
                    std::uint8_t f = edgeFlags(cx, cy, d0);
                    if (f & (BLK_OTHER | BLK_DST))
                        break;
                    cost += stepLength(cx, cy, nx, ny);
                    cx = nx;
                    cy = ny;
                    ++steps;
                    if (!aa.inside({xs[cx], ys[cy]})) {
                        std::int32_t s0 = ((cy * W + cx) << 2) | d0;
                        double h = std::abs(xs[cx] - b.x) + std::abs(ys[cy] - b.y) +
                                   minRemainingBends(xs[cx], ys[cy], b, d0, dp->dirs) * q.cfg.penalty;
                        double cBias = corridorWeight > 0.0 ? corridorWeight * perpDist(xs[cx], ys[cy]) : 0.0;
                        pushStart(s0, cost, cost + heuristicWeight * h + cBias);
                        break;
                    }
                }
            }

            if (heap.empty())
                return false;

            double bestGoalCost = std::numeric_limits<double>::infinity();
            bestGoalParent = -1;
            unsigned pops = 0;

            while (!heap.empty()) {
                HeapNode cur = heap.front();
                std::pop_heap(heap.begin(), heap.end(), cmp);
                heap.pop_back();

                if (cur.state == -2) {
                    if (cur.g <= bestGoalCost)
                        return bestGoalParent != -1;
                    continue;
                }

                if (ws.closedStamp[cur.state] == stamp || cur.g > ws.gCost[cur.state])
                    continue;
                ws.closedStamp[cur.state] = stamp;

                if (++pops > maxPops)
                    return false;

                int cell = cur.state >> 2;
                int curDir = cur.state & 3;
                int iy = cell / W;
                int ix = cell - iy * W;

                for (int ei = 0; ei < numEntries; ++ei) {
                    const auto &e = entries[ei];
                    if (e.ix != ix || e.iy != iy || curDir == e.dOutIdx)
                        continue;
                    double candG = cur.g + e.cost + (curDir != e.dEnterIdx ? q.cfg.penalty : 0.0);
                    if (candG < bestGoalCost) {
                        bestGoalCost = candG;
                        bestGoalParent = cur.state;
                        heap.push_back({candG, candG, -2});
                        std::push_heap(heap.begin(), heap.end(), cmp);
                    }
                }

                for (int d = 0; d < 4; ++d) {
                    if (d == OPP_IDX[curDir])
                        continue;
                    int nx = ix + DIR_DX[d];
                    int ny = iy + DIR_DY[d];
                    if (nx < 0 || nx >= W || ny < 0 || ny >= H)
                        continue;
                    std::uint8_t f = edgeFlags(ix, iy, d);
                    if (f & BLK_ANY)
                        continue;

                    std::int32_t ns = ((ny * W + nx) << 2) | d;
                    if (ws.closedStamp[ns] == stamp)
                        continue;

                    double ng = cur.g + stepLength(ix, iy, nx, ny) + (d != curDir ? q.cfg.penalty : 0.0) +
                                ((f & EDGE_BORDER) ? 1e-4 : 0.0);
                    if (ws.visitedStamp[ns] != stamp || ng < ws.gCost[ns]) {
                        ws.visitedStamp[ns] = stamp;
                        ws.gCost[ns] = ng;
                        ws.parent[ns] = cur.state;
                        double h = std::abs(xs[nx] - b.x) + std::abs(ys[ny] - b.y) +
                                   minRemainingBends(xs[nx], ys[ny], b, d, dp->dirs) * q.cfg.penalty;
                        double cBias = corridorWeight > 0.0 ? corridorWeight * perpDist(xs[nx], ys[ny]) : 0.0;
                        heap.push_back({ng + heuristicWeight * h + cBias, ng, ns});
                        std::push_heap(heap.begin(), heap.end(), cmp);
                    }
                }
            }
            return false;
        };

        if (q.profile) {
            auto tMid = Clock::now();
            q.profile->graphMs += std::chrono::duration<double, std::milli>(tMid - tStart).count();
            tStart = tMid;
        }

        std::int32_t bestGoalParent = -1;
        // Pass 1: exact admissible A* with tight cap (preserves channel centering & optimality).
        bool goalFound = runAStar(1.0, 0.0, truncatedWindow ? 800u : 2500u, bestGoalParent);
        // Pass 2: gentle corridor-guided weighted A* fallback on the already-built grid.
        if (!goalFound && !heap.empty())
            goalFound = runAStar(1.15, truncatedWindow ? 0.02 : 0.0, 20000u, bestGoalParent);

        std::vector<Point> out;
        if (goalFound && bestGoalParent != -1) {
            auto &rev = ws.rawPts;
            rev.clear();
            for (std::int32_t s = bestGoalParent; s != -1; s = ws.parent[s]) {
                int cell = s >> 2;
                int iy = cell / W;
                int ix = cell - iy * W;
                rev.push_back({xs[ix], ys[iy]});
            }
            out.reserve(rev.size() + 2);
            out.push_back(a);
            for (auto it = rev.rbegin(); it != rev.rend(); ++it) {
                if (!(*it == out.back()))
                    out.push_back(*it);
            }
            if (!(out.back() == b))
                out.push_back(b);

            // Merge collinear points while keeping exact terminals.
            size_t count = 0;
            for (size_t i = 0; i < out.size(); ++i) {
                Point p = out[i];
                if (count >= 1 && p == out[count - 1])
                    continue;
                if (count >= 2 && orthoDir(out[count - 2], out[count - 1]) == orthoDir(out[count - 1], p))
                    --count;
                out[count++] = p;
            }
            out.resize(count);
        }

        if (q.profile)
            q.profile->searchMs += std::chrono::duration<double, std::milli>(Clock::now() - tStart).count();
        return out;
    }
};

} // namespace

std::shared_ptr<const ILocalRouteSolver> lazyHananAStarSolver() {
    return std::make_shared<LazyHananAStarSolver>();
}

} // namespace localroute
