#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <vector>

struct Point {
    double x, y;
};

struct ClosestResult {
    double dist;
    Point p1, p2;
};

static double euclidean_dist(const Point& a, const Point& b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

static ClosestResult brute_force(const std::vector<Point>& pts, int left, int right) {
    ClosestResult best{std::numeric_limits<double>::infinity(), {0.0, 0.0}, {0.0, 0.0}};
    for (int i = left; i <= right; ++i) {
        for (int j = i + 1; j <= right; ++j) {
            double d = euclidean_dist(pts[i], pts[j]);
            if (d < best.dist) {
                best = {d, pts[i], pts[j]};
            }
        }
    }
    return best;
}

static ClosestResult strip_closest(const std::vector<Point>& strip, ClosestResult best) {
    int m = static_cast<int>(strip.size());
    for (int i = 0; i < m; ++i) {
        for (int j = i + 1; j < m && (strip[j].y - strip[i].y) < best.dist; ++j) {
            double d = euclidean_dist(strip[i], strip[j]);
            if (d < best.dist) {
                best = {d, strip[i], strip[j]};
            }
        }
    }
    return best;
}

static ClosestResult closest_util(const std::vector<Point>& px, const std::vector<Point>& py, int left, int right) {
    int n = right - left + 1;
    if (n <= 3) {
        return brute_force(px, left, right);
    }

    int mid = left + (right - left) / 2;
    Point mid_pt = px[mid]; 

    std::vector<Point> py_left;
    std::vector<Point> py_right;
    py_left.reserve(mid - left + 1);
    py_right.reserve(right - mid);

    for (const auto& pt : py) {
        if (pt.x < mid_pt.x || (pt.x == mid_pt.x && pt.y <= mid_pt.y)) {
            py_left.push_back(pt);
        } else {
            py_right.push_back(pt);
        }
    }

    ClosestResult res_left = closest_util(px, py_left, left, mid);
    ClosestResult res_right = closest_util(px, py_right, mid + 1, right);
    ClosestResult best = (res_left.dist <= res_right.dist) ? res_left : res_right;

    std::vector<Point> strip;
    strip.reserve(py.size());
    for (const auto& pt : py) {
        if (std::abs(pt.x - mid_pt.x) < best.dist) {
            strip.push_back(pt);
        }
    }

    return strip_closest(strip, best);
}

ClosestResult find_closest_pair(std::vector<Point> points) {
    if (points.size() < 2) {
        return {std::numeric_limits<double>::infinity(), {0.0, 0.0}, {0.0, 0.0}};
    }

    std::vector<Point> px = points;
    std::sort(px.begin(), px.end(), [](const Point& a, const Point& b) {
        if (a.x != b.x) return a.x < b.x;
        return a.y < b.y;
    });

    for (size_t i = 1; i < px.size(); ++i) {
        if (px[i].x == px[i - 1].x && px[i].y == px[i - 1].y) {
            return {0.0, px[i - 1], px[i]};
        }
    }

    std::vector<Point> py = px;
    std::sort(py.begin(), py.end(), [](const Point& a, const Point& b) {
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    });

    return closest_util(px, py, 0, static_cast<int>(px.size()) - 1);
}

int main() {
    int n;
    if (!(std::cin >> n)) {
        return 0;
    }

    std::vector<Point> points(n);
    for (int i = 0; i < n; ++i) {
        std::cin >> points[i].x >> points[i].y;
    }

    ClosestResult ans = find_closest_pair(points);
    if (!std::isfinite(ans.dist)) {
        std::cout << "Need at least 2 points.\n";
        return 0;
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Closest distance: " << ans.dist << "\n";
    std::cout << "Point 1: (" << ans.p1.x << ", " << ans.p1.y << ")\n";
    std::cout << "Point 2: (" << ans.p2.x << ", " << ans.p2.y << ")\n";
    return 0;
}
