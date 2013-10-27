#if defined(UNIT_TEST)
#define BOOST_TEST_MODULE beziarcs
#define BOOST_TEST_DYN_LINK
#endif

#include <iostream>
#include <array>
#include <cmath>

#if defined(UNIT_TEST)
#include <boost/test/unit_test.hpp>
#endif

const double EPSILON = 1e-12;

class Point {
    double x;
    double y;

    friend class Segment;

public:
    Point(double x_, double y_) : x(x_), y(y_) {}

    double dot(Point &other) {
        return x*other.x + y*other.y;
    }

    double cross(Point &other);
    double normalize() const;

    Point operator- (Point &other) const {
        return Point(x-other.x, y-other.y);
    }

    Point operator+ (const Point &other) const {
        return Point(x+other.x, y+other.y);
    }

    Point operator* (double scale) const {
        return Point(x*scale, y*scale);
    }

    friend std::ostream& operator<< (std::ostream &out, const Point &p) {
        out << "(" << p.x << ", " << p.y << ")";
        return out;
    }

#if defined(UNIT_TEST)
    bool operator== (const Point &other) const {
        BOOST_CHECK_CLOSE(x, other.x, EPSILON);
        BOOST_CHECK_CLOSE(y, other.y, EPSILON);
        return true;
    }
#endif
};

class Segment {
    Point s;
    Point e;

public:
    Segment(const Point &s_, const Point &e_) : s(s_), e(e_) {}

    double dist_to(const Point &p) {
        /* point-to-line dist */
        double num = std::abs((e.x-s.x)*(s.y-p.y) - (s.x-p.x)*(e.y-s.y));
        double den = std::sqrt(std::pow(e.x-s.x, 2) + std::pow(e.y-s.y, 2));
        return num / den;
    }

    Point operator() (double t) const {
        /* linear interpolation */
        return (s+e)*t;
    }
};

class CubicBezier {
    std::array<Point, 4> ctrl;

public:
    CubicBezier(std::array<Point, 4> &ctrl_) : ctrl(ctrl_) {}

    double flatness() const {
        Segment chord(ctrl[0], ctrl[3]);
        return std::max(chord.dist_to(ctrl[1]), chord.dist_to(ctrl[2]));
    }
};

#if defined(UNIT_TEST)
BOOST_AUTO_TEST_CASE(dist_to) {
    BOOST_CHECK_CLOSE(Segment(Point(3, 4), Point(9, 9)).dist_to(Point(5, 5)),
                      (double) 4 / std::sqrt(61), EPSILON);
}

BOOST_AUTO_TEST_CASE(segment_interp) {
    Segment s(Point(3, 4), Point(9, 9));
    BOOST_CHECK(s(0.5) == Point(6, 6.5));
}
#else
int main() {
    Point x(3, 4);
    Point y(9, 9);
    std::cout << (x + y) << std::endl;
    std::cout << Segment(x, y).dist_to(Point(5, 5)) << std::endl;
    return 0;
}
#endif
