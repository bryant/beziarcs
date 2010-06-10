from operator import mul
from math import sqrt
from itertools import chain

EPSILON = 10e-6

class Point(object):
    def __init__(self, x, y):
        self.x, self.y = float(x), float(y)
        self.normsq = float(x*x + y*y)

    def __mul__(self, scalar):
        return Point(self.x*scalar, self.y*scalar)

    __rmul__ = __mul__

    def __add__(self, p):
        if type(p) != Point:
            return self

        return Point(self.x+p.x, self.y+p.y)

    __radd__ = __add__

    def __sub__(self, p):
        if type(p) != Point:
            return self
        return Point(self.x-p.x, self.y-p.y)

    def __repr__(self):
        return "<Point @ %f, %f>" % (self.x, self.y)

    def dot(self, p):
        return self.x*p.x + self.y*p.y

    def cross(self, p):
        return self.x*p.y - self.y*p.x

    def normalize(self):
        dist = sqrt(self.normsq)
        return Point(self.x/dist, self.y/dist)

class Arc(object):
    svg_entity = """\
<path fill="none" stroke="{color}" strokewidth="1" \
d="M{s.x},{s.y} A{radius},{radius} 0 {large},{ccw} {e.x},{e.y}" />
"""
    def __init__(self, radius, center, start, end, large, ccw):
        self.center = center
        self.radius, self.start, self.end = radius, start, end
        self.large = large
        self.ccw = ccw

    def to_svg(self, color="#000"):
        return self.svg_entity.format(color=color, s=self.start, e=self.end,
                                      ccw=self.ccw, radius=self.radius,
                                      large=self.large)

    @classmethod
    def from_isosceles(cls, start, apex, end, large=0):
        leg0 = apex - start
        leg1 = end - apex
        cos2p = leg0.dot(leg1) / leg0.normsq

        denom = 1 + cos2p

        if abs(denom) < EPSILON:
            radius = center = None

        else:
            tanp = sqrt((1 - cos2p) / denom)
            radius = sqrt(leg0.normsq) / tanp
            center = apex + (leg1 - leg0).normalize() * (radius/sqrt(denom*0.5))

        ccw = 1 if leg0.cross(leg1) >= 0 else 0
        ccw ^= large

        return cls(radius, center, start, end, large, ccw)

class Bezier(object):
    svg_entity = """\
<path fill="none" stroke="{color}" strokewidth="1" \
d="M{s.x},{s.y} C{c1.x},{c1.y} {c2.x},{c2.y} {e.x},{e.y}" />"""

    def __init__(self, start, c1, c2, end):
        self.s, self.c1, self.c2, self.e = start, c1, c2, end

    def point_at(self, t):
        s = 1 - t
        poly = [s*s*s, 3*s*s*t, 3*s*t*t, t*t*t]
        coeff = [self.s, self.c1, self.c2, self.e]

        return sum(map(mul, coeff, poly))

    @property
    def flatness(self):
        return max(point2line(self.c1, self.s, self.e),
                   point2line(self.c2, self.s, self.e))

    def split(self, t=0.5):
        bez1c1 = interp_seg(self.s, self.c1, t)
        bez2c2 = interp_seg(self.c2, self.e, t)
        mid = interp_seg(self.c1, self.c2, t)
        bez1c2 = interp_seg(bez1c1, mid, t)
        bez2c1 = interp_seg(mid, bez2c2, t)
        j = interp_seg(bez1c2, bez2c1, t)

        return Bezier(self.s, bez1c1, bez1c2, j), \
               Bezier(j, bez2c1, bez2c2, self.e)

    def to_svg(self, color="#000"):
        return self.svg_entity.format(color=color, s=self.s,
                                      c1=self.c1, c2=self.c2,
                                      e=self.e)

    def __repr__(self):
        return "<Bezier %r, %r, %r, %r>" % (self.s, self.c1,
                                            self.c2, self.e)

def interp_seg(s, e, t):
    return Point(t*(s.x+e.x), t*(s.y+e.y))

def point2line(p, ls, le):
    n = abs(float(le.x-ls.x) * float(ls.y-p.y) - \
        float(ls.x-p.x) * float(le.y-ls.y))

    d = sqrt((le.x-ls.x)**2 + (le.y-ls.y)**2)

    return n / d

def flatten(b, err, s=0.0, e=1.0):
    if b.flatness < err:
        return [(b, s, e)]

    a, c = b.split()
    return flatten(a, err, s, 0.5*(e-s)+s) + flatten(c, err, e-0.5*(e-s), e)

def collect2(beziers):
    points = [beziers[0].s, beziers[0].c1, beziers[0].c2, beziers[0].e]

    for bez in beziers[1:]:
        points += [bez.c1, bez.c2, bez.e]

    return points

def collect(beziers):
    return [bezier.s for bezier, s, e in beziers] + [beziers[-1][0].e]

def biarc_quadratic(v, uts, ute):
    a = 2 * (uts.dot(ute) - 1)
    b = 2 * v.dot(uts+ute)
    c = v.dot(v)
    discrim = b**2 - 4*a*c

    print discrim, a, b, c

    if abs(a) < EPSILON:
        if abs(b) < EPSILON:
            return None
        else:
            return - c / b

    # root1 > root0 always
    # root0 = (-b + sqrt(discrim)) / (a + a)
    root1 = 0.5 * (-b - sqrt(discrim)) / a

    return root1

def biarc(p0, t0, p1, t1):
    uts = t0.normalize()
    ute = t1.normalize()
    v = p0 - p1
    alpha = biarc_quadratic(v, uts, ute)

    if alpha == None:
        return None, None

    apex0 = (uts * alpha) + p0
    apex1 = p1 - (ute * alpha)
    joint = 0.5 * (apex0 + apex1)

    #return Arc.from_isosceles(p0, apex0, joint), \
    #       Arc.from_isosceles(joint, apex1, p1)

    return p0, apex0, joint, apex1, p1, alpha

def draw_line(pointlist, color="black"):
    svg_entity = """<path fill="none" strokewidth="1" stroke="{color}" \
d="M{s.x},{s.y} L{e.x},{e.y} {tail}" />"""

    if len(pointlist) < 2:
        return ""

    tail = " ".join("{p.x},{p.y}".format(p=p) for p in pointlist[2:])
    return svg_entity.format(s=pointlist[0], e=pointlist[1], tail=tail, \
                             color=color)

def in_sweep(arc, p):
    bound0 = arc.start - arc.center
    bound1 = arc.end - arc.center
    pv = p - arc.center

    dot01 = bound0.dot(bound1)
    dot02 = bound0.dot(pv)
    dot12 = bound1.dot(pv)
    denom = bound0.normsq * bound1.normsq - dot01 * dot01
    u = (bound1.normsq * dot02 - dot01 * dot12) / denom
    v = (bound0.normsq * dot12 - dot01 * dot02) / denom

    return u, v

def between(m, p0, p1):
    """Assuming that m, p0, and p1 are colinear,
    is m between the other two?"""

    if p0.x != p1.x:
        return (p0.x <= m.x <= p1.x) or (p1.x <= m.x <= p0.x)
    else:
        return (p0.y <= m.y <= p1.y) or (p1.y <= m.y <= p0.y)

def intersect(a0, b0, a1, b1):
    """Given two segments (a0, b0) and (a1, b1),
    obtain the point of intersection"""

    x1, x2, x3, x4 = a0.x, b0.x, a1.x, b1.x
    y1, y2, y3, y4 = a0.y, b0.y, a1.y, b1.y

    denom = (x1-x2) * (y3-y4) - (y1-y2) * (x3-x4)

    if abs(denom) < EPSILON:
        return None, None

    detA = x1*y2 - y1*x2
    detB = x3*y4 - y3*x4
    x = (detA * (x3-x4) - detB * (x1-x2)) / denom
    y = (detA * (y3-y4) - detB * (y1-y2)) / denom

    return x, y

def line_arc_dist(line, arc):
    pass

b = Bezier
p = Point

if __name__ == "__main__":
    from pprint import pprint

    c = b(p(2,2),p(600,20),p(50,600),p(900,70))

    print c.point_at(0)

    print point2line(p(4,4), p(-1,1), p(1,-1))

    a, b = c.split()

    f = open("raf.svg", "w")
    f.write("""<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" width="1000" height="1000">""")
    f.write(c.to_svg())
    flattened = flatten(c, 50)
    segments = collect(flattened)
    print len(segments), "segments"

    #pprint(flattened)

    colors = ["#ff0000", "#00ff00", "#0000ff"]

    for c, i in enumerate(segments[:-1]):
        f.write("""<path fill="none" strokewidth="1" stroke="{color}" d="M{s.x},{s.y} L{e.x},{e.y}" />
""".format(color=colors[c % 3], s=i, e=segments[c+1]))

    #f.write(draw_line(p(0,0), p(200,400), p(400, 0)))
    #f.write(Arc.from_isosceles(Point(0, 0), Point(200, 400), Point(400, 0)).to_svg())

    biarcpts = biarc(p(400, 400), p(-1, -1), p(420, 400), p(-1, 0))
    print biarcpts

    f.write(draw_line(biarcpts[:3]))
    f.write(draw_line(biarcpts[2:5], color="red"))

    large = 0 if biarcpts[-1] >= 0 else 1
    print "large", large
    a = Arc.from_isosceles(*biarcpts[:3], large=large)
    f.write(draw_line((a.center, a.start)))
    f.write(a.to_svg())
    f.write(Arc.from_isosceles(*biarcpts[2:5], large=large).to_svg(color="red"))

    f.write("</svg>")
    f.close()

    start = Point(40, 40)
    end = Point(200, 200)
    # test_biarc()
