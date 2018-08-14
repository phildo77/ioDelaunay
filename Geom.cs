using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;

namespace ioDelaunay
{
    public static class Geom
    {
        public static bool Circumcircle2(Vector2 p0, Vector2 p1, Vector2 p2, out Vector2 center,
            out float radiusSqr)
        {
            double dp0x = p0.x;
            double dp0y = p0.y;
            double dp1x = p1.x;
            double dp1y = p1.y;
            double dp2x = p2.x;
            double dp2y = p2.y;
            var det = (dp0x - dp2x) * (dp1y - dp2y) - (dp1x - dp2x) * (dp0y - dp2y);
            if (det == 0) //TODO use epsilon / approx
            {
                center = new Vector2(float.NaN, float.NaN);
                radiusSqr = float.NaN;
                return false;
            }

            var cent_x = (((dp0x - dp2x) * (dp0x + dp2x) + (dp0y - dp2y) * (dp0y + dp2y)) / 2 * (dp1y - dp2y)
                          - ((dp1x - dp2x) * (dp1x + dp2x) + (dp1y - dp2y) * (dp1y + dp2y)) / 2 * (dp0y - dp2y))
                         / det;

            var cent_y = (((dp1x - dp2x) * (dp1x + dp2x) + (dp1y - dp2y) * (dp1y + dp2y)) / 2 * (dp0x - dp2x)
                          - ((dp0x - dp2x) * (dp0x + dp2x) + (dp0y - dp2y) * (dp0y + dp2y)) / 2 * (dp1x - dp2x))
                         / det;

            center = new Vector2((float) cent_x, (float) cent_y);
            radiusSqr = (float) ((dp2x - cent_x) * (dp2x - cent_x) + (dp2y - cent_y) * (dp2y - cent_y));
            return true;
        }

        public static Vector2 CentroidOfPoly(IEnumerable<Vector2> _pts)
        {
            var count = 0;
            float xSum = 0;
            float ySum = 0;
            foreach (var pt in _pts)
            {
                count++;
                xSum = xSum + pt.x;
                ySum = ySum + pt.y;
            }

            return new Vector2(xSum / count, ySum / count);
            /*
            var count = _pts.Count();
            var x = _pts.Sum(_pt => _pt.x) / count;
            var y = _pts.Sum(_pt => _pt.y) / count;
            return new Vector2f(x, y);
            */
        }


        private static T Min<T>(params T[] _vals)
        {
            return _vals.Min();
        }

        private static T Max<T>(params T[] _vals)
        {
            return _vals.Max();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void Circumcircle(Vector2 a, Vector2 b, Vector2 c,
            out float _centX, out float _centY, out float radiusSqr)
        {
            var A = b.x - a.x;
            var B = b.y - a.y;
            var C = c.x - a.x;
            var D = c.y - a.y;
            var E = A * (a.x + b.x) + B * (a.y + b.y);
            var F = C * (a.x + c.x) + D * (a.y + c.y);
            var G = 2 * (A * (c.y - b.y) - B * (c.x - b.x));

            float minx, miny, dx, dy;

            _centX = (D * E - B * F) / G;
            _centY = (A * F - C * E) / G;
            dx = _centX - a.x;
            dy = _centY - a.y;

            radiusSqr = dx * dx + dy * dy;

            /* If the points of the triangle are collinear, then just find the
             * extremes and use the midpoint as the center of the circumcircle. */

            /*
            if(Math.Abs(G) < 0.000001)
            {
                minx = Min(a.x, b.x, c.x);
                miny = Min(a.y, b.y, c.y);
                dx = (Max(a.x, b.x, c.x) - minx) * 0.5f;
                dy = (Max(a.y, b.y, c.y) - miny) * 0.5f;

                _centX = minx + dx;
                _centY = miny + dy;
                radiusSqr = dx * dx + dy * dy;
            } else {
                _centX = (D * E - B * F) / G;
                _centY = (A * F - C * E) / G;
                dx = _centX - a.x;
                dy = _centY - a.y;

                radiusSqr = dx * dx + dy * dy;
            }

            return true;
            */
        }

        public static bool AreColinear(Vector2 _v0, Vector2 _v1, Vector2 _v2, float _epsilon) //TODO Dynamic Epsilon
        {
            return ((_v0.y - _v1.y) * (_v0.x - _v2.x)).ApproxEqual((_v0.y - _v2.y) * (_v0.x - _v1.x), _epsilon);
        }

        //Don't have to worry about inf or close to zero cases

        public static bool ApproxEqual(this float _a, float _b, float _epsilon)
        {
            return Math.Abs(_a - _b) < _epsilon;
        }


        /*
        public static bool ApproxEqual(this float _a, float _b, float _epsilon)
        {
            float absA = Math.Abs(_a);
            float absB = Math.Abs(_b);
            float diff = Math.Abs(_a - _b);

            if (_a == _b)
            { // shortcut, handles infinitiess
                return true;
            } 
            else if (_a == 0 || _b == 0) 
            {
                // _a or _b is zero or both are extremely close to it
                // relative error is less meaningful here
                return diff < _epsilon;
            }
            else
            { // use relative error
                return diff / (absA + absB) < _epsilon;
            }
        }
        */


        public static bool IsInPolygon(this Vector2 _point, params Vector2[] _poly)
        {
            //public static bool IsInPolygon(Point[] poly, Point point)
            //{
            var coef = _poly.Skip(1).Select((p, i) =>
                    (_point.y - _poly[i].y) * (p.x - _poly[i].x)
                    - (_point.x - _poly[i].x) * (p.y - _poly[i].y))
                .ToList();

            if (coef.Any(p => p == 0))
                return true;

            for (var i = 1; i < coef.Count(); i++)
                if (coef[i] * coef[i - 1] < 0)
                    return false;
            return true;
            //}
        }

        public static float ToDeg(float _radians)
        {
            return _radians * 180f / (float) Math.PI;
        }

        public static bool NearlyEqual(this float a, float b, float epsilon)
        {
            var absA = Math.Abs(a);
            var absB = Math.Abs(b);
            var diff = Math.Abs(a - b);

            if (a == b)
                return true;
            if (a == 0 || b == 0)
                return diff < epsilon;
            return diff / (absA + absB) < epsilon;
        }
    }
}