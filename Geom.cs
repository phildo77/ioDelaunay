using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using Vectorf;

namespace ioDelaunay
{
    public static class Geom
    {
        public static bool Circumcircle2(Vector2f p0, Vector2f p1, Vector2f p2, out Vector2f center, out float radiusSqr)
        {
            double dp0x = p0.x;
            double dp0y = p0.y;
            double dp1x = p1.x;
            double dp1y = p1.y;
            double dp2x = p2.x;
            double dp2y = p2.y;
            double det = (dp0x - dp2x) * (dp1y - dp2y) - (dp1x - dp2x) * (dp0y - dp2y);
            if (det == 0) //TODO use epsilon / approx
            {
                center = new Vector2f(float.NaN, float.NaN);
                radiusSqr = float.NaN;
                return false;
            }
            
            double cent_x = (((dp0x - dp2x) * (dp0x + dp2x) + (dp0y - dp2y) * (dp0y + dp2y)) / 2 * (dp1y - dp2y)
                         - ((dp1x - dp2x) * (dp1x + dp2x) + (dp1y - dp2y) * (dp1y + dp2y)) / 2 * (dp0y - dp2y))
                        / det;

            double cent_y = (((dp1x - dp2x) * (dp1x + dp2x) + (dp1y - dp2y) * (dp1y + dp2y)) / 2 * (dp0x - dp2x)
                         - ((dp0x - dp2x) * (dp0x + dp2x) + (dp0y - dp2y) * (dp0y + dp2y)) / 2 * (dp1x - dp2x))
                        / det;
            
            center = new Vector2f((float)cent_x, (float)cent_y);
            radiusSqr = (float)((dp2x - cent_x) * (dp2x - cent_x) + (dp2y - cent_y) * (dp2y - cent_y));
            return true;
        }
        
        public static Vector2f CentroidOfPoly(IEnumerable<Vector2f> _pts)
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

            return new Vector2f(xSum / count, ySum / count);
            /*
            var count = _pts.Count();
            var x = _pts.Sum(_pt => _pt.x) / count;
            var y = _pts.Sum(_pt => _pt.y) / count;
            return new Vector2f(x, y);
            */
        }
        
        
        private static T Min<T> (params T[] _vals) {
            return _vals.Min();
        }
        private static T Max<T> (params T[] _vals) {
            return _vals.Max();
        }

        
        public static bool Circumcircle(Vector2f a, Vector2f b, Vector2f c, out Vector2f center, out float radiusSqr)
        {
            var A = b.x - a.x;
            var B = b.y - a.y;
            var C = c.x - a.x;
            var D = c.y - a.y;
            var E = A * (a.x + b.x) + B * (a.y + b.y);
            var F = C * (a.x + c.x) + D * (a.y + c.y);
            var G = 2 * (A * (c.y - b.y) - B * (c.x - b.x));

            float minx, miny, dx, dy;

            /* If the points of the triangle are collinear, then just find the
             * extremes and use the midpoint as the center of the circumcircle. */

            if(Math.Abs(G) < 0.000001)
            {
                minx = Min(a.x, b.x, c.x);
                miny = Min(a.y, b.y, c.y);
                dx = (Max(a.x, b.x, c.x) - minx) * 0.5f;
                dy = (Max(a.y, b.y, c.y) - miny) * 0.5f;

                center = new Vector2f(minx + dx, miny + dy);
                radiusSqr = dx * dx + dy * dy;
            } else {
                var cx = (D * E - B * F) / G;
                var cy = (A * F - C * E) / G;

                center = new Vector2f(cx, cy);
                dx = cx - a.x;
                dy = cy - a.y;

                radiusSqr = dx * dx + dy * dy;
            }

            return true;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static bool Circumcircle(Vector2f a, Vector2f b, Vector2f c, 
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

            /* If the points of the triangle are collinear, then just find the
             * extremes and use the midpoint as the center of the circumcircle. */

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
        }
        
        public static bool AreColinear(Vector2f _v0, Vector2f _v1, Vector2f _v2) //TODO Dynamic Epsilon
        {

            return ((_v0.y - _v1.y) * (_v0.x - _v2.x)).ApproxEqual((_v0.y - _v2.y) * (_v0.x - _v1.x), 1f / 1000000f);
            
        }

        public static bool ApproxEqual(this float _a, float _b, float _epsilon)
        {
            float absA = Math.Abs(_a);
            float absB = Math.Abs(_b);
            float diff = Math.Abs(_a - _b);

            if (_a == _b)
            { // shortcut, handles infinitiess
                return true;
            } 
            else if (_a == 0 || _b == 0 || diff < float.Epsilon) 
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

        public static bool IsInPolygon(this Vector2f _point, params Vector2f[] _poly)
        {
            //public static bool IsInPolygon(Point[] poly, Point point)
            //{
                var coef = _poly.Skip(1).Select((p, i) => 
                        (_point.y - _poly[i].y)*(p.x - _poly[i].x) 
                        - (_point.x - _poly[i].x) * (p.y - _poly[i].y))
                    .ToList();

                if (coef.Any(p => p == 0))
                    return true;

                for (int i = 1; i < coef.Count(); i++)
                {
                    if (coef[i] * coef[i - 1] < 0)
                        return false;
                }
                return true;
            //}
        }

        public static float ToDeg(float _radians)
        {
            return _radians * 180f / (float) Math.PI;
        }

        public static bool NearlyEqual(this float a, float b, float epsilon)
        {
            float absA = Math.Abs(a);
            float absB = Math.Abs(b);
            float diff = Math.Abs(a - b);

            if (a == b)
            { // shortcut, handles infinities
                return true;
            } 
            else if (a == 0 || b == 0) 
            {
                // a or b is zero or both are extremely close to it
                // relative error is less meaningful here
                return diff < epsilon;
            }
            else
            { // use relative error
                return diff / (absA + absB) < epsilon;
            }
        }
    }
}