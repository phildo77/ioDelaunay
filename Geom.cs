using System;
using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public static class Geom
    {
        public static bool Circumcircle(Vector2f p0, Vector2f p1, Vector2f p2, out Vector2f center, out float radiusSqr)
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
    }
}