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
    }
}