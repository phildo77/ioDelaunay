using System;
using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public static class Geom
    {
        public static bool Circumcircle(Vector2f p0, Vector2f p1, Vector2f p2, out Vector2f center, out float radius)
        {
            float dA, dB, dC, aux1, aux2, div;

            dA = p0.x * p0.x + p0.y * p0.y;
            dB = p1.x * p1.x + p1.y * p1.y;
            dC = p2.x * p2.x + p2.y * p2.y;

            aux1 = dA * (p2.y - p1.y) + dB * (p0.y - p2.y) + dC * (p1.y - p0.y);
            aux2 = -(dA * (p2.x - p1.x) + dB * (p0.x - p2.x) + dC * (p1.x - p0.x));
            div = 2 * (p0.x * (p2.y - p1.y) + p1.x * (p0.y - p2.y) + p2.x * (p1.y - p0.y));

            if (div == 0)
            {
                center = new Vector2f(float.NaN, float.NaN);
                radius = float.NaN;
                return false;
            }

            center.x = aux1 / div;
            center.y = aux2 / div;

            radius = (float) Math.Sqrt((center.x - p0.x) * (center.x - p0.x) + (center.y - p0.y) * (center.y - p0.y));

            return true;
        }

        public static bool IsValidDelTri(Vector2f _tA0, Vector2f _tAB1, Vector2f _tAB2,
            Vector2f _tB0)
        {
            Vector2f centerA;
            float centerARad;
            Circumcircle(_tA0, _tAB1, _tAB2, out centerA, out centerARad);
            if (float.IsNaN(centerARad))
                return false;

            var distSqr = (centerA - _tB0).sqrMagnitude;
            return distSqr > centerARad * centerARad;
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