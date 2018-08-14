using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        public Rect BoundsRect;
        public List<int> HullIdxs;
        private readonly Vector2 m_Shift; //avoid floating point zero comparisons

        public float MinFloatingPointErr;
        private readonly float n;
        public List<Vector2> Points;
        public List<Triangle> Triangles;

        private Delaunay(List<Vector2> _points)
        {
            Points = _points;
            Triangles = new List<Triangle>();
            var rnd = new Random(_points.Count);
            while (r11 == 0)
                r11 = (float) rnd.NextDouble();
            while (r12 == 0)
                r12 = (float) rnd.NextDouble();
            while (r21 == 0)
                r21 = (float) rnd.NextDouble();
            while (r22 == 0)
                r22 = (float) rnd.NextDouble();

            for (var pIdx = 0; pIdx < Points.Count; ++pIdx)
                BoundsRect.Encapsulate(Points[pIdx]);
            var w = BoundsRect.width;
            var h = BoundsRect.height;
            //var tmp = (float) Math.Sqrt(Points.Count);
            //n = (float) (Math.Sqrt(w * w + h * h) / 1000f) / tmp;  //TODO research best choice for n
            //m_MinError = 0.00001f; //TODO


            //Determine minimum floating point error and degenerate transform scalar
            var min = w < h ? w : h;
            MinFloatingPointErr = min / Points.Count * 0.01f;
            n = MinFloatingPointErr;

            //Shift points to avoid near zero floating point
            var rectMin = BoundsRect.min;
            m_Shift = Vector2.zero - rectMin + Vector2.one;

            for (var pIdx = 0; pIdx < Points.Count; ++pIdx)
                Points[pIdx] += m_Shift;

            BoundsRect.position += m_Shift;
        }

        public Mesh Mesh
        {
            get
            {
                var triIdxs = new int[Triangles.Count * 3];
                for (var tIdx = 0; tIdx < Triangles.Count; ++tIdx)
                for (var vIdx = 0; vIdx < 3; ++vIdx)
                    triIdxs[tIdx * 3 + vIdx] = Triangles[tIdx].Edge(vIdx).OriginIdx;
                return new Mesh(Points.ToArray(), triIdxs);
            }
        }

        public Triangulator triangulator { get; private set; }


        public static Delaunay Create<T>(List<Vector2> _points) where T : Triangulator, new()
        {
            var del = new Delaunay(_points);
            del.triangulator = new T();
            ((ITriangulator) del.triangulator).SetTarget(del);
            return del;
        }

        public void ReTriangulate(IEnumerable<Vector2> _points)
        {
            Points.Clear();
            Triangles.Clear();
            BoundsRect = Rect.zero;

            AddVertices(_points);

            Triangulate();
        }

        public void AddVertex(Vector2 _vertex)
        {
            Points.Add(_vertex);
            //m_PolysContainingVert.Add(new HashSet<Guid>());
            if (BoundsRect == Rect.zero)
                BoundsRect = new Rect(_vertex, Vector2.zero);
            BoundsRect.Encapsulate(_vertex);
        }

        public void AddVertices(IEnumerable<Vector2> _vertices)
        {
            foreach (var vert in _vertices)
                AddVertex(vert);
        }


        public Triangle AddTriToMesh(Triangle.HalfEdge _twinLt, Triangle.HalfEdge _twinRt)
        {
            //Verify validity - TODO remove optimization
            /*
            if (_twinLt.NextEdge.OriginIdx != _twinRt.OriginIdx)
                throw new Exception("AddTriToMesh - twins arent touching");
            */
            return new Triangle(_twinLt, _twinRt, this);
        }

        public void Triangulate()
        {
            triangulator.Triangulate();
        }

        public class Triangle
        {
            public float CCRSq;

            public float CCX;
            public float CCY;

            public Delaunay D;
            public HalfEdge Edge0;
            public HalfEdge Edge1;
            public HalfEdge Edge2;

            public Triangle(HalfEdge _twinLt, HalfEdge _twinRt, Delaunay _d)
            {
                D = _d;
                var v0 = _twinLt.OriginIdx;
                var v1 = _twinRt.NextEdge.OriginIdx;
                var v2 = _twinRt.OriginIdx;

                Edge0 = new HalfEdge(this, v0, _d);
                Edge1 = new HalfEdge(this, v1, _d);
                Edge2 = new HalfEdge(this, v2, _d);

                Edge0.NextEdge = Edge1;
                Edge1.NextEdge = Edge2;
                Edge2.NextEdge = Edge0;

                Edge1.m_Twin = _twinRt;
                _twinRt.m_Twin = Edge1;

                Edge2.m_Twin = _twinLt;
                _twinLt.m_Twin = Edge2;

                var pts = _d.Points;

                Geom.Circumcircle(pts[v0], pts[v1], pts[v2], out CCX, out CCY, out CCRSq);


                _d.Triangles.Add(this);
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (v0 == v1 || v0 == v2 || v1 == v2)
                //    throw new Exception("new Triangle - Dupe Verts");
            }

            public Triangle(HalfEdge _twin, int _newVert, Delaunay _d)
            {
                D = _d;
                var v0 = _twin.OriginIdx;
                var v1 = _newVert;
                var v2 = _twin.NextEdge.OriginIdx;

                Edge0 = new HalfEdge(this, v0, D);
                Edge1 = new HalfEdge(this, v1, D);
                Edge2 = new HalfEdge(this, v2, D);

                Edge0.NextEdge = Edge1;
                Edge1.NextEdge = Edge2;
                Edge2.NextEdge = Edge0;

                Edge2.m_Twin = _twin;
                _twin.m_Twin = Edge2;

                var pts = D.Points;

                Geom.Circumcircle(pts[v0], pts[v1], pts[v2], out CCX, out CCY, out CCRSq);

                _d.Triangles.Add(this);

                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (v0 == v1 || v0 == v2 || v1 == v2)
                //    throw new Exception("new Triangle - Dupe Verts");
            }

            public Triangle(int _v0, int _v1, int _v2, Delaunay _d)
            {
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (_vertIdx0 == _vertIdx1 || _vertIdx0 == _vertIdx2 || _vertIdx1 == _vertIdx2)
                //    throw new Exception("new Triangle - Dupe Verts");

                D = _d;

                Edge0 = new HalfEdge(this, _v0, D);
                Edge1 = new HalfEdge(this, _v1, D);
                Edge2 = new HalfEdge(this, _v2, D);

                Edge0.NextEdge = Edge1;
                Edge1.NextEdge = Edge2;
                Edge2.NextEdge = Edge0;

                var pts = D.Points;

                Geom.Circumcircle(pts[_v0], pts[_v1], pts[_v2], out CCX, out CCY, out CCRSq);


                _d.Triangles.Add(this);
                //Edges = new[] { edge0, edge1, edge2 };
            }

            public HalfEdge Edge(int _idx)
            {
                if (_idx == 0)
                    return Edge0;
                if (_idx == 1)
                    return Edge1;
                return Edge2;
            }

            /// <summary>
            ///     Calculate center and radius of circumcircle of this triangle.
            /// </summary>
            /// <param name="_center">Populated with center coord of circumcircle</param>
            /// <param name="_r">Populated with radius squared of circumcircle</param>
            public void CircumCircle(out Vector2 _center, out float _rSqr)
            {
                Geom.Circumcircle(Edge0.OriginPos, Edge1.OriginPos, Edge2.OriginPos, out _center, out _rSqr);
            }

            public class HalfEdge
            {
                public Delaunay D;

                public HalfEdge m_Twin; //TODO
                public HalfEdge NextEdge;
                public int OriginIdx;
                public Triangle Triangle;

                public HalfEdge(Triangle _triangle, int _originIdx, Delaunay _d)
                {
                    D = _d;
                    Triangle = _triangle;
                    OriginIdx = _originIdx;
                }

                public Vector2 OriginPos => D.Points[OriginIdx];

                public Vector2 AsVector
                {
                    get
                    {
                        if (NextEdge == null) return Vector2.zero;
                        return NextEdge.OriginPos - OriginPos;
                    }
                }

                public void SetTwin(HalfEdge _twin)
                {
                    var newTwin = _twin;
                    if (m_Twin != null)
                        m_Twin.m_Twin = null;
                    m_Twin = newTwin;
                    if (newTwin != null)
                        newTwin.m_Twin = this;
                }
            }
        }

        private interface ITriangulator
        {
            void SetTarget(Delaunay _d);
        }

        public abstract class Triangulator : ITriangulator
        {
            protected Delaunay D;

            void ITriangulator.SetTarget(Delaunay _d)
            {
                D = _d;
            }

            public void Triangulate()
            {
                Algorithm();
                Hull();
            }

            /// <summary>
            ///     This is where HullIdxs should be populateds
            /// </summary>
            protected abstract void Hull();

            /// <summary>
            ///     This is where the triangulation algorithm to populate polys.
            /// </summary>
            protected abstract void Algorithm();
        }
    }

    /// <summary>
    ///     Comparer for comparing two keys, handling equality as beeing greater
    ///     Use this Comparer e.g. with SortedLists or SortedDictionaries, that don't allow duplicate keys
    /// </summary>
    /// <typeparam name="TKey"></typeparam>
    internal class DuplicateKeyComparer<TKey> : IComparer<TKey> where TKey : IComparable
    {
        public int Compare(TKey _x, TKey _y)
        {
            var result = _x.CompareTo(_y);

            return result == 0 ? 1 : result;
        }
    }

    public static class Ext
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleCW(this Vector2 _from, Vector2 _to)
        {
            var signedAngleRad = Math.Atan2(_to.y, _to.x) - Math.Atan2(_from.y, _from.x);
            return signedAngleRad >= 0 ? (float) (2d * Math.PI - signedAngleRad) : (float) -signedAngleRad;
        }


        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float SqrMagFast(this Vector2 _a, float _bX, float _bY)
        {
            var x = _bX - _a.x;
            var y = _bY - _a.y;
            return x * x + y * y;
        }
    }
}