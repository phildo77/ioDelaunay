using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using ioSS.Util;
using ioSS.Util.Maths;
using ioSS.Util.Maths.Geometry;

namespace ioSS.Delaunay
{
    public partial class Delaunay
    {
        #region Constructors

        private Delaunay(Vector2[] _points)
        {
            //Set up random linear tranform coefficients for co-circular / degenerate case resolution.
            var rnd = new Random();
            while (r11 == 0)
                r11 = (float) rnd.NextDouble();
            while (r12 == 0)
                r12 = (float) rnd.NextDouble();
            while (r21 == 0)
                r21 = (float) rnd.NextDouble();
            while (r22 == 0)
                r22 = (float) rnd.NextDouble();

            Points = _points;

            Prog = new Progress();
        }

        #endregion Constructors

        #region Nested Interfaces

        private interface ITriangulator
        {
            #region Methods

            void SetTarget(Delaunay _d);

            #endregion Methods
        }

        #endregion Nested Interfaces

        #region Fields

        /// <summary>
        ///     Outer bounds of the Delaunay graph
        /// </summary>
        public Rect BoundsRect;

        /// <summary>
        ///     Contains outer hull edges in CW order upon completion of triangulation
        /// </summary>
        public List<Triangle.HalfEdge> HullEdges = new List<Triangle.HalfEdge>();

        /// <summary>
        ///     Reference to last triangle created.
        /// </summary>
        public Triangle LastTri;

        /// <summary>
        ///     Allowed error for floating point equality checks.
        /// </summary>
        public float MinFloatingPointErr;

        /// <summary>
        ///     List of points to triangulate
        /// </summary>
        public Vector2[] Points;

        private int m_TriCount;

        private Vector2 m_Shift = Vector2.zero; //avoid floating point zero comparisons
        private float n; //Linear transform coefficient n

        public Progress Prog;

        #endregion Fields

        #region Properties

        /// <summary>
        ///     Generate and return mesh
        /// </summary>
        public Mesh Mesh
        {
            get
            {
                var tris = Triangles();
                var triIdxs = new int[tris.Length * 3];
                for (var tIdx = 0; tIdx < tris.Length; ++tIdx)
                {
                    var tri = tris[tIdx];
                    triIdxs[tIdx * 3] = tri.Edge0.OriginIdx;
                    triIdxs[tIdx * 3 + 1] = tri.Edge1.OriginIdx;
                    triIdxs[tIdx * 3 + 2] = tri.Edge2.OriginIdx;
                }

                return new Mesh(Points, triIdxs);
            }
        }

        /// <summary>
        ///     Access to triangulator
        /// </summary>
        public Triangulator triangulator { get; private set; }

        #endregion Properties

        #region Methods

        /// <summary>
        ///     Create new Delaunay object
        /// </summary>
        /// <param name="_points">Points to be triangulated</param>
        /// <typeparam name="T">Type of triangulator to use for triangulation</typeparam>
        /// <returns>Delaunay object ready for triangulation</returns>
        public static Delaunay Create<T>(Vector2[] _points)
            where T : Triangulator, new()
        {
            var del = new Delaunay(_points);
            del.triangulator = new T();
            ((ITriangulator) del.triangulator).SetTarget(del);
            return del;
        }

        /// <summary>
        ///     Create / return array containing all triangles
        /// </summary>
        /// <returns></returns>
        public Triangle[] Triangles()
        {
            var triangles = new Triangle[m_TriCount];
            var curTri = LastTri;
            for (var tIdx = 0; tIdx < m_TriCount; ++tIdx)
            {
                triangles[tIdx] = curTri;
                curTri = curTri.PrevTri;
            }

            return triangles;
        }

        /// <summary>
        ///     Perform delaunay triangulation on specified list of points
        /// </summary>
        /// <param name="_points">Points to triangulate</param>
        public void Triangulate(Vector2[] _points)
        {
            Points = _points;
            Triangulate();
        }

        /// <summary>
        ///     Perform delaunay triangulation
        /// </summary>
        public void Triangulate()
        {
            Prog.Update(0, "Triangulate Init...");
            ClearTris();
            BoundsRect = new Rect(Rect.zero);

            for (var pIdx = 0; pIdx < Points.Length; ++pIdx)
                BoundsRect.Encapsulate(Points[pIdx]);

            //calculate relative acceptable error and shift away from zero for triangulation
            var w = BoundsRect.width;
            var h = BoundsRect.height;

            //Determine minimum floating point error and degenerate transform scalar
            var min = w < h ? w : h;
            MinFloatingPointErr = min / Points.Length * 0.01f; //TODO research best choice for MinFloatingPointErr
            n = MinFloatingPointErr; //TODO research best choice for n

            //Shift points to avoid near zero floating point
            var rectMin = BoundsRect.min;

            var doShift = !(rectMin.x > 0 && rectMin.y > 0);

            if (doShift)
            {
                m_Shift = Vector2.zero - rectMin + Vector2.one;

                for (var pIdx = 0; pIdx < Points.Length; ++pIdx)
                    Points[pIdx] += m_Shift;

                BoundsRect.position += m_Shift;
            }
            else
            {
                m_Shift = Vector2.zero;
            }

            //Triangulate
            triangulator.Triangulate();

            //Shift back
            if (doShift)
            {
                for (var pIdx = 0; pIdx < Points.Length; ++pIdx)
                    Points[pIdx] -= m_Shift;

                BoundsRect.position -= m_Shift;

                var triScan = LastTri;
                while (triScan != null)
                {
                    triScan.CCX -= m_Shift.x;
                    triScan.CCY -= m_Shift.y;
                    triScan = triScan.PrevTri;
                }
            }
        }

        /// <summary>
        ///     Clear triangle references for GC
        /// </summary>
        private void ClearTris()
        {
            m_TriCount = 0;
            if (LastTri == null) return;
            var triScan = LastTri;
            do
            {
                var triNext = triScan.PrevTri;
                triScan.D = null;
                triScan.Edge0 = null;
                triScan.Edge1 = null;
                triScan.Edge2 = null;
                triScan.PrevTri = null;
                triScan = triNext;
            } while (triScan != null);

            LastTri = null;
        }

        #endregion Methods


        #region Nested Types

        public class Triangle
        {
            #region Methods

            /// <summary>
            ///     Returns edge by index - convenience function
            /// </summary>
            /// <param name="_idx"></param>
            /// <returns>selected edge</returns>
            public HalfEdge Edge(int _idx)
            {
                if (_idx < 0 || _idx > 2) return null;
                if (_idx == 0)
                    return Edge0;
                if (_idx == 1)
                    return Edge1;
                return Edge2;
            }

            #endregion Methods

            #region Nested Types

            public class HalfEdge
            {
                #region Constructors

                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public HalfEdge(Triangle _triangle, int _originIdx, Delaunay _d)
                {
                    D = _d;
                    Triangle = _triangle;
                    OriginIdx = _originIdx;
                }

                #endregion Constructors

                #region Fields

                public Delaunay D;
                public HalfEdge NextEdge;
                public int OriginIdx;
                public Vector2 OriginPos => D.Points[OriginIdx];
                public Triangle Triangle;
                public HalfEdge Twin;

                #endregion Fields
            }

            #endregion Nested Types

            #region Fields

            public float CCRSq;
            public float CCX;
            public float CCY;
            public Delaunay D;
            public HalfEdge Edge0;
            public HalfEdge Edge1;
            public HalfEdge Edge2;
            public Triangle PrevTri;

            #endregion Fields

            #region Constructors

            /// <summary>
            ///     Add triangle to triangulation connecting to two existing edges.  Clockwise
            /// </summary>
            /// <param name="_twinLt">Connecting twin left</param>
            /// <param name="_twinRt">Connecting twin right</param>
            /// <param name="_d">Delaunay parent object</param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

                Edge1.Twin = _twinRt;
                _twinRt.Twin = Edge1;

                Edge2.Twin = _twinLt;
                _twinLt.Twin = Edge2;

                var pts = _d.Points;

                Geom.Circumcircle(pts[v0], pts[v1], pts[v2], out CCX, out CCY, out CCRSq);

                PrevTri = _d.LastTri;
                _d.LastTri = this;

                _d.m_TriCount++;
            }

            /// <summary>
            ///     Add triangle to triangulation connecting to a single edge and a new vertex index.  Clockwise
            /// </summary>
            /// <param name="_twin">existing twin edge to connect</param>
            /// <param name="_newVert">new vertex index</param>
            /// <param name="_d">Delaunay parent object</param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
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

                Edge2.Twin = _twin;
                _twin.Twin = Edge2;

                var pts = D.Points;

                Geom.Circumcircle(pts[v0], pts[v1], pts[v2], out CCX, out CCY, out CCRSq);

                PrevTri = _d.LastTri;
                _d.LastTri = this;

                _d.m_TriCount++;
            }

            /// <summary>
            ///     Create new triangle from three vertex indexes.
            ///     Warning does not check for clockwise or colinear - expects clockwise and not colinear
            /// </summary>
            /// <param name="_v0">Vert index 0</param>
            /// <param name="_v1">Vert index 1</param>
            /// <param name="_v2">Vert index 2</param>
            /// <param name="_d">Delaunay parent objecct</param>
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public Triangle(int _v0, int _v1, int _v2, Delaunay _d)
            {
                D = _d;

                Edge0 = new HalfEdge(this, _v0, D);
                Edge1 = new HalfEdge(this, _v1, D);
                Edge2 = new HalfEdge(this, _v2, D);

                Edge0.NextEdge = Edge1;
                Edge1.NextEdge = Edge2;
                Edge2.NextEdge = Edge0;

                var pts = D.Points;

                Geom.Circumcircle(pts[_v0], pts[_v1], pts[_v2], out CCX, out CCY, out CCRSq);
                PrevTri = _d.LastTri;
                _d.LastTri = this;
                _d.m_TriCount++;
            }

            #endregion Constructors
        }

        public abstract class Triangulator : ITriangulator
        {
            #region Fields

            protected Delaunay D;

            #endregion Fields

            #region Methods

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
            ///     This is where the triangulation algorithm to populate triangles is defined.
            /// </summary>
            protected abstract void Algorithm();

            /// <summary>
            ///     This is where HullEdges should be populated
            /// </summary>
            protected abstract void Hull();

            #endregion Methods
        }

        #endregion Nested Types
    }

    public static class Ext
    {
        #region Methods

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

        #endregion Methods
    }
}