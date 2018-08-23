using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        public Rect BoundsRect;
        public List<int> HullIdxs;  //TODO REMOVE
        public List<Triangle.HalfEdge> HullEdges;
        private Vector2 m_Shift = Vector2.zero; //avoid floating point zero comparisons

        public float MinFloatingPointErr;
        private float n;
        public List<Vector2> Points;
        //public List<Triangle> Triangles;
        public Triangle LastTri;
        public int TriCount = 0;

        private Delaunay(List<Vector2> _points)
        {
            var rnd = new Random(_points.Count);
            while (r11 == 0)
                r11 = (float) rnd.NextDouble();
            while (r12 == 0)
                r12 = (float) rnd.NextDouble();
            while (r21 == 0)
                r21 = (float) rnd.NextDouble();
            while (r22 == 0)
                r22 = (float) rnd.NextDouble();

            Init(_points);
        }

        private void Init(List<Vector2> _points)
        {
            Points = _points;
            ClearTris();
            BoundsRect = new Rect(Rect.zero);

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

            if (rectMin.x > 0 && rectMin.y > 0) return;
            
            m_Shift = Vector2.zero - rectMin + Vector2.one;

            for (var pIdx = 0; pIdx < Points.Count; ++pIdx)
                Points[pIdx] += m_Shift;

            BoundsRect.position += m_Shift;
        }
        
        public Mesh Mesh
        {
            get
            {
                /*
                var triIdxs = new int[Triangles.Count * 3];
                for (var tIdx = 0; tIdx < Triangles.Count; ++tIdx)
                for (var vIdx = 0; vIdx < 3; ++vIdx)
                    triIdxs[tIdx * 3 + vIdx] = Triangles[tIdx].Edge(vIdx).OriginIdx;
                return new Mesh(Points.ToArray(), triIdxs);
                */
                return null;
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

        private void ClearTris()
        {
            TriCount = 0;
            if (LastTri == null) return;
            var triScan = LastTri;
            while (triScan.PrevTri != null)
            {
                var triNext = triScan.PrevTri;
                triScan.D = null;
                triScan.Edge0 = null;
                triScan.Edge1 = null;
                triScan.Edge2 = null;
                triScan.PrevTri = null;
                triScan = triNext;
            }

        }

        public Triangle[] Triangles()
        {
            var triangles = new Triangle[TriCount];
            var curTri = LastTri;
            for (int tIdx = 0; tIdx < TriCount; ++tIdx)
            {
                triangles[tIdx] = curTri;
                curTri = curTri.PrevTri;
            }

            return triangles;
        }
        
        public void ReTriangulate(List<Vector2> _points)
        {
            Points.Clear();
            ClearTris();

            Init(_points);

            Triangulate();
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
            
            //TriList
            public Triangle PrevTri;
            
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

                Edge1.m_Twin = _twinRt;
                _twinRt.m_Twin = Edge1;

                Edge2.m_Twin = _twinLt;
                _twinLt.m_Twin = Edge2;

                var pts = _d.Points;

                Geom.Circumcircle(pts[v0], pts[v1], pts[v2], out CCX, out CCY, out CCRSq);


                //_d.Triangles.Add(this);
                PrevTri = _d.LastTri;
                _d.LastTri = this;

                _d.TriCount++;

                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (v0 == v1 || v0 == v2 || v1 == v2)
                //    throw new Exception("new Triangle - Dupe Verts");
            }

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

                Edge2.m_Twin = _twin;
                _twin.m_Twin = Edge2;

                var pts = D.Points;

                Geom.Circumcircle(pts[v0], pts[v1], pts[v2], out CCX, out CCY, out CCRSq);

                //_d.Triangles.Add(this);
                PrevTri = _d.LastTri;
                _d.LastTri = this;

                _d.TriCount++;
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (v0 == v1 || v0 == v2 || v1 == v2)
                //    throw new Exception("new Triangle - Dupe Verts");
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
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


                //_d.Triangles.Add(this);
                PrevTri = _d.LastTri;
                _d.LastTri = this;
                _d.TriCount++;
            }

            public HalfEdge Edge(int _idx)
            {
                if (_idx == 0)
                    return Edge0;
                if (_idx == 1)
                    return Edge1;
                return Edge2;
            }


            public class HalfEdge
            {
                public Delaunay D;

                public HalfEdge m_Twin; //TODO
                public HalfEdge NextEdge;
                public int OriginIdx;
                public Triangle Triangle;
                
                [MethodImpl(MethodImplOptions.AggressiveInlining)]
                public HalfEdge(Triangle _triangle, int _originIdx, Delaunay _d)
                {
                    D = _d;
                    Triangle = _triangle;
                    OriginIdx = _originIdx;
                }

                public Vector2 OriginPos => D.Points[OriginIdx];
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