using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Security.Cryptography.X509Certificates;
using ioPolygonGraph;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay 
    {
        public List<int> HullIdxs;
        public List<Vector2f> Points;
        public List<Triangle> Triangles;
        public Rectf BoundsRect;
       
        private Triangulator m_Triangulator;

        public static Delaunay Create<T>(List<Vector2f> _points) where T : Triangulator, new()
        {
            var del = new Delaunay(_points);
            del.m_Triangulator = new T();
            ((ITriangulator)del.m_Triangulator).SetTarget(del);
            return del;
        }

        private Delaunay(List<Vector2f> _points)
        {
            Points = _points;
            Triangles = new List<Triangle>();
            Random rnd = new Random(_points.Count);
            while (r11 == 0)
                r11 = (float) rnd.NextDouble();
            while (r12 == 0)
                r12 = (float) rnd.NextDouble();
            while (r21 == 0)
                r21 = (float) rnd.NextDouble();
            while (r22 == 0)
                r22 = (float) rnd.NextDouble();
            
            for(int pIdx = 0; pIdx < Points.Count; ++pIdx)
                BoundsRect.Encapsulate(Points[pIdx]);
            var w = BoundsRect.width;
            var h = BoundsRect.height;
            var tmp = (float) Math.Sqrt(Points.Count);
            n = (float) (Math.Sqrt(w * w + h * h) / 1000f) / tmp;  //TODO research best choice for n
            m_MinError = 0.00001f; //TODO

        }
        
        public void ReTriangulate(IEnumerable<Vector2f> _points)
        {
            Points.Clear();
            Triangles.Clear();
            BoundsRect = Rectf.zero;

            AddVertices(_points);

            Triangulate();
        }
        
        public void AddVertex(Vector2f _vertex)
        {
            Points.Add(_vertex);
            //m_PolysContainingVert.Add(new HashSet<Guid>());
            if(BoundsRect == Rectf.zero)
                BoundsRect = new Rectf(_vertex, Vector2f.zero);
            BoundsRect.Encapsulate(_vertex);
        }

        public void AddVertices(IEnumerable<Vector2f> _vertices)
        {
            foreach (var vert in _vertices)
                AddVertex(vert);

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

        public Triangulator triangulator => m_Triangulator;
       

        
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
            m_Triangulator.Triangulate();
        }


        private bool IsValidTri(int _v0, int _v1, int _v2)
        {
            //Check for dupe verts
            if (_v0 == _v1 || _v0 == _v2 || _v1 == _v2)
                return false;

            //Check for stright line
            var v0 = Points[_v0];
            var v1 = Points[_v1];
            var v2 = Points[_v2];

            var angleCCW = Vector2f.SignedAngle(v1 - v0, v2 - v0);
            if (angleCCW < float.Epsilon && angleCCW > -float.Epsilon)
                return false;
            return true;
        }

        public class Triangle
        {
            public HalfEdge Edge0;
            public HalfEdge Edge1;
            public HalfEdge Edge2;

            public float CCX;
            public float CCY;
            public float CCRSq;

            public HalfEdge Edge(int _idx)
            {
                if (_idx == 0)
                    return Edge0;
                if (_idx == 1)
                    return Edge1;
                return Edge2;
            }
            
            public Triangle(HalfEdge _twinLt, HalfEdge _twinRt, Delaunay _d)
            {
                D = _d;
                var v0 = _twinLt.OriginIdx;
                var v1 = _twinRt.NextEdge.OriginIdx;
                var v2 = _twinRt.OriginIdx;

                Edge0 = new HalfEdge(this, v0, _d);
                Edge1 = new HalfEdge(this, v1, _d);
                Edge2 = new HalfEdge(this, v2, _d);
                
                //Edges = new[] { edge0, edge1, edge2 };

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
                
                //Edges = new[] { edge0, edge1, edge2 };

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

            public Delaunay D;

            /// <summary>
            /// Calculate center and radius of circumcircle of this triangle.
            /// </summary>
            /// <param name="_center">Populated with center coord of circumcircle</param>
            /// <param name="_r">Populated with radius squared of circumcircle</param>
            public void CircumCircle(out Vector2f _center, out float _rSqr)
            {
                Geom.Circumcircle(Edge0.OriginPos, Edge1.OriginPos, Edge2.OriginPos, out _center, out _rSqr);
            }
            
            public class HalfEdge
            {
                public int OriginIdx;
                public Vector2f OriginPos => D.Points[OriginIdx];
                public Triangle Triangle;
                public HalfEdge NextEdge;

                public Delaunay D;

                public Vector2f AsVector
                {
                    get
                    {
                        if (NextEdge == null) return Vector2f.zero;
                        return NextEdge.OriginPos - OriginPos;
                    }
                }

                public HalfEdge(Triangle _triangle, int _originIdx, Delaunay _d)
                {
                    D = _d;
                    Triangle = _triangle;
                    OriginIdx = _originIdx;
                }

                public HalfEdge m_Twin; //TODO

                public void SetTwin(HalfEdge _twin)
                {
                    var newTwin = _twin;
                    if (m_Twin != null)
                        m_Twin.m_Twin = null;
                    m_Twin = newTwin;
                    if(newTwin != null)
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

            void ITriangulator.SetTarget(Delaunay _d) { D = _d; }

            public void Triangulate()
            {
                Algorithm();
                Hull();
            }

            /// <summary>
            /// This is where HullIdxs should be populateds
            /// </summary>
            protected abstract void Hull();
            /// <summary>
            /// This is where the triangulation algorithm to populate polys.
            /// </summary>
            protected abstract void Algorithm();

        }
    }
    /// <summary>
    /// Comparer for comparing two keys, handling equality as beeing greater
    /// Use this Comparer e.g. with SortedLists or SortedDictionaries, that don't allow duplicate keys
    /// </summary>
    /// <typeparam name="TKey"></typeparam>
    internal class DuplicateKeyComparer<TKey> : IComparer<TKey> where TKey : IComparable
    {
        public int Compare(TKey _x, TKey _y)
        {
            int result = _x.CompareTo(_y);

            return result == 0 ? 1 : result;
        }
    }
    
    public static class Ext
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float AngleCW(this Vector2f _from, Vector2f _to)
        {
            var signedAngleRad =  Math.Atan2(_to.y, _to.x) - Math.Atan2(_from.y, _from.x);
            return signedAngleRad >= 0 ?  (float)((2d * Math.PI) - signedAngleRad) : (float)(-signedAngleRad);
        }

        public static bool IsDelaunay(this Delaunay.Triangle.HalfEdge _edge)
        {
            var pts = _edge.D.Points;
            
            var a2 = pts[_edge.NextEdge.NextEdge.OriginIdx];
            var b2 = pts[_edge.m_Twin.NextEdge.NextEdge.OriginIdx];
            var triA = _edge.Triangle;
            var triB = _edge.m_Twin.Triangle;

            if (a2.SqrMagFast(triB.CCX, triB.CCY) < triB.CCRSq)
                return false;

            return (b2.SqrMagFast(triA.CCX, triA.CCY) >= triA.CCRSq);
            
        }
        
        public static bool IsDelaunayOld(this Delaunay.Triangle.HalfEdge _edge)
        {
            var pts = _edge.D.Points;
            
            var a2 = pts[_edge.NextEdge.NextEdge.OriginIdx];
            var b2 = pts[_edge.m_Twin.NextEdge.NextEdge.OriginIdx];
            var ab0 = pts[_edge.OriginIdx];
            var ab1 = pts[_edge.NextEdge.OriginIdx];
            
            float centX;
            float centY;
            float ccRadSqr;
            if (!Geom.Circumcircle(a2, ab0, ab1, out centX, out centY, out ccRadSqr))
                return false;  //Line condition

            //var distToCentSqr = (b2 - ccCent).sqrMagnitude;
            var distToCentSqr = b2.SqrMagFast(centX, centY);
            if (!(distToCentSqr >= ccRadSqr)) 
                return false;

            if (!Geom.Circumcircle(b2, ab1, ab0, out centX, out centY, out ccRadSqr))
                return false;  //Line condition

            //return (a2 - ccCent).sqrMagnitude >= ccRadSqr;
            return a2.SqrMagFast(centX, centY) >= ccRadSqr;
        }

        public static float SqrMagFast(this Vector2f _a, Vector2f _b)
        {
            var x = _b.x - _a.x;
            var y = _b.y - _a.y;
            
            return x * x + y * y;
        }
        
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static float SqrMagFast(this Vector2f _a, float _bX, float _bY)
        {
            var x = _bX - _a.x;
            var y = _bY - _a.y;
            return x * x + y * y;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static void FlipEdge(this Delaunay.Triangle.HalfEdge _edgeA, ref Delaunay.Triangle.HalfEdge[] _outerEdges)
        {
            var oldEdgeA = _edgeA; //Will be new edge A0
            var oldEdgeB = oldEdgeA.m_Twin; //Will be new Edge B0
            var triA = (Delaunay.Triangle) oldEdgeA.Triangle;
            var triB = (Delaunay.Triangle) oldEdgeA.m_Twin.Triangle;
            var a2Idx = _edgeA.NextEdge.NextEdge.OriginIdx;
            var b2Idx = _edgeA.m_Twin.NextEdge.NextEdge.OriginIdx;
                
            //Get edges quickly
            var newEB2 = oldEdgeA.NextEdge;
            var newEA1 = newEB2.NextEdge;
            var newEA2 = oldEdgeB.NextEdge;
            var newEB1 = newEA2.NextEdge;

            oldEdgeA.OriginIdx = b2Idx;
            oldEdgeB.OriginIdx = a2Idx;
            
                
            oldEdgeA.NextEdge = newEA1;
            newEA1.NextEdge = newEA2;
            newEA2.NextEdge = oldEdgeA;

                
            oldEdgeB.NextEdge = newEB1;
            newEB1.NextEdge = newEB2;
            newEB2.NextEdge = oldEdgeB;
                
            newEA2.Triangle = triA;
            newEB2.Triangle = triB;
                
            triA.Edge0 = oldEdgeA;
            triA.Edge1 = newEA1;
            triA.Edge2 = newEA2;
            triB.Edge0 = oldEdgeB;
            triB.Edge1 = newEB1;
            triB.Edge2 = newEB2;

            _outerEdges[0] = newEA1;
            _outerEdges[1] = newEA2;
            _outerEdges[2] = newEB1;
            _outerEdges[3] = newEB2;

            var pts = triA.D.Points;

            Geom.Circumcircle(pts[oldEdgeA.OriginIdx], pts[newEA1.OriginIdx], pts[newEA2.OriginIdx],
                out triA.CCX, out triA.CCY, out triA.CCRSq);

            Geom.Circumcircle(pts[oldEdgeB.OriginIdx], pts[newEB1.OriginIdx], pts[newEB2.OriginIdx],
                out triB.CCX, out triB.CCY, out triB.CCRSq);
        }
    }
}

