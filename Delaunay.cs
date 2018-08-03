using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using ioPolygonGraph;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay : PolygonGraph
    {
        public List<int> HullIdxs;
       
        private Triangulator m_Triangulator;

        public static Delaunay Create<T>(Vector2f[] _points) where T : Triangulator, new()
        {
            var del = new Delaunay(_points);
            del.m_Triangulator = new T();
            ((ITriangulator)del.m_Triangulator).SetTarget(del);
            return del;
        }

        private Delaunay(Vector2f[] _points) : base(_points)
        {
        }
        
        public void ReTriangulate(IEnumerable<Vector2f> _points)
        {
            Points.Clear();
            m_Polys.Clear();
            m_BoundsRect = Rectf.zero;

            AddVertices(_points);

            Triangulate();
        }

        public Mesh Mesh
        {
            get
            {
                var tris = m_Polys.ToArray();
                var triIdxs = new int[m_Polys.Count * 3];
                for (var tIdx = 0; tIdx < tris.Length; ++tIdx)
                for (var vIdx = 0; vIdx < 3; ++vIdx)
                    triIdxs[tIdx * 3 + vIdx] = tris[tIdx].Edge(vIdx).OriginIdx;
                return new Mesh(Points.ToArray(), triIdxs);
            }
        }

        public Triangulator triangulator => m_Triangulator;

        public Triangle[] Triangles => m_Polys.Cast<Triangle>().ToArray();
        
        public Triangle AddTriToMesh(int _vertIdx, Poly.HalfEdge _joiningEdge)
        {
            return new Triangle(_joiningEdge, _vertIdx, this);
        }

        
        public Triangle AddTriToMesh(Poly.HalfEdge _twinLt, Poly.HalfEdge _twinRt)
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

        public interface IDelaunayObj
        {
            Delaunay D { get; }
        }


        public class Triangle : Poly, IDelaunayObj
        {
            
            public Triangle(HalfEdge _twinLt, HalfEdge _twinRt, Delaunay _d) : base(true, _d)
            {
                D = _d;
                var v0 = _twinLt.OriginIdx;
                var v1 = _twinRt.NextEdge.OriginIdx;
                var v2 = _twinRt.OriginIdx;

                var edges = new[]
                {
                    new HalfEdge(this, v0, D),
                    new HalfEdge(this, v1, D),
                    new HalfEdge(this, v2, D)
                };

                edges[0].NextEdge = edges[1];
                edges[1].NextEdge = edges[2];
                edges[2].NextEdge = edges[0];

                Edges = new List<HalfEdge>(edges);

                edges[1].Twin = _twinRt;
                edges[2].Twin = _twinLt;
                
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (v0 == v1 || v0 == v2 || v1 == v2)
                //    throw new Exception("new Triangle - Dupe Verts");

            }

            public Triangle(HalfEdge _twin, int _newVert, Delaunay _d) : base(true, _d)
            {
                D = _d;
                var v0 = _twin.OriginIdx;
                var v1 = _newVert;
                var v2 = _twin.NextEdge.OriginIdx;

                var edge0 = new HalfEdge(this, v0, D);
                var edge1 = new HalfEdge(this, v1, D);
                var edge2 = new HalfEdge(this, v2, D);

                edge0.NextEdge = edge1;
                edge1.NextEdge = edge2;
                edge2.NextEdge = edge0;
                
                Edges.Add(edge0);
                Edges.Add(edge1);
                Edges.Add(edge2);

                edge2.Twin = _twin;
                
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                //if (v0 == v1 || v0 == v2 || v1 == v2)
                //    throw new Exception("new Triangle - Dupe Verts");
            }
            
            public Triangle(int _vertIdx0, int _vertIdx1, int _vertIdx2, Delaunay _d)
                : base(new [] {_vertIdx0, _vertIdx1, _vertIdx2}, true, _d)
            {
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                if (_vertIdx0 == _vertIdx1 || _vertIdx0 == _vertIdx2 || _vertIdx1 == _vertIdx2)
                    throw new Exception("new Triangle - Dupe Verts");

                D = _d;
                
            }

            public Delaunay D { get; }

            /// <summary>
            /// Calculate center and radius of circumcircle of this triangle.
            /// </summary>
            /// <param name="_center">Populated with center coord of circumcircle</param>
            /// <param name="_r">Populated with radius squared of circumcircle</param>
            public void CircumCircle(out Vector2f _center, out float _rSqr)
            {
                Geom.Circumcircle(Edges[0].OriginPos, Edges[1].OriginPos, Edges[2].OriginPos, out _center, out _rSqr);
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
        public static bool IsDelaunay(this PolygonGraph.Poly.HalfEdge _edge)
        {
            var pts = _edge.G.Points;
            
            var a2 = pts[_edge.NextEdge.NextEdge.OriginIdx];
            var b2 = pts[_edge.Twin.NextEdge.NextEdge.OriginIdx];
            var ab0 = pts[_edge.OriginIdx];
            var ab1 = pts[_edge.NextEdge.OriginIdx];
            Vector2f ccCent;
            float ccRadSqr;
            if (!Geom.Circumcircle(a2, ab0, ab1, out ccCent, out ccRadSqr))
                return false;  //Line condition

            var distToCentSqr = (b2 - ccCent).sqrMagnitude;
            if (!(distToCentSqr >= ccRadSqr)) 
                return false;

            if (!Geom.Circumcircle(b2, ab1, ab0, out ccCent, out ccRadSqr))
                return false;  //Line condition

            return (a2 - ccCent).sqrMagnitude >= ccRadSqr;
        }

        public static void FlipEdge(this PolygonGraph.Poly.HalfEdge _edgeA, ref PolygonGraph.Poly.HalfEdge[] _outerEdges)
        {
            var oldEdgeA = _edgeA; //Will be new edge A0
            var oldEdgeB = oldEdgeA.Twin; //Will be new Edge B0
            var triA = (Delaunay.Triangle) oldEdgeA.Poly;
            var triB = (Delaunay.Triangle) oldEdgeA.Twin.Poly;
            var a2Idx = _edgeA.NextEdge.NextEdge.OriginIdx;
            var b2Idx = _edgeA.Twin.NextEdge.NextEdge.OriginIdx;
                
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
                
            newEA2.PolyID = triA.ID;
            newEB2.PolyID = triB.ID;
                
            triA.Edges[0] = oldEdgeA;
            triA.Edges[1] = newEA1;
            triA.Edges[2] = newEA2;
            triB.Edges[0] = oldEdgeB;
            triB.Edges[1] = newEB1;
            triB.Edges[2] = newEB2;

            _outerEdges[0] = newEA1;
            _outerEdges[1] = newEA2;
            _outerEdges[2] = newEB1;
            _outerEdges[3] = newEB2;
        }
    }
}

