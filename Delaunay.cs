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
        public int[] HullIdxs;

        private Triangulator m_Triangulator;

        public static Delaunay Create<T>(Vector2f[] _points) where T : Triangulator, new()
        {
            var del = new Delaunay(_points);
            del.m_Triangulator = new T();
            ((ITriangulator)del.m_Triangulator).SetTarget(del);
            return del;
        }
        
        private Delaunay(Vector2f[] _points)
            : base(_points)
        {
            m_EdgeStack = new EdgeStack(this);
        }
        
        public void ReTriangulate(IEnumerable<Vector2f> _points)
        {
            Points.Clear();
            m_Polys.Clear();
            m_BoundsRect = Rectf.zero;

            AddVertices(_points);

            Triangulate();
        }


        public Triangle[] Triangles => m_Polys.Cast<Triangle>().ToArray();

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

        /*
        public Triangle AddTriToMesh(int _vertIdx, Poly.HalfEdge _joiningEdge)
        {
            var newVerts = new[]
            {
                _vertIdx,
                _joiningEdge.NextEdge.OriginIdx,
                _joiningEdge.OriginIdx
            };

            if (!IsValidTri(newVerts[0], newVerts[1], newVerts[2])) // DEBUG - Remove for optimization
            {
                Trace.WriteLine("Invalid Tri for verts: " + newVerts[0] + " " + newVerts[1] + " " + newVerts[2]);
                return null;
            }

            var newTri = new Triangle(newVerts[0], newVerts[1], newVerts[2], this);
            var newEdge = newTri.EdgeWithOrigin(newVerts[1]); //TODO Check both edge verts?
            newEdge.Twin = _joiningEdge;
            return newTri;
        }
        */
        
        public Triangle AddTriToMesh(int _vertIdx, Poly.HalfEdge _joiningEdge)
        {
            return new Triangle(_joiningEdge, _vertIdx, this);
        }

        /*
        public Triangle AddTriToMesh(Poly.HalfEdge _twinLt, Poly.HalfEdge _twinRt)
        {
            //Verify validity
            if (_twinLt.NextEdge.OriginIdx != _twinRt.OriginIdx)
                throw new Exception("AddTriToMesh - twins arent touching");

            var newVerts = new[]
            {
                _twinLt.OriginIdx,
                _twinRt.NextEdge.OriginIdx,
                _twinRt.OriginIdx
            };

            if (!IsValidTri(newVerts[0], newVerts[1], newVerts[2]))
            {
                Trace.WriteLine("Invalid Tri for verts: " + newVerts[0] + " " + newVerts[1] + " " + newVerts[2]);
                return null;
            }

            
            var newTri = new Triangle(newVerts[0], newVerts[1], newVerts[2], this);
            var newEdgeLt = newTri.EdgeWithOrigin(newVerts[2]);
            newEdgeLt.Twin = _twinLt;
            var newEdgeRt = newTri.EdgeWithOrigin(newVerts[1]);
            newEdgeRt.Twin = _twinRt;
            return newTri;
        }
        */
        
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
            /// <summary>
            /// Additional per edge data to optimize edge flipping / legalization.
            /// </summary>
            public readonly TriEdgeData[] EdgeData;

            public TriEdgeData EdgeDataWithOrigin(int _idx)
            {
                for(var eIdx = 0; eIdx < EdgeData.Length; ++eIdx)
                    if (EdgeData[eIdx].Edge.OriginIdx == _idx)
                        return EdgeData[eIdx];
                return null;
                //return EdgeData[m_OriginToEdgeIdx[_idx]];
            }

            public Triangle(HalfEdge _twinLt, HalfEdge _twinRt, Delaunay _d) : base(true, _d)
            {
                D = _d;
                var v0 = _twinLt.OriginIdx;
                var v1 = _twinRt.NextEdge.OriginIdx;
                var v2 = _twinRt.OriginIdx;

                EdgeData = new TriEdgeData[3];
                var edge0 = AddEdge(v0, null);
                var edge1 = AddEdge(v1, _twinRt);
                var edge2 = AddEdge(v2, _twinLt);
                EdgeData[0] = new TriEdgeData(edge0);
                EdgeData[1] = new TriEdgeData(edge1);
                EdgeData[2] = new TriEdgeData(edge2);
                
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                if (v0 == v1 || v0 == v2 || v1 == v2)
                    throw new Exception("new Triangle - Dupe Verts");

            }

            public Triangle(HalfEdge _twin, int _newVert, Delaunay _d) : base(true, _d)
            {
                D = _d;
                var v0 = _twin.OriginIdx;
                var v1 = _newVert;
                var v2 = _twin.NextEdge.OriginIdx;
                
                EdgeData = new TriEdgeData[3];
                var edge0 = AddEdge(v0, null);
                var edge1 = AddEdge(v1, null);
                var edge2 = AddEdge(v2, _twin);
                EdgeData[0] = new TriEdgeData(edge0);
                EdgeData[1] = new TriEdgeData(edge1);
                EdgeData[2] = new TriEdgeData(edge2);
                
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                if (v0 == v1 || v0 == v2 || v1 == v2)
                    throw new Exception("new Triangle - Dupe Verts");
            }
            
            public Triangle(int _vertIdx0, int _vertIdx1, int _vertIdx2, Delaunay _d)
                : base(new [] {_vertIdx0, _vertIdx1, _vertIdx2}, true, _d)
            {
                //Check for dupe verts - DEBUG TODO - Remove for optimization
                if (_vertIdx0 == _vertIdx1 || _vertIdx0 == _vertIdx2 || _vertIdx1 == _vertIdx2)
                    throw new Exception("new Triangle - Dupe Verts");

                D = _d;

                EdgeData = new TriEdgeData[3];
                
                for (int eIdx = 0; eIdx < 3; ++eIdx)
                    EdgeData[eIdx] = new TriEdgeData(Edges[eIdx]);
                
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

            public void FlipEdgeOld(TriEdgeData _edgeDataA)
            {
                var oldEdgeA = _edgeDataA.Edge;
                var triA = (Triangle) oldEdgeA.Poly;
                var triB = (Triangle) oldEdgeA.Twin.Poly;
                var a2Idx = _edgeDataA.vA2Idx;
                var b2Idx = _edgeDataA.vB2Idx;
                
                //Get edges quickly
                var newEB2 = oldEdgeA.NextEdge;
                var newEA1 = newEB2.NextEdge;
                var newEA2 = oldEdgeA.Twin.NextEdge;
                var newEB1 = newEA2.NextEdge;

                var newEdgesA = new List<HalfEdge>
                {
                    new HalfEdge(triA, b2Idx, D),
                    newEA1,
                    newEA2

                };
                
                newEdgesA[0].NextEdge = newEdgesA[1];
                newEdgesA[1].NextEdge = newEdgesA[2];
                newEdgesA[2].NextEdge = newEdgesA[0];

                var newEdgesB = new List<HalfEdge>
                {
                    new HalfEdge(triB, a2Idx, D),
                    newEB1,
                    newEB2
                };
                
                newEdgesB[0].NextEdge = newEdgesB[1];
                newEdgesB[1].NextEdge = newEdgesB[2];
                newEdgesB[2].NextEdge = newEdgesB[0];
                
                newEdgesA[2].PolyID = triA.ID;
                newEdgesB[2].PolyID = triB.ID;
                
                triA.Edges = newEdgesA;
                triB.Edges = newEdgesB;
                
                newEdgesA[0].Twin = newEdgesB[0];
                
                for (int eIdx = 0; eIdx < 3; ++eIdx)
                {
                    triA.EdgeData[eIdx] = new TriEdgeData(triA.Edges[eIdx]);
                    triB.EdgeData[eIdx] = new TriEdgeData(triB.Edges[eIdx]);
                }
            }
            
            
            public class TriEdgeData
            {
                public int vA2Idx => m_Edge.NextEdge.NextEdge.OriginIdx;
                public int vB2Idx => m_Edge.Twin.NextEdge.NextEdge.OriginIdx;
                public int vAB0Idx;
                public int vAB1Idx => m_Edge.NextEdge.OriginIdx;

                private readonly HalfEdge m_Edge;
                public HalfEdge Edge => m_Edge;

                public TriEdgeData(HalfEdge _edge)
                {
                    m_Edge = _edge;
                    vAB0Idx = _edge.OriginIdx;
                    //vAB1Idx = _edge.NextEdge.OriginIdx;
                    D = (Delaunay) _edge.G;
                }
            
                private Delaunay D { get; }
            
                /// <summary>
                /// Checks whether this Triangle is a valid Delaunay Triangulation against it's neighbors.
                /// </summary>
                public bool IsDelaunay
                {
                    get
                    {
                        var a2 = D.Points[vA2Idx];
                        var b2 = D.Points[vB2Idx];
                        var ab0 = D.Points[vAB0Idx];
                        var ab1 = D.Points[vAB1Idx];
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

            private float m_Progress;
            public float Progress => m_Progress;

            public void Triangulate()
            {
                m_Progress = 0;
                Algorithm(ref m_Progress);
                Hull(ref m_Progress);
                m_Progress = 100f;
            }

            /// <summary>
            /// This is where HullIdxs should be populateds
            /// </summary>
            protected abstract void Hull(ref float _progress);
            /// <summary>
            /// This is where the triangulation algorithm to populate polys.
            /// </summary>
            protected abstract void Algorithm(ref float _progress);

            /// <summary>
            /// Convenience triangle accessor
            /// </summary>
            /// <param name="_triID">ID of triangle</param>
            /// <returns>Triangle with corresponding ID</returns>
            protected Triangle Tri(int _triID)
            {
                return (Triangle) D.m_Polys[_triID];
            }
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

        public static int VAB0Idx(this PolygonGraph.Poly.HalfEdge _edge)
        {
            return _edge.OriginIdx;
        }

        public static int VAB1Idx(this PolygonGraph.Poly.HalfEdge _edge)
        {
            return _edge.NextEdge.OriginIdx;
        }

        public static int VA2Idx(this PolygonGraph.Poly.HalfEdge _edge)
        {
            return _edge.NextEdge.NextEdge.OriginIdx;
        }

        public static int VB2Idx(this PolygonGraph.Poly.HalfEdge _edge)
        {
            return _edge.Twin.NextEdge.NextEdge.OriginIdx;
        }

        public static List<PolygonGraph.Poly.HalfEdge> FlipEdge(this PolygonGraph.Poly.HalfEdge _edgeA)
        {
            var oldEdgeA = _edgeA;
            var oldEdgeB = oldEdgeA.Twin;
            var triA = (Delaunay.Triangle) oldEdgeA.Poly;
            var triB = (Delaunay.Triangle) oldEdgeA.Twin.Poly;
            var a2Idx = _edgeA.VA2Idx();
            var b2Idx = _edgeA.VB2Idx();
                
            //Get edges quickly
            var newEB2 = oldEdgeA.NextEdge;
            var newEA1 = newEB2.NextEdge;
            var newEA2 = oldEdgeB.NextEdge;
            var newEB1 = newEA2.NextEdge;

            oldEdgeA.OriginIdx = b2Idx;
            oldEdgeB.OriginIdx = a2Idx;
            
            var newEdgesA = new List<PolygonGraph.Poly.HalfEdge>
            {
                oldEdgeA,
                newEA1,
                newEA2
            };
                
            newEdgesA[0].NextEdge = newEdgesA[1];
            newEdgesA[1].NextEdge = newEdgesA[2];
            newEdgesA[2].NextEdge = newEdgesA[0];

            var newEdgesB = new List<PolygonGraph.Poly.HalfEdge>
            {
                oldEdgeB,
                newEB1,
                newEB2
            };
                
            newEdgesB[0].NextEdge = newEdgesB[1];
            newEdgesB[1].NextEdge = newEdgesB[2];
            newEdgesB[2].NextEdge = newEdgesB[0];
                
            newEdgesA[2].PolyID = triA.ID;
            newEdgesB[2].PolyID = triB.ID;
                
            triA.Edges = newEdgesA;
            triB.Edges = newEdgesB;
                
            //newEdgesA[0].Twin = newEdgesB[0];
                
            return new List<PolygonGraph.Poly.HalfEdge>
            {
                newEdgesA[1],
                newEdgesA[2],
                newEdgesB[1],
                newEdgesB[2]
            };
        }
    }
}

