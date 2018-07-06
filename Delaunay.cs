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
        }
        
        public void ReTriangulate(IEnumerable<Vector2f> _points)
        {
            Points.Clear();
            m_Polys.Clear();
            m_PolysContainingVert.Clear();
            m_BoundsRect = Rectf.zero;

            AddVertices(_points);

            Triangulate();
        }


        public Triangle[] Triangles => m_Polys.Values.Cast<Triangle>().ToArray();

        public Mesh Mesh
        {
            get
            {
                var tris = m_Polys.Values.ToArray();
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
                return EdgeData[m_OriginToEdgeIdx[_idx]];
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

            public void FlipEdge(int _edgeOriginIdx)
            {
                var oldEdgeA = EdgeWithOrigin(_edgeOriginIdx);
                var edgeData = EdgeDataWithOrigin(_edgeOriginIdx);
                var triA = (Triangle) oldEdgeA.Poly;
                var triB = (Triangle) oldEdgeA.Twin.Poly;
                var ab0Idx = edgeData.vAB0Idx;
                var ab1Idx = edgeData.vAB1Idx;
                var a2Idx = edgeData.vA2Idx;
                var b2Idx = edgeData.vB2Idx;


                var newEdgesA = new List<HalfEdge>
                {
                    new HalfEdge(triA, b2Idx, D),
                    triA.EdgeWithOrigin(a2Idx),
                    triB.EdgeWithOrigin(ab0Idx)

                };

                var newEdgesB = new List<HalfEdge>
                {
                    new HalfEdge(triB, a2Idx, D),
                    triB.EdgeWithOrigin(b2Idx),
                    triA.EdgeWithOrigin(ab1Idx)
                };

                newEdgesA[2].PolyID = triA.ID;
                newEdgesB[2].PolyID = triB.ID;

                D.m_PolysContainingVert[ab1Idx].Remove(triA.ID);
                D.m_PolysContainingVert[ab0Idx].Remove(triB.ID);

                D.m_PolysContainingVert[b2Idx].Add(triA.ID);
                D.m_PolysContainingVert[a2Idx].Add(triB.ID);
                
                triA.Edges = newEdgesA;
                triB.Edges = newEdgesB;
                
                triA.m_OriginToEdgeIdx = new Dictionary<int, int>
                {
                    {b2Idx, 0},
                    {a2Idx, 1},
                    {ab0Idx, 2}
                };
                
                triB.m_OriginToEdgeIdx = new Dictionary<int, int>
                {
                    {a2Idx, 0},
                    {b2Idx, 1},
                    {ab1Idx, 2}
                };
                
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
                public readonly int vAB0Idx;
                public readonly int vAB1Idx;

                private readonly HalfEdge m_Edge;
                public HalfEdge Edge => m_Edge;

                public TriEdgeData(HalfEdge _edge)
                {
                    m_Edge = _edge;
                    vAB0Idx = _edge.OriginIdx;
                    vAB1Idx = _edge.NextEdge.OriginIdx;
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
            protected Dictionary<Guid, Poly> Polys => D.m_Polys;
            protected List<HashSet<Guid>> PolysContainingVert => D.m_PolysContainingVert;

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
            protected Triangle Tri(Guid _triID)
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
}