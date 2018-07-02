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
        
        public void Triangulate(IEnumerable<Vector2f> _points)
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

        public Triangle AddTriToMesh(int _vertIdx, Poly.HalfEdge _joiningEdge)
        {
            var newVerts = new[]
            {
                _vertIdx,
                _joiningEdge.NextEdge.OriginIdx,
                _joiningEdge.OriginIdx
            };

            if (!IsValidTri(newVerts[0], newVerts[1], newVerts[2]))
            {
                Trace.WriteLine("Invalid Tri for verts: " + newVerts[0] + " " + newVerts[1] + " " + newVerts[2]);
                return null;
            }

            var newTri = new Triangle(newVerts, this);
            var newEdge = newTri.EdgeWithOrigin(newVerts[1]); //TODO Check both edge verts?
            newEdge.Twin = _joiningEdge;
            return newTri;
        }

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

            var newTri = new Triangle(newVerts, this);
            var newEdgeLt = newTri.EdgeWithOrigin(newVerts[2]);
            newEdgeLt.Twin = _twinLt;
            var newEdgeRt = newTri.EdgeWithOrigin(newVerts[1]);
            newEdgeRt.Twin = _twinRt;
            return newTri;
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

            public TriEdgeData[] EdgeData;

            public TriEdgeData EdgeDataWithOrigin(int _idx)
            {
                return EdgeData[m_OriginToEdgeIdx[_idx]];
            }
            
            public Triangle(int[] _vertIdxs, Delaunay _d)
                : base(_vertIdxs, true, _d)
            {
                if (_vertIdxs.Length != 3)
                    throw new Exception("Vert count must be exactly 3");
                //Check for dupe verts
                if (_vertIdxs[0] == _vertIdxs[1] || _vertIdxs[0] == _vertIdxs[2] || _vertIdxs[1] == _vertIdxs[2])
                    throw new Exception("new Triangle - Dupe Verts"); //TODO Handle this

                D = _d;

                EdgeData = new TriEdgeData[3];
                
                for (int eIdx = 0; eIdx < 3; ++eIdx)
                    EdgeData[eIdx] = new TriEdgeData(Edges[eIdx]);
                
            }

            public Delaunay D { get; }

            public void CircumCircle(out Vector2f _center, out float _r)
            {
                Geom.Circumcircle(Edges[0].OriginPos, Edges[1].OriginPos, Edges[2].OriginPos, out _center, out _r);
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
            
                public Delaunay D { get; }
            
                public bool IsDelaunay
                {
                    get
                    {
                        var a2 = D.Points[vA2Idx];
                        var b2 = D.Points[vB2Idx];
                        var ab0 = D.Points[vAB0Idx];
                        var ab1 = D.Points[vAB1Idx];
                        Vector2f ccCent;
                        float ccRad;
                        if (!Geom.Circumcircle(a2, ab0, ab1, out ccCent, out ccRad))
                            throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                        var distToCentSqr = (b2 - ccCent).sqrMagnitude;
                        if (!(distToCentSqr >= ccRad * ccRad)) 
                            return false;
            
                        if (!Geom.Circumcircle(b2, ab1, ab0, out ccCent, out ccRad))
                            throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                        return (a2 - ccCent).sqrMagnitude >= ccRad * ccRad;
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
}