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
        protected HashSet<Guid> m_CheckedTris;
        public int[] HullIdxs;

        private Triangulator m_Triangulator;

        public Delaunay(Vector2f[] _points)
            : base(_points)
        {
            m_CheckedTris = new HashSet<Guid>();
            m_Triangulator = new CircleSweep();
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
                return new Mesh(Points, triIdxs);
            }
        }

        public HashSet<int> DebugForceAllLegalize()
        {
            var affectedVerts = new HashSet<int>();
            foreach (var triID in m_Polys.Keys)
            {
                affectedVerts = Legalize(triID);
            }

            return affectedVerts;
        }
        
        public Triangulator triangulator
        {
            get { return m_Triangulator; }

            set
            {
                m_Triangulator = value;
                m_Triangulator.SetD(this);
            }
        }

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


        public Triangle Tri(Guid _triID)
        {
            return (Triangle) m_Polys[_triID];
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
            var v0 = m_Vertices[_v0].Pos;
            var v1 = m_Vertices[_v1].Pos;
            var v2 = m_Vertices[_v2].Pos;

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
            public static int DebugAddTriCnt = 0;
            public Triangle(int[] _vertIdxs, Delaunay _d)
                : base(_vertIdxs, true, _d)
            {
                if (_vertIdxs.Length != 3)
                    throw new Exception("Vert count must be exactly 3");
                //Check for dupe verts
                if (_vertIdxs[0] == _vertIdxs[1] || _vertIdxs[0] == _vertIdxs[2] || _vertIdxs[1] == _vertIdxs[2])
                    throw new Exception("new Triangle - Dupe Verts"); //TODO Handle this

                D = _d;

                //Sort points clockwise
                var v0 = D.m_Vertices[_vertIdxs[0]].Pos;
                var v1 = D.m_Vertices[_vertIdxs[1]].Pos;
                var v2 = D.m_Vertices[_vertIdxs[2]].Pos;

                //Force Clockwise
                var angleCCW = Vector2f.SignedAngle(v1 - v0, v2 - v0);
                if (angleCCW > 0)
                    Reform(_vertIdxs[0], _vertIdxs[2], _vertIdxs[1]);
                else if (angleCCW == 0)
                    throw new Exception("new Triangle - Striaght line"); //TODO Handle this
                if(DebugAddTriCnt != 0)//TODO debug
                    DebugVisualizer.Visualize(D, null, "addTri" + DebugAddTriCnt);
                if(DebugAddTriCnt == 45)
                    Console.WriteLine("Debug");
                DebugAddTriCnt++;
            }

            public Delaunay D { get; }

            public void CircumCircle(out Vector2f _center, out float _r)
            {
                Geom.Circumcircle(Verts[0].Pos, Verts[1].Pos, Verts[2].Pos, out _center, out _r);
            }

            public void Reform(int _vIdx0, int _vIdx1, int _vIdx2)
            {
                Reform(new[] {_vIdx0, _vIdx1, _vIdx2});
            }

            
            
            public class HullEdge : HalfEdge
            {
                public HullEdge(Poly _poly, int _originIdx, PolygonGraph _g) : base(_poly, _originIdx, _g)
                {
                }
            }
        }

        public abstract class Triangulator
        {
            protected Delaunay D;
            protected Vector2f[] Points => D.Points;
            protected Dictionary<Guid, Poly> Polys => D.m_Polys;
            protected HashSet<Guid>[] PolysContainingVert => D.m_PolysContainingVert.ToArray();
            protected Vertex[] Vertices => D.m_Vertices.ToArray();

            public void SetD(Delaunay _d)
            {
                D = _d;
            }

            public void Triangulate()
            {
                Algorithm();
                Hull();
            }

            /// <summary>
            /// This is where HullIdxs should be populated
            /// </summary>
            protected abstract void Hull();
            /// <summary>
            /// This is where the triangulation algorithm to populate polys.
            /// </summary>
            protected abstract void Algorithm();

            protected Triangle Tri(Guid _triID)
            {
                return (Triangle) D.m_Polys[_triID];
            }
        }
    }
}