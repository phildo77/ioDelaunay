using System;
using System.Collections.Generic;
using System.Linq;
using System.Security.Cryptography;
using Vectorf;

namespace ioDelaunay
{
    public abstract class DelaunayBase
    {
        protected readonly Vector2f[] m_Points;
        protected readonly Vertex[] m_Vertices;
        
        protected Dictionary<Guid, Triangle> m_Triangles;
        
        public DelaunayBase(Vector2f[] _points)
        {
            m_Points = _points;
            m_Vertices = new Vertex[m_Points.Length];
            for (int idx = 0; idx < m_Points.Length; ++idx)
                m_Vertices[idx] = new Vertex(idx, this);
            m_Triangles = new Dictionary<Guid, Triangle>();
        }

        protected abstract void Algorithm();

        public Mesh Triangulate()
        {
            Algorithm();
            return Mesh;
        }
        
        protected Triangle AddTriToMesh(int _vertIdx, HalfEdge _joiningEdge)
        {
            var newVerts = new[]
            {
                _vertIdx,
                _joiningEdge.NextEdge.OriginIdx,
                _joiningEdge.OriginIdx
            };
            var newTri = new Triangle(newVerts, this);
            var newEdge = newTri.GetHalfEdgeWithOrigin(newVerts[1]);
            newEdge.Twin = _joiningEdge;
            _joiningEdge.Twin = newEdge;
            var dupes = DebugDupeEdgeCheck(newTri);
            if (dupes.Count != 0)
                Console.WriteLine(dupes.ToArray());
            return newTri;

        }

        

        protected Triangle AddTriToMesh(HalfEdge _leftTwin, HalfEdge _rtTwin)
        {
            var newVerts = new[]
            {
                _leftTwin.OriginIdx,
                _rtTwin.NextEdge.OriginIdx,
                _rtTwin.OriginIdx
            };

            var newTri = new Triangle(newVerts, this);
            var newEdgeA = newTri.GetHalfEdgeWithOrigin(newVerts[1]);
            newEdgeA.Twin = _rtTwin;
            _rtTwin.Twin = newEdgeA;
            var newEdgeB = newTri.GetHalfEdgeWithOrigin(newVerts[2]);
            newEdgeB.Twin = _leftTwin;
            _leftTwin.Twin = newEdgeB;
            var dupes = DebugDupeEdgeCheck(newTri);
            if (dupes.Count != 0)
                Console.WriteLine(dupes.ToArray());
            return newTri;
        }
        
        
        
#if DEBUG
        private List<HalfEdge> DebugDupeEdgeCheck(Triangle _tri)
        {
            var dupes = new List<HalfEdge>();
            foreach (var hEdgeA in _tri.HalfEdges)
            {
                foreach (var tri in m_Triangles)
                {
                    if (tri.Key == _tri.ID) continue;

                    foreach (var hEdgeB in tri.Value.HalfEdges)
                    {
                        if (hEdgeA.OriginIdx != hEdgeB.OriginIdx)
                            continue;
                        if (hEdgeA.NextEdge.OriginIdx != hEdgeB.NextEdge.OriginIdx)
                            continue;
                        dupes.Add(hEdgeA);
                        dupes.Add(hEdgeB);
                    }
                }
            }

            return dupes;
        }
#endif

        public Mesh Mesh
        {
            get
            {
                var tris = m_Triangles.Values.ToArray();
                var triIdxs = new int[m_Triangles.Count * 3];
                for (int tIdx = 0; tIdx < tris.Length; ++tIdx)
                    for (int vIdx = 0; vIdx < 3; ++vIdx)
                        triIdxs[tIdx * 3 + vIdx] = tris[tIdx].HalfEdges[vIdx].OriginIdx;
                return new Mesh(m_Points, triIdxs);
            }
        }
        
        protected class Vertex : DelaunayObj
        {
            public Vertex(int idx, DelaunayBase _d) : base(_d)
            {
                Idx = idx;
            }
            
            #region Fields
            
            public int Idx;

            #endregion Fields
            
            public Vector2f Pos => D.m_Points[Idx];

            public override string ToString()
            {
                return "Vrt Idx: " + Idx + " Pos: " + Pos;
            }
        }
        
        protected class Triangle : DelaunayObj
        {
            #region Fields

            //Key is Vertex Origin ID
            public readonly HalfEdge[] HalfEdges;
            public readonly Guid ID;

            private readonly Dictionary<int, int> m_EdgeIdxByVertIdx;

            #endregion Fields

            #region Constructors

            public Triangle(int[] _vertIdxs, DelaunayBase _dRef) : base(_dRef)
            {
                if(_vertIdxs.Length != 3)
                    throw new Exception("Vert count must be exactly 3");

                //Sort points clockwise

                var centroid = Geom.CentroidOfPoly(D.m_Points[_vertIdxs[0]], D.m_Points[_vertIdxs[1]],
                    D.m_Points[_vertIdxs[2]]);
                var pointsAroundOrigin = new Dictionary<int, Vector2f>()
                {
                    {_vertIdxs[0], D.m_Points[_vertIdxs[0]] - centroid},
                    {_vertIdxs[1], D.m_Points[_vertIdxs[1]] - centroid},
                    {_vertIdxs[2], D.m_Points[_vertIdxs[2]] - centroid}
                };
                
                var clkWisePAO = pointsAroundOrigin.OrderByDescending(_pt => Math.Atan2(_pt.Value.x, _pt.Value.y)).ToArray();
                
                m_EdgeIdxByVertIdx = new Dictionary<int, int>();
                HalfEdges = new HalfEdge[3];

                ID = Guid.NewGuid();

                D.m_Triangles.Add(ID, this);
                
                HalfEdges[0] = new HalfEdge(this, clkWisePAO[0].Key, D);
                HalfEdges[1] = new HalfEdge(this, clkWisePAO[1].Key, D);
                HalfEdges[2] = new HalfEdge(this, clkWisePAO[2].Key, D);
                
                HalfEdges[0].NextEdge = HalfEdges[1];
                HalfEdges[1].NextEdge = HalfEdges[2];
                HalfEdges[2].NextEdge = HalfEdges[0];
                
                m_EdgeIdxByVertIdx.Add(VertIdxs[0], 0);
                m_EdgeIdxByVertIdx.Add(VertIdxs[1], 1);
                m_EdgeIdxByVertIdx.Add(VertIdxs[2], 2);
            }

            //For edge flip
            private Triangle(Guid _id, HalfEdge[] _edges, DelaunayBase _d) : base(_d)
            {
                ID = _id;
                HalfEdges = _edges;
                D.m_Triangles[_id] = this;
            }
            
            //Edge Flip - returns false if not neighbors
            private bool FlipEdge(Guid _triAID, Guid _triBID)
            {
                var triA = D.m_Triangles[_triAID];

                //Find ab edge
                var eA0 = triA.HalfEdges[0];
                var firstEdge = eA0;
                while (true)
                {
                    if (eA0.Twin != null)
                        if (eA0.Twin.TriID == _triBID)
                            break;
                    eA0 = eA0.NextEdge;
                    if (eA0 == firstEdge)
                        return false;
                }

                var eA1 = eA0.NextEdge;
                var eA2 = eA1.NextEdge;

                var eB1 = eA0.Twin;
                var eB0 = eB1.NextEdge;
                var eB2 = eB0.NextEdge;

                var newEdgesA = new[]
                {
                    new HalfEdge(triA, eA2.OriginIdx, triA.D),
                    new HalfEdge(triA, eA1.OriginIdx, triA.D),
                    new HalfEdge(triA, eB2.OriginIdx, triA.D)
                };

                return true;
            }

            

            #endregion Constructors

            public HalfEdge GetHalfEdgeWithOrigin(int _vertIdx)
            {
                return !m_EdgeIdxByVertIdx.ContainsKey(_vertIdx) ? null : HalfEdges[m_EdgeIdxByVertIdx[_vertIdx]];
            }

            public int GetHalfEdgeIdxWithOrigin(int _vertIdx)
            {
                return !m_EdgeIdxByVertIdx.ContainsKey(_vertIdx) ? -1 : m_EdgeIdxByVertIdx[_vertIdx];
            }

            public int[] VertIdxs => new[]
            {
                HalfEdges[0].OriginIdx,
                HalfEdges[1].OriginIdx,
                HalfEdges[2].OriginIdx
            };

            public Vertex[] Verts
            {
                get { return VertIdxs.Select(_id => D.m_Vertices[_id]).ToArray(); }
            }
            
        }
        
        protected class HalfEdge : DelaunayObj
        {
            #region Fields

            public readonly Guid TriID;

            private Guid m_NeighborID; //Aligned with edge idx
            private int m_TwinIdx; //Aligned with edge idx - this is idx of edge on neighbor
            
            public int Idx
            {
                get
                {
                    var tri = D.m_Triangles[TriID];
                    for (int idx = 0; idx < 3; ++idx)
                        if (tri.HalfEdges[idx] == this)
                            return idx;
                    return -1;
                }
            }

            public readonly int OriginIdx;
            public Vertex Origin => D.m_Vertices[OriginIdx];

            public HalfEdge Twin
            {
                get
                {
                    if (m_NeighborID == Guid.Empty) return null;
                    return D.m_Triangles[m_NeighborID].HalfEdges[m_TwinIdx];
                }

                set
                {
                    if (value == null)
                    {
                        m_NeighborID = Guid.Empty;
                        m_TwinIdx = -1;
                        return;
                    }
                    m_NeighborID = value.TriID;
                    m_TwinIdx = value.Idx;
                }
            }

            public HalfEdge NextEdge
            {
                get
                {
                    var tri = D.m_Triangles[TriID];
                    var idx = Idx;
                    return idx == 2 ? tri.HalfEdges[0] : tri.HalfEdges[idx + 1];
                }

                set
                {
                    var idx = Idx;
                    D.m_Triangles[TriID].HalfEdges[idx == 2 ? 0 : idx + 1] = value;
                }
            }

            #endregion Fields

            #region Constructors

            public HalfEdge(Triangle _tri, int _originID, DelaunayBase _d) : base(_d)
            {
                m_NeighborID = Guid.Empty;
                TriID = _tri.ID;
                OriginIdx = _originID;
                Twin = null;
            }
            #endregion Constructors

            public Triangle Tri => D.m_Triangles[TriID];

            public override string ToString()
            {
                var o = (Origin ?? (Object) "").ToString();
                var t = Twin?.Origin.ToString() ?? "";
                return "HlfEdg Org: " + o + " Nxt: " + NextEdge.Origin + " Twin: " + t;
            }
        }

        
        protected abstract class DelaunayObj
        {
            protected readonly DelaunayBase D;
            protected DelaunayObj(DelaunayBase _d)
            {
                D = _d;
            }
        }
    }
}