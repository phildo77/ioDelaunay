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
        
        public class FlipInfo
        {
            public readonly Dictionary<Guid, EdgeRef[]> EdgeChangeData;

            public FlipInfo(Guid _triAID, Guid _triBID)
            {
                EdgeChangeData = new Dictionary<Guid, EdgeRef[]>();
                EdgeChangeData.Add(_triAID, new EdgeRef[3]);
                EdgeChangeData.Add(_triBID, new EdgeRef[3]);
            }

            public void Update(ref Guid _triID, ref int _edgeIdx)
            {
                var edgeRefs = EdgeChangeData[_triID];
                _triID = edgeRefs[_edgeIdx].NewTriID;
                _edgeIdx = edgeRefs[_edgeIdx].NewEdgeIdx;
            }

            public bool IsTriChanged(Guid _triID)
            {
                return EdgeChangeData.ContainsKey(_triID);
            }
                
            public class EdgeRef
            {
                public readonly Guid NewTriID;
                public readonly int NewEdgeIdx;

                public EdgeRef(Guid _newTriID, int _newEdgeIdx)
                {
                    NewTriID = _newTriID;
                    NewEdgeIdx = _newEdgeIdx;
                }
            }
        }

        //Edge Flip - returns null if not neighbors
        private FlipInfo FlipEdge(Guid _triAID, Guid _triBID)
        {
            
            var triA = m_Triangles[_triAID];
            var triB = m_Triangles[_triBID];
            

            //Find ab edge
            
            var eA2 = triA.HalfEdges[0];
            var firstEdge = eA2;
            while (true)
            {
                if (eA2.Twin != null)
                    if (eA2.Twin.TriID == _triBID)
                        break;
                eA2 = eA2.NextEdge;
                if (eA2 == firstEdge)
                    return null;
            }

            var eA0 = eA2.NextEdge;
            var eA1 = eA0.NextEdge;

            var eB2 = eA2.Twin;
            var eB0 = eB2.NextEdge;
            var eB1 = eB0.NextEdge;

            var newEdgesA = new HalfEdge[3];

            newEdgesA[eA0.Idx] = new HalfEdge(triA, eA0.OriginIdx, this);
            newEdgesA[eA1.Idx] = new HalfEdge(triA, eA1.OriginIdx, this);
            newEdgesA[eA2.Idx] = new HalfEdge(triA, eB1.OriginIdx, this);


            var newEdgesB = new HalfEdge[3];

            newEdgesB[eB0.Idx] = new HalfEdge(triB, eB0.OriginIdx, this);
            newEdgesB[eB1.Idx] = new HalfEdge(triB, eB1.OriginIdx, this);
            newEdgesB[eB2.Idx] = new HalfEdge(triB, eA1.OriginIdx, this);
            

            newEdgesA[0].Twin = eA0.Twin;
            newEdgesA[2].Twin = eB1.Twin;

            newEdgesB[0].Twin = eB0.Twin;
            newEdgesB[2].Twin = eA1.Twin;
            
            var fi = new FlipInfo(triA.ID, triB.ID);
            fi.EdgeChangeData[_triAID][0] = new FlipInfo.EdgeRef(triA.ID, 0);
            fi.EdgeChangeData[_triAID][1] = new FlipInfo.EdgeRef(triB.ID, 2);
            fi.EdgeChangeData[_triAID][2] = null;

            fi.EdgeChangeData[_triBID][0] = new FlipInfo.EdgeRef(triB.ID, 0);
            fi.EdgeChangeData[_triBID][1] = new FlipInfo.EdgeRef(triA.ID, 2);
            fi.EdgeChangeData[_triBID][2] = null;
            
            triA.HalfEdges = newEdgesA;
            triB.HalfEdges = newEdgesB;
            
            newEdgesA[1].Twin = newEdgesB[1];
            newEdgesB[1].Twin = newEdgesA[1];

            return fi;

        }
            
            public Dictionary<int, FlipInfo> Legalize(Guid _startTri)
            {
                
                var flipInfoByVertIdx = new Dictionary<int, FlipInfo>();
                foreach (var edge in m_Triangles[_startTri].HalfEdges)
                {
                    if (edge.Twin == null) continue;
                    var vA2 = edge.NextEdge.NextEdge.Origin;
                    var vAB0 = edge.Origin;
                    var vAB1 = edge.NextEdge.Origin;
                    var vB2 = edge.Twin.NextEdge.NextEdge.Origin;
                    Vector2f ccCent;
                    float ccRad;

                    if(!Geom.Circumcircle(vA2.Pos, vAB0.Pos, vAB1.Pos, out ccCent, out ccRad))
                        throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                    var checkDistSqr = (vB2.Pos - ccCent).sqrMagnitude;
                    if (checkDistSqr >= ccRad) continue;

                    var vIdx = edge.OriginIdx;
                    var fi = FlipEdge(edge.TriID, edge.Twin.TriID);
                    flipInfoByVertIdx.Add(vIdx, fi);
                    //TODO RECURSE
                }

                return flipInfoByVertIdx;
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
            public HalfEdge[] HalfEdges;
            public readonly Guid ID;

            
            private readonly Dictionary<int, int> m_EdgeIdxByVertIdx;

            #endregion Fields

            #region Constructors

            public Triangle(int[] _vertIdxs, DelaunayBase _dRef) : base(_dRef)
            {
                if(_vertIdxs.Length != 3)
                    throw new Exception("Vert count must be exactly 3");

                //Sort points clockwise
                var v0 = D.m_Points[_vertIdxs[0]];
                var v1 = D.m_Points[_vertIdxs[1]];
                var v2 = D.m_Points[_vertIdxs[2]];
                var v1Idx = _vertIdxs[1];
                var v2Idx = _vertIdxs[2];

                if (Vector2f.SignedAngle(v1 - v0, v2 - v0) < 0)
                {
                    var vTmpIdx = v1Idx;
                    v1Idx = v2Idx;
                    v2Idx = vTmpIdx;
                }
                
                m_EdgeIdxByVertIdx = new Dictionary<int, int>();
                HalfEdges = new HalfEdge[3];

                ID = Guid.NewGuid();

                D.m_Triangles.Add(ID, this);
                
                HalfEdges[0] = new HalfEdge(this, _vertIdxs[0], D);
                HalfEdges[1] = new HalfEdge(this, v1Idx, D);
                HalfEdges[2] = new HalfEdge(this, v2Idx, D);
                
                m_EdgeIdxByVertIdx.Add(VertIdxs[0], 0);
                m_EdgeIdxByVertIdx.Add(VertIdxs[1], 1);
                m_EdgeIdxByVertIdx.Add(VertIdxs[2], 2);
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
                    if(m_TwinIdx == -1 && m_NeighborID != Guid.Empty)
                        Console.WriteLine("Debug WTF");
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