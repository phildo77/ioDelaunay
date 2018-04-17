namespace ioDelaunay
{
    using System;
    using System.Collections.Generic;
    using System.Linq;

    using Vectorf;

    public abstract partial class DelaunayBase
    {
        #region Fields

        protected readonly Vector2f[] m_Points;
        protected readonly HashSet<Guid>[] m_TrisContainingVert;
        protected readonly Vertex[] m_Vertices;

        protected Dictionary<Guid, Triangle> m_Triangles;
        protected HashSet<Guid> m_CheckedTris;

        #endregion Fields

        public List<Triangle> Triangles => m_Triangles.Values.ToList(); //DEBUG TODO

        #region Constructors

        protected DelaunayBase(Vector2f[] _points)
        {
            m_Triangles = new Dictionary<Guid, Triangle>();
            m_Points = _points;
            m_Vertices = new Vertex[m_Points.Length];
            m_TrisContainingVert = new HashSet<Guid>[m_Points.Length];

            m_CheckedTris = new HashSet<Guid>();

            for (int idx = 0; idx < m_Points.Length; ++idx)
            {
                m_Vertices[idx] = new Vertex(idx, this);
                m_TrisContainingVert[idx] = new HashSet<Guid>();
            }
        }

        #endregion Constructors

        protected abstract void Algorithm();
        
        public Mesh Triangulate()
        {
            Algorithm();
            return Mesh;
        }
        
        public Mesh Mesh
        {
            get
            {
                var tris = m_Triangles.Values.ToArray();
                var triIdxs = new int[m_Triangles.Count * 3];
                for (int tIdx = 0; tIdx < tris.Length; ++tIdx)
                for (int vIdx = 0; vIdx < 3; ++vIdx)
                    triIdxs[tIdx * 3 + vIdx] = tris[tIdx].HalfEdge(vIdx).OriginIdx;
                return new Mesh(m_Points, triIdxs);
            }
        }
        
        #region Nested Types

        public abstract class DelaunayObj
        {
            #region Fields

            public readonly DelaunayBase D;

            #endregion Fields

            #region Constructors

            public DelaunayObj(DelaunayBase _d)
            {
                D = _d;
            }

            #endregion Constructors
        }

        public class HalfEdge : DelaunayObj
        {
            #region Fields

            public readonly int OriginIdx;
            public readonly Guid TriID;

            public int Idx => D.m_Triangles[TriID].HalfEdgeIdxWithVert(OriginIdx);
            public Vertex Origin => D.m_Vertices[OriginIdx];
            public Triangle Tri => D.m_Triangles[TriID];

            private Guid m_NeighborID; //Aligned with edge idx
            private int m_TwinIdx; //Aligned with edge idx - this is idx of edge on neighbor

            #endregion Fields

            #region Constructors

            public HalfEdge(Triangle _tri, int _originID, DelaunayBase _d)
                : base(_d)
            {
                TriID = _tri.ID;
                OriginIdx = _originID;
                Twin = null;
            }

            #endregion Constructors

            #region Properties

            public HalfEdge NextEdge
            {
                get
                {
                    var tri = D.m_Triangles[TriID];
                    var idx = Idx;
                    return idx == 2 ? tri.HalfEdge(0) : tri.HalfEdge(idx + 1);
                }
            }

            public HalfEdge Twin
            {
                get
                {
                    return m_NeighborID == Guid.Empty ? null : D.m_Triangles[m_NeighborID].HalfEdge(m_TwinIdx);
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
                    value.m_NeighborID = TriID;
                    value.m_TwinIdx = Idx;
                    
                    //DEBUG TODO
                    if (value.OriginIdx == OriginIdx)
                        Console.WriteLine("DEBUG");
                }
            }

            #endregion Properties

            #region Methods

            public override string ToString()
            {
                var o = (Origin ?? (Object) "").ToString();
                var t = Twin?.Origin.ToString() ?? "";
                return "HlfEdg Org: " + o + " Nxt: " + NextEdge.Origin + " Twin: " + t;
            }

            #endregion Methods
        }

        public class Triangle : DelaunayObj
        {
            #region Fields

            public readonly Guid ID;

            public int[] VertIdxs => new[]
            {
                m_HalfEdges[0].OriginIdx,
                m_HalfEdges[1].OriginIdx,
                m_HalfEdges[2].OriginIdx
            };

            private Dictionary<int, int> m_HalfEdgeByVertIdx;
            private HalfEdge[] m_HalfEdges;

            #endregion Fields

            #region Constructors

            public Triangle(int[] _vertIdxs, DelaunayBase _dRef)
                : base(_dRef)
            {
                if(_vertIdxs.Length != 3)
                    throw new Exception("Vert count must be exactly 3");
                //Check for dupe verts
                if (_vertIdxs[0] == _vertIdxs[1] || _vertIdxs[0] == _vertIdxs[2] || _vertIdxs[1] == _vertIdxs[2])
                    throw new Exception("new Triangle - Dupe Verts");  //TODO Handle this

                //Sort points clockwise
                var v0 = D.m_Points[_vertIdxs[0]];
                var v1 = D.m_Points[_vertIdxs[1]];
                var v2 = D.m_Points[_vertIdxs[2]];

                var v1Idx = _vertIdxs[1];
                var v2Idx = _vertIdxs[2];

                //Force Clockwise
                var angleCCW = Vector2f.SignedAngle(v1 - v0, v2 - v0);
                if (angleCCW > 0)
                {
                    var vTmpIdx = v1Idx;
                    v1Idx = v2Idx;
                    v2Idx = vTmpIdx;
                }
                else if (angleCCW == 0)
                    throw new Exception("new Triangle - Striaght line"); //TODO Handle this

                //Create Edges
                m_HalfEdgeByVertIdx = new Dictionary<int, int>();
                m_HalfEdges = new HalfEdge[3];

                ID = Guid.NewGuid();

                D.m_Triangles.Add(ID, this);

                m_HalfEdges[0] = new HalfEdge(this, _vertIdxs[0], D);
                m_HalfEdges[1] = new HalfEdge(this, v1Idx, D);
                m_HalfEdges[2] = new HalfEdge(this, v2Idx, D);

                D.m_TrisContainingVert[_vertIdxs[0]].Add(ID);
                D.m_TrisContainingVert[v1Idx].Add(ID);
                D.m_TrisContainingVert[v2Idx].Add(ID);

                m_HalfEdgeByVertIdx.Add(VertIdxs[0], 0);
                m_HalfEdgeByVertIdx.Add(VertIdxs[1], 1);
                m_HalfEdgeByVertIdx.Add(VertIdxs[2], 2);
            }

            #endregion Constructors

            #region Properties

            public HashSet<Guid> NeighborIDs
            {
                get
                {
                    var nbrs = new HashSet<Guid>();
                    for(int idx = 0; idx < 3; ++idx)
                        if (m_HalfEdges[idx].Twin != null)
                            nbrs.Add(m_HalfEdges[idx].Twin.TriID);
                    return nbrs;
                }
            }

            public Vertex[] Verts
            {
                get { return VertIdxs.Select(_id => D.m_Vertices[_id]).ToArray(); }
            }

            #endregion Properties

            #region Methods

            public HalfEdge GetHalfEdgeWithOrigin(int _vertIdx)
            {
                return !m_HalfEdgeByVertIdx.ContainsKey(_vertIdx) ? null : m_HalfEdges[m_HalfEdgeByVertIdx[_vertIdx]];
            }

            public HalfEdge HalfEdge(int _idx)
            {
                return m_HalfEdges[_idx];
            }

            public int HalfEdgeIdxWithVert(int _idx)
            {
                return m_HalfEdgeByVertIdx[_idx];
            }

            public HalfEdge HalfEdgeWithVert(int _idx)
            {
                return m_HalfEdges[m_HalfEdgeByVertIdx[_idx]];
            }

            public HalfEdge HalfEdgeWithVert(Vertex _vert)
            {
                return HalfEdgeWithVert(_vert.Idx);
            }

            public void SetHalfEdges(HalfEdge[] _edges)
            {
                m_HalfEdges = _edges;
                m_HalfEdgeByVertIdx.Clear();
                for(int idx = 0; idx < 3; ++idx)
                    m_HalfEdgeByVertIdx.Add(m_HalfEdges[idx].OriginIdx, idx);
            }

            #endregion Methods
        }

        public class Vertex : DelaunayObj
        {
            #region Fields

            public int Idx;
            public Vector2f Pos => D.m_Points[Idx];

            #endregion Fields

            #region Constructors

            public Vertex(int idx, DelaunayBase _d)
                : base(_d)
            {
                Idx = idx;
            }

            #endregion Constructors

            #region Methods

            public override string ToString()
            {
                return "Vrt Idx: " + Idx + " Pos: " + Pos;
            }

            #endregion Methods
        }

        #endregion Nested Types
    }
}