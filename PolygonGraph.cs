using System;
using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioPolygonGraph
{
    public abstract class PolygonGraph
    {
        public readonly Rectf BoundsRect;
        protected readonly Vector2f[] m_Points;
        protected readonly Dictionary<Guid, Poly> m_Polys;
        protected readonly Vertex[] m_Vertices;

        protected HashSet<Guid>[] m_PolysContainingVert;

        protected PolygonGraph(Vector2f[] _points)
        {
            m_Points = _points;
            m_Vertices = new Vertex[m_Points.Length];
            m_Polys = new Dictionary<Guid, Poly>();
            m_PolysContainingVert = new HashSet<Guid>[m_Points.Length];
            var bndsRect = new Rectf(m_Points[0], Vector2f.zero);
            for (var idx = 0; idx < m_Points.Length; ++idx)
            {
                m_Vertices[idx] = new Vertex(idx, this);
                m_PolysContainingVert[idx] = new HashSet<Guid>();
                bndsRect.Encapsulate(m_Points[idx]);
            }

            BoundsRect = bndsRect;
        }

        public Poly GetPoly(Guid _id)
        {
            return m_Polys[_id];
        }


        public class Vertex : IPolyGraphObj
        {
            public readonly int Idx;

            public Vertex(int _idx, PolygonGraph _g)
            {
                G = _g;
                Idx = _idx;
            }

            public Vector2f Pos => G.m_Points[Idx];

            public float x => Pos.x;
            public float y => Pos.y;
            public PolygonGraph G { get; }

            public override string ToString()
            {
                return "Vrt Idx: " + Idx + " Pos: " + Pos;
            }
        }


        public class Poly : IPolyGraphObj
        {
            private const int HALFEDGE_NULL_IDX = -1;
            public readonly Guid ID;
            protected bool Closed;

            protected int[] m_EdgeOrigins;

            protected Guid[] m_EdgeToNbrID;
            protected int[] m_EdgeToTwinEdgeIdx;
            protected Dictionary<int, int> m_OriginToEdgeIdx;

            protected Poly(int[] _vertIdxsOrdered, bool _closed, PolygonGraph _g)
            {
                G = _g;
                ID = Guid.NewGuid();
                G.m_Polys.Add(ID, this);
                Reform(_vertIdxsOrdered);
            }

            public int[] VertIdxs => new List<int>(m_EdgeOrigins).ToArray();

            public Vertex[] Verts
            {
                get { return VertIdxs.Select(_id => G.m_Vertices[_id]).ToArray(); }
            }

            public HashSet<Guid> NeighborIDs
            {
                get
                {
                    var nbrs = new HashSet<Guid>();
                    foreach (var nbrID in m_EdgeToNbrID)
                        if (nbrID != Guid.Empty)
                            nbrs.Add(nbrID);
                    return nbrs;
                }
            }

            public HalfEdge[] Edges
            {
                get
                {
                    var edges = new HalfEdge[m_EdgeOrigins.Length];
                    for (var eIdx = 0; eIdx < m_EdgeOrigins.Length; ++eIdx) edges[eIdx] = Edge(eIdx);

                    return edges;
                }
            }


            public PolygonGraph G { get; }

            public HalfEdge EdgeWithOrigin(int _vertIdx)
            {
                return new HalfEdge(this, _vertIdx, G);
            }

            public HalfEdge Edge(int _edgeIdx)
            {
                return new HalfEdge(this, m_EdgeOrigins[_edgeIdx], G);
            }

            public void Reform(int[] _vertIdxsOrdered)
            {
                if (m_EdgeOrigins != null)
                    foreach (var originIdx in m_EdgeOrigins)
                        G.m_PolysContainingVert[originIdx].Remove(ID);
                m_EdgeOrigins = _vertIdxsOrdered;
                m_OriginToEdgeIdx = new Dictionary<int, int>();
                m_EdgeToNbrID = new Guid[m_EdgeOrigins.Length];
                m_EdgeToTwinEdgeIdx = new int[m_EdgeOrigins.Length];
                for (var idx = 0; idx < m_EdgeOrigins.Length; ++idx)
                {
                    m_EdgeToNbrID[idx] = Guid.Empty;
                    m_EdgeToTwinEdgeIdx[idx] = HALFEDGE_NULL_IDX;
                    m_OriginToEdgeIdx.Add(m_EdgeOrigins[idx], idx);
                    G.m_PolysContainingVert[m_EdgeOrigins[idx]].Add(ID);
                }
            }


            public class HalfEdge : IPolyGraphObj
            {
                public readonly int OriginIdx;
                public readonly Guid PolyID;

                public HalfEdge(Poly _poly, int _originIdx, PolygonGraph _g)
                {
                    G = _g;
                    PolyID = _poly.ID;
                    OriginIdx = _originIdx;
                }

                public Vertex Origin => G.m_Vertices[OriginIdx];

                public HalfEdge NextEdge
                {
                    get
                    {
                        var edgeIdx = EdgeIdx;
                        if (edgeIdx != Poly.m_EdgeOrigins.Length - 1)
                            return Poly.EdgeWithOrigin(Poly.m_EdgeOrigins[edgeIdx + 1]);
                        return Poly.Closed ? null : Poly.EdgeWithOrigin(Poly.m_EdgeOrigins[0]);
                    }
                }

                public Poly Poly => G.GetPoly(PolyID);

                public int EdgeIdx => Poly.m_OriginToEdgeIdx[OriginIdx];


                public HalfEdge Twin
                {
                    get
                    {
                        var nbrPolyID = Poly.m_EdgeToNbrID[EdgeIdx];
                        if (nbrPolyID == Guid.Empty) return null;
                        var nbrPoly = G.GetPoly(nbrPolyID);
                        return nbrPoly.Edge(Poly.m_EdgeToTwinEdgeIdx[EdgeIdx]);
                    }

                    set
                    {
                        if (value == null)
                        {
                            if (Poly.m_EdgeToNbrID[EdgeIdx] != Guid.Empty)
                            {
                                var nbrPoly = G.GetPoly(Poly.m_EdgeToNbrID[EdgeIdx]);
                                var twinIdx = Poly.m_EdgeToTwinEdgeIdx[EdgeIdx];
                                nbrPoly.m_EdgeToTwinEdgeIdx[twinIdx] = HALFEDGE_NULL_IDX;
                                nbrPoly.m_EdgeToNbrID[twinIdx] = Guid.Empty;
                            }

                            Poly.m_EdgeToTwinEdgeIdx[EdgeIdx] = HALFEDGE_NULL_IDX;
                            Poly.m_EdgeToNbrID[EdgeIdx] = Guid.Empty;
                            return;
                        }

                        Poly.m_EdgeToTwinEdgeIdx[EdgeIdx] = value.EdgeIdx;
                        Poly.m_EdgeToNbrID[EdgeIdx] = value.PolyID;
                        var nPoly = G.GetPoly(value.PolyID);
                        nPoly.m_EdgeToTwinEdgeIdx[value.EdgeIdx] = EdgeIdx;
                        nPoly.m_EdgeToNbrID[value.EdgeIdx] = PolyID;
                    }
                }

                public PolygonGraph G { get; }
            }
        }
    }


    public interface IPolyGraphObj
    {
        PolygonGraph G { get; }
    }
}