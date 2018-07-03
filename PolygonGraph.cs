using System;
using System.Collections.Generic;
using System.Linq;
using ioDelaunay;
using Vectorf;

namespace ioPolygonGraph
{
    
    public abstract class PolygonGraph
    {
        protected Rectf m_BoundsRect = Rectf.zero;
        public Rectf BoundsRect => m_BoundsRect;
        public List<Vector2f> Points;
        protected Dictionary<Guid, Poly> m_Polys;
        

        protected List<HashSet<Guid>> m_PolysContainingVert;

        protected PolygonGraph()
        {
            m_Polys = new Dictionary<Guid, Poly>();
            Points = new List<Vector2f>();
            m_PolysContainingVert = new List<HashSet<Guid>>();
        }

        protected PolygonGraph(Vector2f[] _points)
        {
            Points = _points.ToList();
            m_Polys = new Dictionary<Guid, Poly>();
            m_PolysContainingVert = new List<HashSet<Guid>>();
            var bndsRect = new Rectf(Points[0], Vector2f.zero);
            for (var idx = 0; idx < Points.Count; ++idx)
            {
                m_PolysContainingVert.Add(new HashSet<Guid>());
                bndsRect.Encapsulate(Points[idx]);
            }

            m_BoundsRect = bndsRect;
        }

        public Poly GetPoly(Guid _id)
        {
            return m_Polys[_id];
        }

        public void AddVertex(Vector2f _vertex)
        {
            Points.Add(_vertex);
            m_PolysContainingVert.Add(new HashSet<Guid>());
            if(BoundsRect == Rectf.zero)
                m_BoundsRect = new Rectf(_vertex, Vector2f.zero);
            m_BoundsRect.Encapsulate(_vertex);
        }

        public void AddVertices(IEnumerable<Vector2f> _vertices)
        {
            foreach (var vert in _vertices)
                AddVertex(vert);

        }
        
        public void RemovePoly(Guid _id)
        {
            var poly = m_Polys[_id];
            foreach (var edge in poly.Edges)
            {
                edge.Twin = null;
                m_PolysContainingVert[edge.OriginIdx].Remove(_id);
                m_Polys.Remove(_id);
            }
        }

        public void CleanVerts()
        {
            //TODO
        }
        //TODO Clean function if there are unused verts
        protected bool RemoveVerts(IEnumerable<int> _vertIdxs)
        {
            return false; //TODO
        }

        public class Poly : IPolyGraphObj
        {
            public readonly Guid ID;
            public bool Closed;
            public List<HalfEdge> Edges;
            
            protected Dictionary<int, int> m_OriginToEdgeIdx;

            protected Poly(int[] _vertIdxsOrdered, bool _closed, PolygonGraph _g)
            {
                G = _g;
                ID = Guid.NewGuid();
                G.m_Polys.Add(ID, this);
                Closed = _closed;
                Reform(_vertIdxsOrdered);
            }

            public List<int> VertIdxs => Edges.Select(_edge => _edge.OriginIdx).ToList();
            
            public PolygonGraph G { get; }

            public HalfEdge EdgeWithOrigin(int _vertIdx)
            {
                return Edges[m_OriginToEdgeIdx[_vertIdx]];
            }

            public HalfEdge Edge(int _edgeIdx)
            {
                return Edges[_edgeIdx];
            }

            private void FindAndSetTwin(int _edgeIdx)
            {
                var vA1Idx = -1;
                if (_edgeIdx == Edges.Count - 1)
                {
                    if (!Closed) return;
                    vA1Idx = Edges[0].OriginIdx;
                }
                else
                    vA1Idx = Edges[_edgeIdx + 1].OriginIdx;
                //Twins
                var vA0Idx = Edges[_edgeIdx].OriginIdx;
                var nbrPolys = G.m_PolysContainingVert[vA0Idx].Intersect(G.m_PolysContainingVert[vA1Idx])
                    .Except(new HashSet<Guid> {ID}).ToList();
                    
                //DEBUG
                /*
                if(nbrPolys.Count > 1)
                    Console.WriteLine("Debug");
                */

                if (nbrPolys.Count == 1)
                {
                    var nbrPoly = G.m_Polys[nbrPolys[0]];
                    var twin = nbrPoly.EdgeWithOrigin(vA1Idx);
                    if (twin.NextEdge.OriginIdx != vA0Idx)
                        twin = nbrPoly.EdgeWithOrigin(vA0Idx);
                    Edges[_edgeIdx].Twin = twin;
                }
            }

            
            
            public void Reform(int[] _vertIdxsOrdered)
            {
                if (Edges != null)
                {
                    for (int eIdx = 0; eIdx < Edges.Count; ++eIdx)
                    {
                        G.m_PolysContainingVert[Edges[eIdx].OriginIdx].Remove(ID);
                        if (Edges[eIdx].Twin != null)
                            Edges[eIdx].Twin.Twin = null;
                    }
                }
                    
                Edges = new List<HalfEdge>();
                
                m_OriginToEdgeIdx = new Dictionary<int, int>();

                var originIdx = _vertIdxsOrdered[0];
                Edges.Add(new HalfEdge(this, _vertIdxsOrdered[0], G));
                m_OriginToEdgeIdx.Add(originIdx, 0);
                G.m_PolysContainingVert[originIdx].Add(ID);
                for (int eIdx = 1; eIdx < _vertIdxsOrdered.Length; ++eIdx)
                {
                    originIdx = _vertIdxsOrdered[eIdx];
                    Edges.Add(new HalfEdge(this, originIdx, G));
                    FindAndSetTwin(eIdx - 1);
                    m_OriginToEdgeIdx.Add(originIdx, eIdx);
                    G.m_PolysContainingVert[originIdx].Add(ID);
                }

                if (Closed)
                    FindAndSetTwin(Edges.Count - 1);
            }

            public class HalfEdge : IPolyGraphObj
            {
                public int OriginIdx;
                public Vector2f OriginPos => G.Points[OriginIdx];
                public readonly Guid PolyID;

                public Vector2f AsVector
                {
                    get
                    {
                        if (NextEdge == null) return Vector2f.zero;
                        return NextEdge.OriginPos - OriginPos;
                    }
                }

                public HalfEdge(Poly _poly, int _originIdx, PolygonGraph _g)
                {
                    G = _g;
                    PolyID = _poly.ID;
                    OriginIdx = _originIdx;
                }

                public HalfEdge NextEdge
                {
                    get
                    {
                        var edgeIdx = EdgeIdx;
                        if (edgeIdx != Poly.Edges.Count - 1)
                            return Poly.Edges[edgeIdx + 1];
                        return Poly.Closed ? Poly.Edges[0] : null;
                    }
                }
                
                public Poly Poly => G.GetPoly(PolyID);

                public int EdgeIdx => Poly.m_OriginToEdgeIdx[OriginIdx];

                protected HalfEdge m_Twin;
                public HalfEdge Twin
                {
                    get { return m_Twin; }

                    set
                    {
                        var curTwin = m_Twin;
                        var newTwin = value;
                        if (curTwin != null)
                            curTwin.m_Twin = null;
                        m_Twin = newTwin;
                        if(newTwin != null)
                            newTwin.m_Twin = this;
                    }
                }

                public PolygonGraph G { get; }
            }
        }
        public interface IPolyGraphObj
        {
            PolygonGraph G { get; }
        }
    }


    
}