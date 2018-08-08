using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioPolygonGraph
{
    
    public abstract class PolygonGraph
    {
        protected Rectf m_BoundsRect = Rectf.zero;
        public Rectf BoundsRect => m_BoundsRect;
        public List<Vector2f> Points;
        //protected Dictionary<Guid, Poly> m_Polys;
        protected List<Poly> m_Polys;
        

        //protected List<HashSet<Guid>> m_PolysContainingVert;

        protected PolygonGraph()
        {
            m_Polys = new List<Poly>();
            Points = new List<Vector2f>();
        }

        protected PolygonGraph(Vector2f[] _points)
        {
            Points = _points.ToList();
            m_Polys = new List<Poly>();
            var bndsRect = new Rectf(Points[0], Vector2f.zero);
            for (var idx = 0; idx < Points.Count; ++idx)
                bndsRect.Encapsulate(Points[idx]);

            m_BoundsRect = bndsRect;
        }

        public Poly GetPoly(int _id)
        {
            return m_Polys[_id];
        }

        public void AddVertex(Vector2f _vertex)
        {
            Points.Add(_vertex);
            //m_PolysContainingVert.Add(new HashSet<Guid>());
            if(BoundsRect == Rectf.zero)
                m_BoundsRect = new Rectf(_vertex, Vector2f.zero);
            m_BoundsRect.Encapsulate(_vertex);
        }

        public void AddVertices(IEnumerable<Vector2f> _vertices)
        {
            foreach (var vert in _vertices)
                AddVertex(vert);

        }
        
        //Expensive TODO
        public void RemovePoly(int _id)
        {
            var poly = m_Polys[_id];
            foreach (var edge in poly.Edges)
            {
                edge.Twin = null;
                //m_PolysContainingVert[edge.OriginIdx].Remove(_id);
                m_Polys.RemoveAt(_id);
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
            public readonly int ID;
            public bool Closed;
            public List<HalfEdge> Edges;
            
            protected Poly(bool _closed, PolygonGraph _g)
            {
                G = _g;
                ID = G.m_Polys.Count;
                G.m_Polys.Add(this);
                Closed = _closed;
                Edges = new List<HalfEdge>();
            }
            
            protected Poly(int[] _vertIdxsOrdered, bool _closed, PolygonGraph _g)
            {
                G = _g;
                ID = G.m_Polys.Count;
                G.m_Polys.Add(this);
                Closed = _closed;
                Edges = new List<HalfEdge>();
                var prevEdge = new HalfEdge(this, _vertIdxsOrdered[0], G);
                Edges.Add(prevEdge);
                for (int eIdx = 1; eIdx < _vertIdxsOrdered.Length; ++eIdx)
                {
                    var curEdge = new HalfEdge(this, _vertIdxsOrdered[eIdx], G);
                    Edges.Add(curEdge);
                    prevEdge.NextEdge = curEdge;
                    prevEdge = curEdge;
                }

                if (Closed)
                    Edges[Edges.Count - 1].NextEdge = Edges[0];
            }
            

            public HalfEdge AddEdge(int _originIdx, HalfEdge _twin)
            {
                var edge = new HalfEdge(this, _originIdx, G);
                Edges.Add(edge);
                if (Edges.Count != 1)
                {
                    var prevEdge = Edges[Edges.Count - 2];
                    prevEdge.NextEdge = edge;
                }
                edge.NextEdge = Closed ? Edges[0] : null;
                
                if (_twin != null)
                    edge.Twin = _twin;

                return edge;
            }
            
            public List<int> VertIdxs => Edges.Select(_edge => _edge.OriginIdx).ToList();
            
            public PolygonGraph G { get; }

            public HalfEdge EdgeWithOrigin(int _vertIdx)
            {
                for(int eIdx = 0; eIdx < Edges.Count; ++eIdx)
                    if (Edges[eIdx].OriginIdx == _vertIdx)
                        return Edges[eIdx];
                return null;
            }

            public HalfEdge Edge(int _edgeIdx)
            {
                return Edges[_edgeIdx];
            }

            

            public class HalfEdge : IPolyGraphObj
            {
                public int OriginIdx;
                public Vector2f OriginPos => G.Points[OriginIdx];
                public int PolyID;
                public HalfEdge NextEdge;

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

                /*
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
                */
                
                public Poly Poly => G.GetPoly(PolyID);

                public int EdgeIdx
                {
                    get
                    {
                        var edges = Poly.Edges;
                        for (int eIdx = 0; eIdx < edges.Count; ++eIdx)
                        {
                            if (edges[eIdx].OriginIdx == OriginIdx)
                                return eIdx;
                        }

                        return -1;
                    }
                }

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