using System;
using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        public HashSet<int> Legalize(Guid _startTriID)
        {
            var eStack = new Stack<FEdge>();
            var eMarked = new HashSet<int>();
            var affectedVerts = new HashSet<int>();
            foreach (var hEdge in m_Polys[_startTriID].Edges)
                if (hEdge.Twin != null)
                {
                    var fEdge = new FEdge(hEdge, this);
                    eMarked.Add(fEdge.GetHashCode());
                }

            while (eStack.Count != 0)
            {
                var fEdge = eStack.Pop();
                eMarked.Remove(fEdge.GetHashCode());
                if (!IsDelaunay(fEdge))
                {
                    Poly.HalfEdge[] _outerEdges;
                    affectedVerts.UnionWith(fEdge.Flip(out _outerEdges));
                    foreach (var oEdge in _outerEdges)
                        if (!eMarked.Contains(oEdge.GetHashCode()))
                        {
                            var newEdge = new FEdge(oEdge, this);
                            eMarked.Add(newEdge.GetHashCode());
                            eStack.Push(newEdge);
                        }
                }
            }

            return affectedVerts;
        }

        private bool IsDelaunay(FEdge _edge)
        {
            if (!_edge.HasTwin) return true;
            var vA2 = m_Vertices[_edge.vA2Idx];
            var vAB0 = m_Vertices[_edge.vAB0Idx];
            var vAB1 = m_Vertices[_edge.vAB1Idx];
            var vB2 = m_Vertices[_edge.vB2Idx];

            Vector2f ccCent;
            float ccRad;
            if (!Geom.Circumcircle(vA2.Pos, vAB0.Pos, vAB1.Pos, out ccCent, out ccRad))
                throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

            var checkDistSqr = (vB2.Pos - ccCent).sqrMagnitude;
            return checkDistSqr >= ccRad * ccRad;
        }

        private void Legalize(ref HashSet<int> _affectedVerts, ref HashSet<Guid> _toCheck)
        {
            var curTriID = _toCheck.First();
            _toCheck.Remove(curTriID);
            for (var edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
            {
                var edge = Tri(curTriID).Edge(edgeIdx);
                if (edge.Twin == null) continue;

                var nbrID = edge.Twin.PolyID;
                var vA2 = edge.NextEdge.NextEdge.Origin;
                var vAB0 = edge.Origin;
                var vAB1 = edge.NextEdge.Origin;
                var vB2 = edge.Twin.NextEdge.NextEdge.Origin;

                Vector2f ccCent;
                float ccRad;
                if (!Geom.Circumcircle(vA2.Pos, vAB0.Pos, vAB1.Pos, out ccCent, out ccRad))
                    throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                var checkDistSqr = (vB2.Pos - ccCent).sqrMagnitude;
                if (checkDistSqr >= ccRad * ccRad)
                {
                    if (!m_CheckedTris.Contains(edge.Twin.PolyID))
                        _toCheck.Add(edge.Twin.PolyID);
                    continue;
                }

                _affectedVerts.UnionWith(FlipEdge(edge.PolyID, edge.Twin.PolyID));


                var newChecks = new HashSet<Guid> {curTriID, nbrID};
                newChecks.UnionWith(Tri(curTriID).NeighborIDs);
                newChecks.UnionWith(Tri(nbrID).NeighborIDs);

                m_CheckedTris.ExceptWith(newChecks);
                _toCheck.UnionWith(newChecks);
                return;
            }

            m_CheckedTris.Add(curTriID);
        }

        /*
        public HashSet<int> Legalize(params Guid[] _triIDs)
        {
            
            var affectedVerts = new HashSet<int>();
            var toCheck = new HashSet<Guid>();
            toCheck.UnionWith(_triIDs);
            m_CheckedTris.ExceptWith(_triIDs);
            while (toCheck.Count > 0)
                Legalize(ref affectedVerts, ref toCheck);
            return affectedVerts;
        }
        
        private void Legalize(ref HashSet<int> _affectedVerts, ref HashSet<Guid> _toCheck)
        {
            var curTriID = _toCheck.First();
            _toCheck.Remove(curTriID);
            for(int edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
            {
                var edge = Tri(curTriID).Edge(edgeIdx);
                if (edge.Twin == null) continue;

                var nbrID = edge.Twin.PolyID;
                var vA2 = edge.NextEdge.NextEdge.Origin;
                var vAB0 = edge.Origin;
                var vAB1 = edge.NextEdge.Origin;
                var vB2 = edge.Twin.NextEdge.NextEdge.Origin;

                Vector2f ccCent;
                float ccRad;
                if(!Geom.Circumcircle(vA2.Pos, vAB0.Pos, vAB1.Pos, out ccCent, out ccRad))
                    throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                var checkDistSqr = (vB2.Pos - ccCent).sqrMagnitude;
                if (checkDistSqr >= (ccRad * ccRad))
                {
                    if (!m_CheckedTris.Contains(edge.Twin.PolyID))
                        _toCheck.Add(edge.Twin.PolyID);
                    continue;
                }

                _affectedVerts.UnionWith(FlipEdge(edge.PolyID, edge.Twin.PolyID));

                
                var newChecks = new HashSet<Guid> {curTriID, nbrID};
                newChecks.UnionWith(Tri(curTriID).NeighborIDs);
                newChecks.UnionWith(Tri(nbrID).NeighborIDs);

                m_CheckedTris.ExceptWith(newChecks);
                _toCheck.UnionWith(newChecks);
                return;
            }

            m_CheckedTris.Add(curTriID);
        }
        */
        //Edge Flip
        private HashSet<int> FlipEdge(Guid _triAID, Guid _triBID)
        {
            var triA = Tri(_triAID);
            var triB = Tri(_triBID);

            //Find twin
            var oeA2 = triA.Edge(0);
            var first = oeA2;
            while (true)
            {
                if (oeA2.Twin != null)
                    if (oeA2.Twin.PolyID == _triBID)
                        break;
                oeA2 = oeA2.NextEdge;
                if (oeA2 == first)
                    throw new Exception("Flip Edge - Tris not neighbors");
            }

            var oeB2 = oeA2.Twin;

            var twinA0 = oeA2.NextEdge.Twin;
            var twinA2 = oeB2.NextEdge.NextEdge.Twin;

            var twinB0 = oeB2.NextEdge.Twin;
            var twinB2 = oeA2.NextEdge.NextEdge.Twin;

            var nvA0 = oeA2.NextEdge.OriginIdx;
            var nvA1 = oeA2.NextEdge.NextEdge.OriginIdx;

            var nvB0 = oeB2.NextEdge.OriginIdx;
            var nvB1 = oeB2.NextEdge.NextEdge.OriginIdx;

            triA.Reform(nvA0, nvA1, nvB1);
            triB.Reform(nvB0, nvB1, nvA1);

            triA.Edge(0).Twin = twinA0;
            triA.Edge(2).Twin = twinA2;
            triB.Edge(0).Twin = twinB0;
            triB.Edge(2).Twin = twinB2;
            triA.Edge(1).Twin = triB.Edge(1);

            var affectedVertIdxs = new HashSet<int>
            {
                triA.Edge(0).OriginIdx,
                triA.Edge(1).OriginIdx,
                triA.Edge(2).OriginIdx,
                triB.Edge(0).OriginIdx
            };

            return affectedVertIdxs;
        }

        public class FEdge : IDelaunayObj
        {
            public readonly int hEdgeAIdx;
            public readonly Guid TriAID;
            public readonly int vA2Idx;
            public readonly int vAB0Idx;
            public readonly int vAB1Idx;
            public readonly int vB2Idx;

            public FEdge(Poly.HalfEdge _he, Delaunay _d)
            {
                D = _d;
                TriAID = _he.PolyID;
                hEdgeAIdx = _he.EdgeIdx;
                vAB0Idx = _he.OriginIdx;
                if (_he.Twin == null)
                {
                    vB2Idx = -1;
                    return;
                }

                vAB1Idx = _he.NextEdge.OriginIdx;
                vA2Idx = _he.NextEdge.NextEdge.OriginIdx;
                vB2Idx = _he.Twin.NextEdge.NextEdge.OriginIdx;
            }

            public bool HasTwin => vB2Idx != -1;

            public Delaunay D { get; }

            public override int GetHashCode()
            {
                return HashCode(vAB0Idx, vAB1Idx);
            }

            private static int HashCode(int _vAb0Idx, int _vAb1Idx)
            {
                var ha = _vAb0Idx;
                var hb = _vAb1Idx;
                if (hb < ha)
                {
                    var temp = hb;
                    hb = ha;
                    ha = temp;
                }

                return ha.GetHashCode() ^ (hb.GetHashCode() << 2);
            }

            public HashSet<int> Flip(out Poly.HalfEdge[] _outerEdgesWTwin)
            {
                var outerEdges = new List<Poly.HalfEdge>();
                var triA = (Triangle) D.m_Polys[TriAID];
                var triB = (Triangle) triA.Edge(hEdgeAIdx).Twin.Poly;

                var twinA0 = triA.EdgeWithOrigin(vAB1Idx).Twin;
                var twinA2 = triB.EdgeWithOrigin(vB2Idx);

                var twinB0 = triB.EdgeWithOrigin(vAB0Idx);
                var twinB2 = triA.EdgeWithOrigin(vA2Idx);

                var nvA0 = vAB1Idx;
                var nvA1 = vA2Idx;

                var nvB0 = vAB0Idx;
                var nvB1 = vB2Idx;

                triA.Reform(nvA0, nvA1, nvB1);
                triB.Reform(nvB0, nvB1, nvA1);

                triA.Edge(0).Twin = twinA0;
                triA.Edge(2).Twin = twinA2;
                triB.Edge(0).Twin = twinB0;
                triB.Edge(2).Twin = twinB2;
                triA.Edge(1).Twin = triB.Edge(1);

                if (triA.Edge(0).Twin != null)
                    outerEdges.Add(triA.Edge(0));
                if (triA.Edge(2).Twin != null)
                    outerEdges.Add(triA.Edge(2));
                if (triB.Edge(0).Twin != null)
                    outerEdges.Add(triB.Edge(0));
                if (triB.Edge(2).Twin != null)
                    outerEdges.Add(triB.Edge(2));
                _outerEdgesWTwin = outerEdges.ToArray();

                var affectedVertIdxs = new HashSet<int>
                {
                    triA.Edge(0).OriginIdx,
                    triA.Edge(1).OriginIdx,
                    triA.Edge(2).OriginIdx,
                    triB.Edge(0).OriginIdx
                };

                return affectedVertIdxs;
            }
        }
    }
}