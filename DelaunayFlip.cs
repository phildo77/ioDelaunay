using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Dynamic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay
    {

        public HashSet<int> LegalizeAll()
        {
            var tris = Triangles;
            var eMarked = new HashSet<FEdge>(new FEdge.FEdgeComparer());
            var eStack = new Stack<FEdge>();
            var affectedVerts = new HashSet<int>();

            foreach (var tri in tris)
                foreach(var hEdge in tri.Edges)
                    if (hEdge.Twin != null)
                    {
                        var fEdge = new FEdge(hEdge, this);
                        if (eMarked.Contains(fEdge)) continue;
                        eMarked.Add(fEdge);
                        eStack.Push(fEdge);
                    }

            var debugIter = 0; //TODO
            while (eStack.Count != 0)
            {
                var fEdge = eStack.Pop();
                eMarked.Remove(fEdge);
                if (!fEdge.Exists) continue;
                if (IsDelaunay(fEdge)) continue;
                FEdge[] _outerEdges = null;
                if(debugIter == 26)
                    Console.WriteLine("Debug"); //TODO
                affectedVerts.UnionWith(fEdge.Flip(out _outerEdges));
                //Debug
                {
                    foreach (var dfe in eStack)
                    {
                        if(dfe.vA2Idx == dfe.vB2Idx)
                            Console.WriteLine("Debug");
                    }
                }
                DebugVisualizer.Visualize(this, null, "legal" + debugIter++); //TODO
                foreach (var oEdge in _outerEdges)
                {
                    var debugHC = oEdge.GetHashCode();
                    Console.WriteLine(debugHC.ToString());
                    if (!eMarked.Contains(oEdge))
                    {
                        {//TODO DEBUG
                            foreach (var dedge in eStack)
                            {
                                if((oEdge.vAB0Idx == dedge.vAB0Idx && oEdge.vAB1Idx == dedge.vAB1Idx) ||
                                   (oEdge.vAB0Idx == dedge.vAB1Idx && oEdge.vAB1Idx == dedge.vAB0Idx))
                                    Console.WriteLine("Debug");
                            }
                        }
                        eMarked.Add(oEdge);
                        eStack.Push(oEdge);
                        
                    } 
                }
                    
            }

            return affectedVerts;
        }
        
        public HashSet<int> Legalize(Guid _startTriID)
        {
            var eStack = new Stack<FEdge>();
            var eMarked = new HashSet<FEdge>(new FEdge.FEdgeComparer());
            var affectedVerts = new HashSet<int>();
            foreach (var hEdge in m_Polys[_startTriID].Edges)
                if (hEdge.Twin != null)
                {
                    var fEdge = new FEdge(hEdge, this);
                    eMarked.Add(fEdge);
                    eStack.Push(fEdge);
                }

            while (eStack.Count != 0)
            {
                var fEdge = eStack.Pop();
                eMarked.Remove(fEdge);
                if (!fEdge.Exists) continue;
                if (IsDelaunay(fEdge)) continue;
                FEdge[] _outerEdges;
                affectedVerts.UnionWith(fEdge.Flip(out _outerEdges));
                foreach (var oEdge in _outerEdges)
                {
                    var debugHC = oEdge.GetHashCode();
                    Console.WriteLine(debugHC.ToString());
                    if (!eMarked.Contains(oEdge))
                    {
                        eMarked.Add(oEdge);
                        eStack.Push(oEdge);
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

            var distToCentSqr = (vB2.Pos - ccCent).sqrMagnitude;
            if (!(distToCentSqr >= ccRad * ccRad)) 
                return false;
            
            if (!Geom.Circumcircle(vB2.Pos, vAB1.Pos, vAB0.Pos, out ccCent, out ccRad))
                throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

            return (vA2.Pos - ccCent).sqrMagnitude >= ccRad * ccRad;

        }

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
            public readonly int vAB0Idx;
            public readonly int vAB1Idx;
            public int vA2Idx => EdgeAB0.NextEdge.NextEdge.OriginIdx;
            public int vB2Idx => TwinAB0.NextEdge.NextEdge.OriginIdx;

            public FEdge(Poly.HalfEdge _he, Delaunay _d)
            {
                D = _d;
                vAB0Idx = _he.OriginIdx;
                vAB1Idx = _he.NextEdge.OriginIdx;
            }

            public class FEdgeComparer : IEqualityComparer<FEdge>
            {
                public bool Equals(FEdge _x, FEdge _y)
                {
                    return (_x.vAB0Idx == _y.vAB1Idx && _x.vAB1Idx == _y.vAB0Idx) ||
                        (_x.vAB0Idx == _y.vAB0Idx && _x.vAB1Idx == _y.vAB1Idx);
                }

                public int GetHashCode(FEdge _obj)
                {
                    return _obj.GetHashCode();
                }
            }
            
            public bool Exists
            {
                get
                {
                    var contTris = D.m_PolysContainingVert[vAB0Idx].Intersect(D.m_PolysContainingVert[vAB1Idx]).ToArray();
                    return contTris.Length != 0;

                }
            }
            
            public Triangle TriA
            {
                get
                {
                    var contTris = D.m_PolysContainingVert[vAB0Idx].Intersect(D.m_PolysContainingVert[vAB1Idx]).ToArray();
                    if(contTris.Length != 2) //TODO DEBUG
                        Console.WriteLine("No Twin");
                    var tri = (Triangle)D.m_Polys[contTris[0]];
                    var edgeWAB0Idx = tri.EdgeWithOrigin(vAB0Idx);
                    if (edgeWAB0Idx.NextEdge.OriginIdx != vAB1Idx)
                        tri = (Triangle)edgeWAB0Idx.NextEdge.NextEdge.Twin.Poly;
                    return tri;
                }
            }

            public bool HasTwin => TriA.EdgeWithOrigin(vAB0Idx).Twin != null;
            public Triangle TriB => (Triangle) (HasTwin ? D.m_Polys[TwinAB0.PolyID] : null);
            private Poly.HalfEdge TwinAB0 => EdgeAB0.Twin;
            public Poly.HalfEdge EdgeAB0 => TriA.EdgeWithOrigin(vAB0Idx);

            public Delaunay D { get; }

            public override int GetHashCode()
            {
                var ha = vAB0Idx;
                var hb = vAB1Idx;
                if (hb < ha)
                {
                    var temp = hb;
                    hb = ha;
                    ha = temp;
                }

                return ha.GetHashCode() ^ (hb.GetHashCode() << 2);

            }

            public HashSet<int> Flip(out FEdge[] _outerEdgesWTwin)
            {
                var outerEdges = new List<FEdge>();
                var triA = TriA;
                var triB = (Triangle) triA.EdgeWithOrigin(vAB0Idx).Twin.Poly;

                
                
                var newA1Twin = triA.EdgeWithOrigin(vAB1Idx).Twin;
                var newA0Twin = triB.EdgeWithOrigin(vB2Idx).Twin;

                var newB0Twin = triA.EdgeWithOrigin(vA2Idx).Twin;
                var newB1Twin = triB.EdgeWithOrigin(vAB0Idx).Twin;

                var nvA0Idx = vB2Idx;
                var nvB0Idx = vA2Idx;

                var nvA1Idx = vAB1Idx;
                var nvA2Idx = vA2Idx;

                var nvB1Idx = vAB0Idx;
                var nvB2Idx = vB2Idx;

                //FEdge unsafe after this
                triA.Reform(nvA0Idx, nvA1Idx, nvA2Idx);
                triB.Reform(nvB0Idx, nvB1Idx, nvB2Idx);

                triA.Edge(0).Twin = newA0Twin;
                triA.Edge(1).Twin = newA1Twin;
                triB.Edge(0).Twin = newB0Twin;
                triB.Edge(1).Twin = newB1Twin;
                triA.Edge(2).Twin = triB.Edge(2);

                if (triA.Edge(0).Twin != null)
                    outerEdges.Add(new FEdge(triA.Edge(0), D));
                if (triA.Edge(1).Twin != null)
                    outerEdges.Add(new FEdge(triA.Edge(1), D)); 
                if (triB.Edge(0).Twin != null)
                    outerEdges.Add(new FEdge(triB.Edge(0), D)); 
                if (triB.Edge(1).Twin != null)
                    outerEdges.Add(new FEdge(triB.Edge(1), D)); 
                _outerEdgesWTwin = outerEdges.ToArray();

                var affectedVertIdxs = new HashSet<int>
                {
                    triA.Edge(0).OriginIdx,
                    triA.Edge(1).OriginIdx,
                    triB.Edge(0).OriginIdx,
                    triB.Edge(1).OriginIdx
                };

                return affectedVertIdxs;
            }
        }
    }
}