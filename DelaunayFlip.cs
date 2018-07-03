using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Diagnostics;
using System.Dynamic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        private class EdgeStack : IDelaunayObj
        {

            private List<Triangle.TriEdgeData> m_KeyStack;
            private Dictionary<Triangle.TriEdgeData, Triangle.TriEdgeData> m_Edges;
            
            public EdgeStack(Delaunay _d)
            {
                D = _d;
                m_Edges = new Dictionary<Triangle.TriEdgeData, Triangle.TriEdgeData>(new EdgeComparer());
                m_KeyStack = new List<Triangle.TriEdgeData>();
            }
            

            private class EdgeComparer : IEqualityComparer<Triangle.TriEdgeData>
            {
                public bool Equals(Triangle.TriEdgeData _x, Triangle.TriEdgeData _y)
                {
                    return (_x.vAB0Idx == _y.vAB1Idx && _x.vAB1Idx == _y.vAB0Idx) ||
                           (_x.vAB0Idx == _y.vAB0Idx && _x.vAB1Idx == _y.vAB1Idx);
                }

                public int GetHashCode(Triangle.TriEdgeData _obj)
                {
                    var ha = _obj.vAB0Idx;
                    var hb = _obj.vAB1Idx;
                    if (hb < ha)
                    {
                        var temp = ha;
                        ha = hb;
                        hb = temp;
                    }
                    return ha.GetHashCode() ^ (hb.GetHashCode() << 2);
                }
            }
            
            public void Push(Triangle.TriEdgeData _edge)
            {
                if (_edge.Edge.Twin == null) return;
                if (m_Edges.ContainsKey(_edge)) return;
                m_KeyStack.Add(_edge);
                m_Edges.Add(_edge, _edge);
            }

            public Triangle.TriEdgeData Pop()
            {
                var key = m_KeyStack[m_KeyStack.Count - 1];
                m_KeyStack.RemoveAt(m_KeyStack.Count - 1);
                var edge = m_Edges[key];
                m_Edges.Remove(key);
                return edge;
            }

            private void RefreshEdge(Triangle.TriEdgeData _edge)
            {
                /* DEBUG
                if(m_Edges.Count != 0)
                    Console.WriteLine("Debug");
                */
                if (!m_Edges.ContainsKey(_edge)) return;
                m_Edges[_edge] = _edge;
            }
            
            public bool IsEmpty => m_KeyStack.Count == 0;

            public List<Triangle.TriEdgeData> Flip(Triangle.TriEdgeData _edgeData)
            {
                var triA = (Triangle)_edgeData.Edge.Poly;
                var triB = (Triangle)_edgeData.Edge.Twin.Poly;
                var triAID = triA.ID;
                var triBID = triB.ID;
                var a2Idx = _edgeData.vA2Idx;
                var b2Idx = _edgeData.vB2Idx;
                var ab0Idx = _edgeData.vAB0Idx;
                var ab1Idx = _edgeData.vAB1Idx;
                var newVertsA = new[]
                {
                    a2Idx, b2Idx, ab1Idx
                };
                var newVertsB = new[]
                {
                    b2Idx, a2Idx, ab0Idx
                };
                    
                D.RemovePoly(triAID);
                D.RemovePoly(triBID);

                triA = new Triangle(newVertsA[0], newVertsA[1], newVertsA[2], D);
                triB = new Triangle(newVertsB[0], newVertsB[1], newVertsB[2], D);

                var outerEdges = new List<Triangle.TriEdgeData>
                {
                    triA.EdgeDataWithOrigin(ab1Idx),
                    triB.EdgeDataWithOrigin(a2Idx),
                    triB.EdgeDataWithOrigin(ab0Idx),
                    triA.EdgeDataWithOrigin(b2Idx)
                };

                foreach (var edge in outerEdges)
                    RefreshEdge(edge);
                
                return outerEdges;
            }

            public Delaunay D { get; }
        }
            
            
        public HashSet<int> Legalize(Guid _startTriID)
        {
            var eStack = new EdgeStack(this);
            var affectedVerts = new HashSet<int>();
            var tri = (Triangle)m_Polys[_startTriID];
            foreach (var hEdge in tri.EdgeData)
                eStack.Push(hEdge);

            while (!eStack.IsEmpty)
            {
                var edgeStackObj = eStack.Pop();
                if (edgeStackObj.IsDelaunay) continue;
                var outerEdges = eStack.Flip(edgeStackObj);
                foreach (var oEdge in outerEdges)
                {
                    affectedVerts.Add(oEdge.Edge.OriginIdx);
                    eStack.Push(oEdge);
                }
                    
            }

            return affectedVerts;
        }
        


    }
}