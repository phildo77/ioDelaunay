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

            public List<Triangle.TriEdgeData> FlipEdgeOld(Triangle.TriEdgeData _edgeData)
            {
                var triA = (Triangle) _edgeData.Edge.Poly;
                var triB = (Triangle) _edgeData.Edge.Twin.Poly;

                triA.FlipEdgeOld(_edgeData);
                
                var outerEdges = new List<Triangle.TriEdgeData>
                {
                    triB.EdgeDataWithOrigin(_edgeData.vAB1Idx),
                    triA.EdgeDataWithOrigin(_edgeData.vA2Idx),
                    triA.EdgeDataWithOrigin(_edgeData.vAB0Idx),
                    triB.EdgeDataWithOrigin(_edgeData.vB2Idx)
                };
                foreach (var edge in outerEdges)
                    RefreshEdge(edge);
                
                return outerEdges;
            }
            
            public Delaunay D { get; }
            
        }


        private EdgeStack m_EdgeStack;

        public void LegalizeOld(int _startTriID)
        {
            var tri = (Triangle)m_Polys[_startTriID];
            foreach (var hEdge in tri.EdgeData)
                m_EdgeStack.Push(hEdge);

            while (!m_EdgeStack.IsEmpty)
            {
                var edgeStackObj = m_EdgeStack.Pop();
                if (edgeStackObj.IsDelaunay) continue;
                var outerEdges = m_EdgeStack.FlipEdgeOld(edgeStackObj);
                foreach (var oEdge in outerEdges)
                    m_EdgeStack.Push(oEdge);
            }
        }



        private Stack<Poly.HalfEdge> m_EdgeStack2 = new Stack<Poly.HalfEdge>();
        //private HashSet<Poly.HalfEdge> m_EdgesPending = new HashSet<Poly.HalfEdge>();
        public void Legalize(params Poly.HalfEdge[] _edges)
        {
            foreach (var edge in _edges)
                if(edge.Twin != null)
                    m_EdgeStack2.Push(edge);
            
            while (m_EdgeStack2.Count != 0)
            {
                var curEdge = m_EdgeStack2.Pop();
                //m_EdgesPending.Remove(curEdge);
                if (curEdge.IsDelaunay()) continue;
                var oEdges = curEdge.FlipEdge();
                foreach(var oEdge in oEdges)
                    if (oEdge.Twin != null)
                        m_EdgeStack2.Push(oEdge);
                            
            }
        }
        

    }
}