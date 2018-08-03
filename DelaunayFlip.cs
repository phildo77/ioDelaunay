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
        private Stack<Poly.HalfEdge> m_EdgeStack = new Stack<Poly.HalfEdge>();

        private Poly.HalfEdge[] m_RefOuterEdges = new Poly.HalfEdge[4];
        //private HashSet<Poly.HalfEdge> m_EdgesPending = new HashSet<Poly.HalfEdge>();
        public void Legalize(params Poly.HalfEdge[] _edges)
        {
            foreach (var edge in _edges)
                if(edge.Twin != null)
                    m_EdgeStack.Push(edge);
            
            while (m_EdgeStack.Count != 0)
            {
                var curEdge = m_EdgeStack.Pop();
                //m_EdgesPending.Remove(curEdge);
                if (curEdge.IsDelaunay()) continue;
                curEdge.FlipEdge(ref m_RefOuterEdges);
                foreach(var oEdge in m_RefOuterEdges)
                    if (oEdge.Twin != null)
                        m_EdgeStack.Push(oEdge);
                            
            }
        }
        

    }
}