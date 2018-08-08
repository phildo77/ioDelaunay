using System.Collections.Generic;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        private Stack<Triangle.HalfEdge> m_EdgeStack = new Stack<Triangle.HalfEdge>();
        private Triangle.HalfEdge[] m_RefOuterEdges = new Triangle.HalfEdge[4];
        
        /// <summary>
        /// Iterative legalization - Assumes all edges have twins
        /// </summary>
        /// <param name="_edges"></param>
        public void Legalize(params Triangle.HalfEdge[] _edges)
        {
            for (int eIdx = 0; eIdx < _edges.Length; ++eIdx)
            {
                m_EdgeStack.Push(_edges[eIdx]);
            }
            
            while (m_EdgeStack.Count != 0)
            {
                var curEdgeA0 = m_EdgeStack.Pop();
                var curTwinB0 = curEdgeA0.m_Twin;
                var pts = curEdgeA0.D.Points;
                var triA = curEdgeA0.Triangle;
                var triB = curTwinB0.Triangle;
                //Get edges quickly
                var newEB2 = curEdgeA0.NextEdge;
                var newEA1 = newEB2.NextEdge;
                var newEA2 = curTwinB0.NextEdge;
                var newEB1 = newEA2.NextEdge;

                var a2Idx = newEA1.OriginIdx;
                var b2Idx = newEB1.OriginIdx;
                
                var a2 = pts[a2Idx];
                var b2 = pts[b2Idx];
                //Check if legal local Delaunay
                {
            
                    //var a2 = pts[curEdge.NextEdge.NextEdge.OriginIdx];
                    //var b2 = pts[curEdge.Twin.NextEdge.NextEdge.OriginIdx];

                    
                    if (a2.SqrMagFast(triB.CCX, triB.CCY) >= triB.CCRSq &&
                        b2.SqrMagFast(triA.CCX, triA.CCY) >= triA.CCRSq)
                        continue;
                }

                //Flip edge
                {
                
                    

                    curEdgeA0.OriginIdx = b2Idx;
                    curTwinB0.OriginIdx = a2Idx;
            
                
                    curEdgeA0.NextEdge = newEA1;
                    newEA1.NextEdge = newEA2;
                    newEA2.NextEdge = curEdgeA0;

                
                    curTwinB0.NextEdge = newEB1;
                    newEB1.NextEdge = newEB2;
                    newEB2.NextEdge = curTwinB0;
                
                    newEA2.Triangle = triA;
                    newEB2.Triangle = triB;
                
                    triA.Edge0 = curEdgeA0;
                    triA.Edge1 = newEA1;
                    triA.Edge2 = newEA2;
                    triB.Edge0 = curTwinB0;
                    triB.Edge1 = newEB1;
                    triB.Edge2 = newEB2;

                    m_RefOuterEdges[0] = newEA1;
                    m_RefOuterEdges[1] = newEA2;
                    m_RefOuterEdges[2] = newEB1;
                    m_RefOuterEdges[3] = newEB2;


                    Geom.Circumcircle(pts[curEdgeA0.OriginIdx], pts[newEA1.OriginIdx], pts[newEA2.OriginIdx],
                        out triA.CCX, out triA.CCY, out triA.CCRSq);

                    Geom.Circumcircle(pts[curTwinB0.OriginIdx], pts[newEB1.OriginIdx], pts[newEB2.OriginIdx],
                        out triB.CCX, out triB.CCY, out triB.CCRSq);
                }
                
                
                //if (curEdge.IsDelaunay()) continue;
                //curEdgeA0.FlipEdge(ref m_RefOuterEdges);
                foreach(var oEdge in m_RefOuterEdges)
                    if (oEdge.m_Twin != null)
                        m_EdgeStack.Push(oEdge);
                            
            }
        }
        

    }
}