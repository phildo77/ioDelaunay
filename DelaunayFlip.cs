﻿using System.Collections.Generic;
using System.Diagnostics;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        private Stack<Triangle.HalfEdge> m_EdgeStack = new Stack<Triangle.HalfEdge>();
        private Triangle.HalfEdge[] m_RefOuterEdges = new Triangle.HalfEdge[4];


        public static bool enableVis = false; //TODO DEBUG
        public static List<Triangle> debugTris = null;
        private float m_MinError;
        
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

                /*
                if (enableVis && debugTris == null) //TODO DEBUG
                {
                    var dta = curEdgeA0.Triangle;
                    var dtb = curEdgeA0.m_Twin.Triangle;
                    //var dtc = curEdgeA0.NextEdge.m_Twin.Triangle;
                    //var dtd = curEdgeA0.NextEdge.NextEdge.m_Twin.Triangle;
                    var dtf = curEdgeA0.m_Twin.NextEdge.m_Twin.Triangle;
                    var dtg = curEdgeA0.m_Twin.NextEdge.NextEdge.m_Twin.Triangle;
                    debugTris = new List<Triangle> {dta,dtb,dtf,dtg};
                }
                */
                
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

                    
                    
                    var a2toCCB = a2.SqrMagFast(triB.CCX, triB.CCY);
                    var b2toCCA = b2.SqrMagFast(triA.CCX, triA.CCY);
                    if (a2toCCB >= triB.CCRSq && b2toCCA >= triA.CCRSq)
                        continue;

                    if (AreCocircular(curEdgeA0))
                    {
                        //TODO DEBUG
                        DebugVisualizer.LogTri(debugCircIter++ + "circDebug", new [] {triA, triB});

                        if(SolveDegenerate(curEdgeA0))
                            continue;
                    }
                        
                    
                    //Debug TODO
                    if (Geom.AreColinear(triA.Edge0.OriginPos, triA.Edge1.OriginPos, triA.Edge2.OriginPos))
                        Trace.WriteLine("Debug line");
                    if (Geom.AreColinear(triB.Edge0.OriginPos, triB.Edge1.OriginPos, triB.Edge2.OriginPos))
                        Trace.WriteLine("Debug line");
                    
                }

                //TODO DEBUG
                if (enableVis && m_EdgeStack.Count > 1000)
                {
                    DebugVisualizer.Visualize("HighStackCountPre", curEdgeA0);
                    
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
                
                //TODO DEBUG
                if (enableVis && m_EdgeStack.Count > 1000)
                {
                    DebugVisualizer.Visualize("HighStackCountPost", curEdgeA0);
                    
                }
                //if (curEdge.IsDelaunay()) continue;
                //curEdgeA0.FlipEdge(ref m_RefOuterEdges);
                foreach(var oEdge in m_RefOuterEdges)
                    if (oEdge.m_Twin != null)
                        m_EdgeStack.Push(oEdge);
                            
            }
            
            
        }


        private float n;

        private float[,] rxy = new float[2, 2];
        private float r11 = 0f;
        private float r12 = 0f;
        private float r21 = 0f;
        private float r22 = 0f;
        
        private bool SolveDegenerate(Triangle.HalfEdge _edge)
        {
            var pts = _edge.D.Points;
            var ab0 = pts[_edge.OriginIdx];
            var ab1 = pts[_edge.NextEdge.OriginIdx];
            var a2 = pts[_edge.NextEdge.NextEdge.OriginIdx];
            var b2 = pts[_edge.m_Twin.NextEdge.NextEdge.OriginIdx];

            var nab0 = RndLinXfrm(ab0);
            var nab1 = RndLinXfrm(ab1);
            var na2 = RndLinXfrm(a2);
            var nb2 = RndLinXfrm(b2);

            float tACCX, tACCY, tACCRSq, tBCCX, tBCCY, tBCCRSq;
            
            Geom.Circumcircle(nab0, nab1, na2, out tACCX, out tACCY, out tACCRSq);

            Geom.Circumcircle(nab1, nab0, nb2, out tBCCX, out tBCCY, out tBCCRSq); 
            
            var a2toCCB = na2.SqrMagFast(tBCCX, tBCCY);
            var b2toCCA = nb2.SqrMagFast(tACCX, tACCY);
            return a2toCCB >= tBCCRSq && b2toCCA >= tACCRSq;
        }

        //TODO DEBUG
        private int debugCircIter = 0;
        private bool AreCocircular(Triangle.HalfEdge _edge)
        {
            
            var triA = _edge.Triangle;
            var triB = _edge.m_Twin.Triangle;

            var ccxChk = triA.CCX.ApproxEqual(triB.CCX, m_MinError);
            var ccyChk = triA.CCY.ApproxEqual(triB.CCY, m_MinError);
            var radChk = triA.CCRSq.ApproxEqual(triB.CCRSq, 0.1f);
            var debugChk = ccxChk && ccyChk && radChk;
            
            //TODO DEBUG
            if(enableVis)
                DebugVisualizer.LogTri(debugCircIter++ + "circDebug", new [] {triA, triB}, " Cocir Check: " + debugChk);

                
            return ccxChk && ccyChk && radChk;
        }

        private Vector2f RndLinXfrm(Vector2f _vec)
        {
            var x = _vec.x;
            var y = _vec.y;

            var nx = (x * (1 + n * r11)) + (n * r21);
            var ny = (x * (n * r12) + (y * (1 + n * r22)));

            return new Vector2f(nx, ny);
        }
        
        
        

    }
}