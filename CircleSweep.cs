using System.Diagnostics;
using System.Runtime.ConstrainedExecution;

namespace ioDelaunay
{
    using System;
    using System.Collections.Generic;
    using System.Linq;

    using Vectorf;

    public class CircleSweep : DelaunayBase
    {
        // Don't change
        private readonly Vector2f m_Origin; //By Point Idx
        private readonly PolartPt[] m_PolPos; //By Point Idx
        private readonly int[] m_VertIdxsByR;

        // Change
        private Frontier m_Frontier;

        public Vector2f Origin => m_Origin;
        
        public CircleSweep(Vector2f[] _points)
            : base(_points)
        {
            //Init
            {
                m_Origin = CalcOrigin();
                m_PolPos = m_Points.Select(_pt => new PolartPt(_pt, m_Origin, this)).ToArray();

                m_VertIdxsByR = new int[m_Vertices.Length];
                var tempVertIdxs = new List<int>();
                for (int idx = 0; idx < m_Vertices.Length; ++idx)
                    tempVertIdxs.Add(idx);

                m_VertIdxsByR = tempVertIdxs.OrderBy(_idx => m_PolPos[_idx].r).ToArray();
                
                #if DEBUG
                var debugR = new Dictionary<Vector2f, float>();
                for (int idx = 0; idx < m_VertIdxsByR.Length; ++idx)
                {
                    var rIdx = m_VertIdxsByR[idx];
                    debugR.Add(m_Vertices[idx].Pos, m_PolPos[rIdx].r);
                }
                Console.WriteLine(debugR.ToString());
                #endif
                
                //Init Frontier
                var triVertIdxs = new[] {m_VertIdxsByR[0], m_VertIdxsByR[1], m_VertIdxsByR[2]};

                var firstTri = new Triangle(triVertIdxs, this);

                m_Frontier = new Frontier(firstTri, this);

            }
        }

        protected void Legalize(Triangle _triA)
        {
            /*
            foreach (var edge in _triA.HalfEdges)
            {
                if (edge.Twin == null) continue;
                var vA2 = edge.NextEdge.NextEdge.Origin;
                var vAB0 = edge.Origin;
                var vAB1 = edge.NextEdge.Origin;
                var vB2 = edge.Twin.NextEdge.NextEdge.Origin;
                Vector2f ccCent;
                float ccRad;

                if(!Geom.Circumcircle(vA2.Pos, vAB0.Pos, vAB1.Pos, out ccCent, out ccRad))
                    throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                var checkDistSqr = (vB2.Pos - ccCent).sqrMagnitude;
                if (checkDistSqr >= ccRad) continue;
                
                //Flip Edge
                var eA0 = 
            }
            */
        }
        
        protected override void Algorithm()
        {
            for (int rIdx = 3; rIdx < m_VertIdxsByR.Length; ++rIdx)
            {
                var debug1 = m_Frontier.DebugScanForTwin();
                if (debug1.Count != 0)
                    Console.WriteLine(debug1.ToString());
                var ofVertIdx = m_VertIdxsByR[rIdx];
                var ofVert = m_Vertices[ofVertIdx];

                // 1) Project
                var thetaHit = PolPos(ofVert.Idx).Theta;
                var fntVerts = m_Frontier.Project(thetaHit);

                // 2) Create Tri and Legalize
                debug1 = m_Frontier.DebugScanForTwin();
                if (debug1.Count != 0)
                    Console.WriteLine(debug1.ToString());
                var tri = AddTriToMesh(ofVertIdx, fntVerts[0].EdgeRight);
                var newFntPt = m_Frontier.Add(ofVert, tri, fntVerts);
                
                // TODO Legalize Tri (Recurse)
                
                // 3) Walk Left
                
                while (true)
                {
                    debug1 = m_Frontier.DebugScanForTwin();
                    if (debug1.Count != 0)
                        Console.WriteLine(debug1.ToString());
                    var fntC = newFntPt.Left;
                    var fntL = fntC.Left;
                    var vecR = newFntPt.Vert.Pos - fntC.Vert.Pos;
                    var vecL = fntL.Vert.Pos - fntC.Vert.Pos;
                    var angle = Vector2f.SignedAngle(vecL, vecR);
                    debug1 = m_Frontier.DebugScanForTwin();
                    if (debug1.Count != 0)
                        Console.WriteLine(debug1.ToString());
                    if ((angle >= 90f) || (angle < 0d)) break;
                    tri = AddTriToMesh(fntL.EdgeRight, fntC.EdgeRight);
                    m_Frontier.Remove(fntC, tri.GetHalfEdgeWithOrigin(fntL.VertIdx));
                    
                    debug1 = m_Frontier.DebugScanForTwin();
                    if (debug1.Count != 0)
                        Console.WriteLine(debug1.ToString());
                }
                // 4) Walk Right
                while (true)
                {
                    
                    debug1 = m_Frontier.DebugScanForTwin();
                    if (debug1.Count != 0)
                        Console.WriteLine(debug1.ToString());
                    var fntC = newFntPt.Right;
                    var fntR = fntC.Right;
                    var vecR = fntR.Vert.Pos - fntC.Vert.Pos;
                    var vecL = newFntPt.Vert.Pos - fntC.Vert.Pos;
                    var angle = Vector2f.SignedAngle(vecL, vecR);
                    
                    debug1 = m_Frontier.DebugScanForTwin();
                    if (debug1.Count != 0)
                        Console.WriteLine(debug1.ToString());
                    if ((angle >= 90f) || (angle < 0f)) break;
                    tri = AddTriToMesh(newFntPt.EdgeRight, fntC.EdgeRight);
                    m_Frontier.Remove(fntC, tri.GetHalfEdgeWithOrigin(newFntPt.VertIdx));
                    
                    debug1 = m_Frontier.DebugScanForTwin();
                    if (debug1.Count != 0)
                        Console.WriteLine(debug1.ToString());
                }
            }
        }

        public Vector2f[] DebugFirstTri;
        private Vector2f CalcOrigin()
        {
            var m_BoundsRect = new Rectf(m_Points[0].x, m_Points[0].y, 0, 0);
            foreach (var pt in m_Points)
                m_BoundsRect.Encapsulate(pt);
            var cent = m_BoundsRect.center;
            var closest = new SortedList<float, int>();
            for(int idx = 0; idx < m_Points.Length; ++idx)
            {
                var pt = m_Points[idx];
                var distSqr = (cent - pt).sqrMagnitude;
                if (closest.Count <= 3)
                {
                    closest.Add(distSqr, idx);
                    continue;
                }

                if (distSqr > closest.Keys[2])
                    continue;

                closest.Add(distSqr, idx);
            }

            var firstTriIdxs = new[] {closest.Values[0], closest.Values[1], closest.Values[2]};
            var triPts = new[] {m_Points[firstTriIdxs[0]], m_Points[firstTriIdxs[1]], m_Points[firstTriIdxs[2]]};
            DebugFirstTri = triPts;
            var cc = Geom.CentroidOfPoly(triPts);
            return cc;
        }

        private Vertex[] GetVerts(int[] _vertIdxs)
        {
            return _vertIdxs.Select(_idx => m_Vertices[_idx]).ToArray();
        }

        private PolartPt PolPos(int _vertIdx)
        {
            return m_PolPos[_vertIdx];
        }

        private PolartPt PolPos(Vertex _vert)
        {
            return m_PolPos[_vert.Idx];
        }

        protected class DCircleSweepObj : DelaunayObj
        {
            protected readonly CircleSweep CS;

            protected DCircleSweepObj(CircleSweep _cs)
                : base(_cs)
            {
                CS = _cs;
            }
        }

        private class Frontier : DCircleSweepObj
        {
            private SortedList<float, FrontierPt> m_Frontier; //By Theta

            public Frontier(Triangle _firstTri, CircleSweep _cs)
                : base(_cs)
            {
                m_Frontier = new SortedList<float, FrontierPt>();
                var verts = _firstTri.VertIdxs.Select(_id => CS.m_Vertices[_id]).ToArray();

                var ftPts = _firstTri.Verts.Select(_vert => new FrontierPt(_vert.Idx, _firstTri.GetHalfEdgeWithOrigin(_vert.Idx), CS)).ToArray();

                ftPts[0].Right = ftPts[1];
                ftPts[0].Left = ftPts[2];
                ftPts[1].Right = ftPts[2];
                ftPts[1].Left = ftPts[0];
                ftPts[2].Right = ftPts[0];
                ftPts[2].Left = ftPts[1];

                Console.WriteLine(ftPts[2].ToString());

                for (int idx = 0; idx < 3; ++idx) //TODO handle same theta
                    m_Frontier.Add(CS.PolPos(verts[idx]).Theta, ftPts[idx]);
            }

            public List<FrontierPt> DebugScanForTwin()
            {
                var dsft = new List<FrontierPt>();
                foreach (var fPt in m_Frontier)
                {
                    if (fPt.Value.EdgeRight.Twin != null)
                        dsft.Add(fPt.Value);
                }

                return dsft;
            }

            public HashSet<FrontierPt> DebugScanForOrder()
            {
                var problems = new HashSet<FrontierPt>();
                for (int idx = 0; idx < m_Frontier.Count; ++idx)
                {
                    var curFPt = m_Frontier.Values[idx];
                    var ltIdx = idx == 0 ? m_Frontier.Count - 1 : idx - 1;
                    var rtIdx = idx == m_Frontier.Count - 1 ? 0 : idx + 1;

                    var ltFpt = m_Frontier.Values[ltIdx];
                    var rtFpt = m_Frontier.Values[rtIdx];
                    
                    if (ltFpt != curFPt.Left)
                    {
                        problems.Add(curFPt);
                        problems.Add(ltFpt);
                    }

                    if (rtFpt != curFPt.Right)
                    {
                        problems.Add(curFPt);
                        problems.Add(rtFpt);
                    }


                }
                
                return problems;
            }
            
            public FrontierPt Add(Vertex _newVert, Triangle _newTri, FrontierPt[] _between)
            {
                var debug1 = DebugScanForTwin();
                if (debug1.Count != 0)
                    Console.WriteLine(debug1.ToString());


                var debug2 = DebugScanForOrder();
                if(debug2.Count != 0)
                    Console.WriteLine(debug2.ToString());
                var _newEdge = _newTri.GetHalfEdgeWithOrigin(_newVert.Idx);
                var ftPt = new FrontierPt(_newVert.Idx, _newEdge, CS);

                

                var ftLt = _between[0];
                var ftRt = _between[1];

                ftLt.Right = ftRt.Left = ftPt;
                ftLt.EdgeRight = _newTri.GetHalfEdgeWithOrigin(ftLt.VertIdx);

                ftPt.Left = ftLt;
                ftPt.Right = ftRt;
                
                m_Frontier.Add(CS.PolPos(_newVert).Theta, ftPt); //TODO add epsilon offset for dupe?
                
                debug1 = DebugScanForTwin();
                if (debug1.Count != 0)
                    Console.WriteLine(debug1.ToString());


                debug2 = DebugScanForOrder();
                if(debug2.Count != 0)
                    Console.WriteLine(debug2.ToString());
                return ftPt;
            }

            public void Remove(FrontierPt _fPt, HalfEdge _replEdgeLt)
            {
                var fpLeft = _fPt.Left;
                var fpRight = _fPt.Right;
                fpLeft.Right = fpRight;
                fpRight.Left = fpLeft;
                fpLeft.EdgeRight = _replEdgeLt;
                
                
                var debugCntPre = m_Frontier.Count;
                m_Frontier.Remove(CS.m_PolPos[_fPt.VertIdx].Theta);
                var debugCntPost = m_Frontier.Count;
                if(debugCntPre == debugCntPost)
                    Console.WriteLine("WTF");
                _fPt.Dispose();
                

            }

            public FrontierPt[] Project(float _theta)
            {
                var fpThetas = m_Frontier.Keys.ToList();
                var rtPtIdx = fpThetas.BinarySearch(_theta);
                //if(rtPtIdx == -1)
                //    Console.WriteLine("WTF DEBUG");
                if (rtPtIdx < 0)
                {
                    rtPtIdx = ~rtPtIdx;
                    if (rtPtIdx == fpThetas.Count)
                        rtPtIdx = 0;
                }

                var ltRt = new[] {m_Frontier.Values[rtPtIdx].Left, m_Frontier.Values[rtPtIdx]};
                
                //if(ltRt[0].Right != ltRt[1] || ltRt[1].Left != ltRt[0])
                //    Console.WriteLine("WTF DEBUG");
                
                return ltRt;
            }

            public class FrontierPt : DCircleSweepObj
            {
                public readonly int VertIdx;

                public HalfEdge EdgeRight
                {
                    get { return CS.m_Triangles[m_TriRightID].HalfEdges[m_EdgeRightIdx];}
                    set
                    {
                        m_TriRightID = value.TriID;
                        m_EdgeRightIdx = value.Tri.GetHalfEdgeIdxWithOrigin(value.OriginIdx);
                    }
                }
                public FrontierPt Left;
                public FrontierPt Right;
                public Vertex Vert => CS.m_Vertices[VertIdx];
                private Guid m_TriRightID;
                private int m_EdgeRightIdx;

                public FrontierPt(int vertIdx, HalfEdge _edgeRight, CircleSweep _cs)
                    : base(_cs)
                {
                    VertIdx = vertIdx;
                    EdgeRight = _edgeRight;
                }

                public void Dispose()
                {
                    Left = null;
                    Right = null;
                }

                public override string ToString()
                {
                    var v = (Vert ?? (Object) "").ToString();
                    var er = (EdgeRight ?? (Object) "").ToString();
                    return "Fnt Pt v: " + v + " RtV: " + Right.Vert.Idx + " LtV: " + Left.Vert.Idx + " EdgeRt: " + er;
                }
            }
        }

        private class PolartPt : DCircleSweepObj
        {
            public readonly float r;
            public readonly float Theta;

            public PolartPt(float _radius, float _theta, CircleSweep _cs)
                : base(_cs)
            {
                r = _radius;
                Theta = _theta;
            }

            public PolartPt(Vector2f _cPt, Vector2f _origin, CircleSweep _cs)
                : base(_cs)
            {
                var nx = _cPt.x - _origin.x;
                var ny = _cPt.y - _origin.y;

                r = (float) Math.Sqrt((nx * nx) + (ny * ny));
                Theta = (float) Math.Atan2(ny, nx);
            }
        }
    }
}