using System;
using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public class CircleSweep : DelaunayBase
    {

        // Don't change
        private readonly Vector2f m_Origin; //By Point Idx
        private readonly PolartPt[] m_PolPos; //By Point Idx
        private readonly int[] m_VertIdxsByR;
        private readonly Rectf m_BoundsRect;

        // Change
        private Frontier m_Frontier;

        public Vector2f Origin => m_Origin;
        public Rectf Bounds => new Rectf(m_BoundsRect);
        public Frontier DebugFrontier => m_Frontier; //TODO Remove
        
        public CircleSweep(Vector2f[] _points)
            : base(_points)
        {
            m_Origin = CalcOrigin(out m_BoundsRect);
            m_PolPos = m_Points.Select(_pt => new PolartPt(_pt, m_Origin, this)).ToArray();

            m_VertIdxsByR = new int[m_Vertices.Length];
            var tempVertIdxs = new List<int>();
            for (int idx = 0; idx < m_Vertices.Length; ++idx)
                tempVertIdxs.Add(idx);

            m_VertIdxsByR = tempVertIdxs.OrderBy(_idx => m_PolPos[_idx].r).ToArray();
            
            //Init Frontier
            var triVertIdxs = new[] {m_VertIdxsByR[0], m_VertIdxsByR[1], m_VertIdxsByR[2]};

            var firstTri = new Triangle(triVertIdxs, this);

            m_Frontier = new Frontier(firstTri, this);
            
        }

        protected void LegalizeFrontier(HashSet<int> _affectedVerts)
        {
            foreach (var vertIdx in _affectedVerts)
            {
                if (!m_Frontier.ContainsVert(vertIdx)) continue;
                var fPt = m_Frontier.GetHaving(vertIdx);
                var rtIdx = fPt.Right.VertIdx;

                
                var commonTris = GetTrisContainingAllVerts(fPt.VertIdx, rtIdx).ToArray();
                
                fPt.EdgeRight = m_Triangles[commonTris[0]].GetHalfEdgeWithOrigin(vertIdx);
            }
        }

        public HashSet<Guid> GetTrisContainingAllVerts(params Vertex[] _verts)
        {
            return GetTrisContainingAllVerts(_verts.Select(_vert => _vert.Idx).ToArray());
        }

        public HashSet<Guid> GetTrisContainingAllVerts(params int[] _vertIdxs)
        {
            var trisContaining = new HashSet<Guid>(m_TrisContainingVert[_vertIdxs[0]]);
            for(int idx = 1; idx < _vertIdxs.Length; ++idx)
                trisContaining.IntersectWith(m_TrisContainingVert[_vertIdxs[idx]]);
            return trisContaining;
        }

        public enum RL
        {
            Right, Left
        }

        private void Walk(RL _dir, int _viIdx)
        {
            var fvi = m_Frontier.GetHaving(_viIdx);

            while (true)
            {
                var fvn = fvi[_dir];

                var vecL = fvn[RL.Left].Vert.Pos - fvn.Vert.Pos;
                var vecR = fvn[RL.Right].Vert.Pos - fvn.Vert.Pos;

                var angleCW = AngleCW(vecL, vecR);
                if (angleCW > (Math.PI / 2f)) break;

                var twinLt = fvn[RL.Left].EdgeRight;
                var twinRt = fvn.EdgeRight;
                var newTri = AddTriToMesh(twinLt, twinRt);
                m_Frontier.Remove(fvn);
                var changedVerts = Legalize(newTri.ID, twinLt.TriID, twinRt.TriID);
                LegalizeFrontier(changedVerts);
            }
            
        }

        private void FillBasin(RL _dir, int _viIdx)
        {
            //Setup
            var iDir = _dir == RL.Right ? RL.Left : RL.Right;
            var fvi = m_Frontier.GetHaving(_viIdx);
            var fvrp = fvi[_dir][_dir];
            var polvi = PolPos(fvi.VertIdx);
            var polvrp = PolPos(fvrp.VertIdx);
            
            var r2 = polvrp.r;
            var dr = polvi.r - r2;
            var dTheta = _dir == RL.Right ? polvrp.Theta - polvi.Theta : polvi.Theta - polvrp.Theta;

            //Check heuristic
            var h = dr / (r2 * dTheta);
            if (h < 2) return;

            DebugVisualizer.Visualize(this, "PossibleBasinDetected");
            //Find Basin Min
            var fBasStart = fvrp;
            var fBasMin = fBasStart;
            while (PolPos(fBasMin.VertIdx).r > PolPos(fBasMin[_dir].VertIdx).r)
                fBasMin = fBasMin[_dir];
            
            //Find Basin End
            var fBasEnd = fBasMin;
            while (true)
            {
                var vecL = fBasEnd[RL.Left].Vert.Pos - fBasEnd.Vert.Pos;
                var vecR = fBasEnd[RL.Right].Vert.Pos - fBasEnd.Vert.Pos;
                var angleCW = AngleCW(vecL, vecR);
                if (angleCW >= (Math.PI - float.Epsilon)) break;
                fBasEnd = fBasEnd[_dir];
            }

            if (fBasEnd.VertIdx == fBasMin.VertIdx || fBasStart[_dir].VertIdx == fBasEnd.VertIdx) return;
            
            //Sort Basin points by R
            var basinPts = new SortedList<float, int>() { { PolPos(fBasEnd.VertIdx).r, fBasEnd.VertIdx}};
            var fBasScan = fBasStart;
            while (fBasScan.VertIdx != fBasEnd.VertIdx)
            {
                basinPts.Add(PolPos(fBasScan.VertIdx).r, fBasScan.VertIdx);
                fBasScan = fBasScan[_dir];
            }
            
            
            //Triangluate Basin
            
            while (basinPts.Count > 2)
            {
                var newVerts = new[]
                {
                    basinPts.Values[0],
                    basinPts.Values[1],
                    basinPts.Values[2]
                };
                
                //Find frontier point that will be removed
                var fOut = m_Frontier.GetHaving(newVerts[0]);
                if (!newVerts.Contains(fOut[RL.Right].VertIdx) || !newVerts.Contains(fOut[RL.Left].VertIdx))
                    fOut = m_Frontier.GetHaving(newVerts[1]);
                
                //Check that tri is valid (due to radius from Origin)
                var vecLt = fOut.Left.Vert.Pos - fOut.Vert.Pos;
                var vecRt = fOut.Right.Vert.Pos - fOut.Vert.Pos;
                if (AngleCW(vecLt, vecRt) >= (Math.PI - float.Epsilon)) break;

                var twinLt = fOut[RL.Left].EdgeRight;
                var twinRt = fOut.EdgeRight;

                var newTri = AddTriToMesh(twinLt, twinRt);
                m_Frontier.Remove(fOut);
                
                var changedVerts = Legalize(newTri.ID, twinLt.TriID, twinRt.TriID);
                LegalizeFrontier(changedVerts);

                basinPts.Remove(PolPos(fOut.VertIdx).r);
                DebugVisualizer.Visualize(this, "BasinTriAdded" + basinPts.Count);
            }
        }

        private void Finalize()
        {
            DebugVisualizer.Visualize(this, "PreFinalize");
            var fStart = m_Frontier.FrontierPts[0];
            var fScan = fStart.Right;
            while (fScan.VertIdx != fStart.VertIdx)
            {
                var vecL = fScan.Left.Vert.Pos - fScan.Vert.Pos;
                var vecR = fScan.Right.Vert.Pos - fScan.Vert.Pos;
                while (AngleCW(vecL, vecR) < (Math.PI - float.Epsilon))
                {
                    var twinLt = fScan.Left.EdgeRight;
                    var twinRt = fScan.EdgeRight;
                    var newTri = AddTriToMesh(twinLt, twinRt);
                    fScan = fScan.Left;
                    if (fStart.VertIdx == fScan.Right.VertIdx)
                        fStart = fScan.Left;
                    m_Frontier.Remove(fScan.Right);
                    var changedVerts = Legalize(newTri.ID, twinLt.TriID, twinRt.TriID);
                    LegalizeFrontier(changedVerts);
                    
                    vecL = fScan.Left.Vert.Pos - fScan.Vert.Pos;
                    vecR = fScan.Right.Vert.Pos - fScan.Vert.Pos;
                    
                }

                fScan = fScan.Right;

            }
            DebugVisualizer.Visualize(this, "PostFinalize");
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
                var newFntPt = m_Frontier.Add(ofVert, tri, fntVerts[0], fntVerts[1]);
                
                var fiData = Legalize(tri.ID, fntVerts[0].TriRightID);
                LegalizeFrontier(fiData);
                
                
                // 3) Walk Left
                Walk(RL.Left, newFntPt.VertIdx);

                // 4) Walk Right
                Walk(RL.Right, newFntPt.VertIdx);
                
                // 5) Check / Fill Basin Right
                FillBasin(RL.Right, newFntPt.VertIdx);
                
                // 6) Check / Fill Basin Left
                FillBasin(RL.Left, newFntPt.VertIdx);
            }
            
            // 7) Finalize
            Finalize();
        }

        private Vector2f CalcOrigin(out Rectf _boundsRect)
        {
            _boundsRect = new Rectf(m_Points[0].x, m_Points[0].y, 0, 0);
            foreach (var pt in m_Points)
                _boundsRect.Encapsulate(pt);
            var cent = _boundsRect.center;
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

        public class DCircleSweepObj : DelaunayObj
        {
            protected readonly CircleSweep CS;

            protected DCircleSweepObj(CircleSweep _cs)
                : base(_cs)
            {
                CS = _cs;
            }
        }

        public class Frontier : DCircleSweepObj //TODO PRIVATE?
        {
            private SortedList<float, FrontierPt> m_Frontier; //By Theta
            private Dictionary<int, FrontierPt> m_FrontierByVertIdx;

            public List<FrontierPt> FrontierPts => m_Frontier.Values.ToList();
            
            public bool ContainsVert(Vertex _vert)
            {
                return m_FrontierByVertIdx.ContainsKey(_vert.Idx);
            }

            public bool ContainsVert(int _vertIdx)
            {
                return m_FrontierByVertIdx.ContainsKey(_vertIdx);
            }

            public FrontierPt GetHaving(int _vertIdx)
            {
                var fpt = !m_FrontierByVertIdx.ContainsKey(_vertIdx) ? null : m_FrontierByVertIdx[_vertIdx];
                if (fpt == null)
                    return null;
                return fpt;
            }

            public Frontier(Triangle _firstTri, CircleSweep _cs)
                : base(_cs)
            {
                m_Frontier = new SortedList<float, FrontierPt>();
                m_FrontierByVertIdx = new Dictionary<int, FrontierPt>();
                
                var verts = _firstTri.VertIdxs.Select(_id => CS.m_Vertices[_id]).ToArray();

                var ftPts = _firstTri.Verts.Select(_vert => new FrontierPt(_vert.Idx, _firstTri.GetHalfEdgeWithOrigin(_vert.Idx), CS)).ToArray();

                ftPts[0].Right = ftPts[1];
                ftPts[0].Left = ftPts[2];
                ftPts[1].Right = ftPts[2];
                ftPts[1].Left = ftPts[0];
                ftPts[2].Right = ftPts[0];
                ftPts[2].Left = ftPts[1];


                for (int idx = 0; idx < 3; ++idx) //TODO handle same theta
                {
                    m_Frontier.Add(CS.PolPos(verts[idx]).Theta, ftPts[idx]);
                    m_FrontierByVertIdx.Add(ftPts[idx].VertIdx, ftPts[idx]);
                }
                    
            }

            public List<FrontierPt> DebugScanForTwin()
            {
                var dsft = new List<FrontierPt>();
                foreach (var fPt in m_Frontier)
                {
                    if (fPt.Value.EdgeRight.Twin != null)
                        dsft.Add(fPt.Value);
                }
                if(dsft.Count != 0)
                    throw new Exception("WTF");
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
                if(problems.Count != 0)
                    throw new Exception("WTF");
                return problems;
            }
            
            public FrontierPt Add(Vertex _newVert, Triangle _newTri, FrontierPt _fLt, FrontierPt _fRt)
            {


                var debug2 = DebugScanForOrder();
                if(debug2.Count != 0)
                    Console.WriteLine(debug2.ToString());
                var newEdge = _newTri.GetHalfEdgeWithOrigin(_newVert.Idx);
                var fNew = new FrontierPt(_newVert.Idx, newEdge, CS);

                
                _fLt.Right = _fRt.Left = fNew;
                _fLt.EdgeRight = _newTri.GetHalfEdgeWithOrigin(_fLt.VertIdx);

                fNew.Left = _fLt;
                fNew.Right = _fRt;

                m_FrontierByVertIdx.Add(_newVert.Idx, fNew);
                m_Frontier.Add(CS.PolPos(_newVert).Theta, fNew); //TODO add epsilon offset for dupe?
                
                var debug1 = DebugScanForTwin();
                if (debug1.Count != 0)
                    Console.WriteLine(debug1.ToString());


                debug2 = DebugScanForOrder();
                if(debug2.Count != 0)
                    Console.WriteLine(debug2.ToString());
                return fNew;
            }

            public void Remove(FrontierPt _fPt)
            {
                var fpLeft = _fPt.Left;
                var fpRight = _fPt.Right;
                fpLeft.Right = fpRight;
                fpRight.Left = fpLeft;
                
                
                
                var debugCntPre = m_Frontier.Count;
                m_Frontier.Remove(CS.m_PolPos[_fPt.VertIdx].Theta);
                m_FrontierByVertIdx.Remove(_fPt.VertIdx);
                
                var commonTris = CS.m_TrisContainingVert[fpLeft.VertIdx].Intersect(CS.m_TrisContainingVert[fpRight.VertIdx]).ToArray();
                if (commonTris.Length != 1)
                {
                    var tris = commonTris.Select(_id => CS.m_Triangles[_id]).ToArray();
                    
                    throw new Exception("WTF?" + tris.ToString()); //TODO DEBUG
                }
                fpLeft.EdgeRight = CS.m_Triangles[commonTris[0]].GetHalfEdgeWithOrigin(fpLeft.VertIdx);
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

            public Guid DebugGetTriangleAtFrontierRt(FrontierPt _fPt)
            {
                var trisLt = CS.m_TrisContainingVert[_fPt.VertIdx].Select(_id => CS.m_Triangles[_id]).ToArray();
                var trisRt = CS.m_TrisContainingVert[_fPt.Right.VertIdx].Select(_id => CS.m_Triangles[_id]).ToArray();
                var common = trisLt.Intersect(trisRt);
                var commonTris = CS.m_TrisContainingVert[_fPt.VertIdx].Intersect(CS.m_TrisContainingVert[_fPt.Right.VertIdx]).ToArray();
                if (commonTris.Length != 1)
                {
                    var tris = commonTris.Select(_id => CS.m_Triangles[_id]).ToArray();
                    
                    throw new Exception("WTF?" + tris.ToString()); //TODO DEBUG
                }

                return commonTris[0];
            }

            public class FrontierPt : DCircleSweepObj
            {
                public readonly int VertIdx;

                public HalfEdge EdgeRight
                {
                    get { return CS.m_Triangles[m_TriRightID].HalfEdge(m_EdgeRightIdx);}
                    set
                    {
                        m_TriRightID = value.TriID;
                        m_EdgeRightIdx = value.Tri.HalfEdgeIdxWithVert(value.OriginIdx);
                    }
                }
                public FrontierPt Left;
                public FrontierPt Right;
                public Vertex Vert => CS.m_Vertices[VertIdx];
                private Guid m_TriRightID;
                private int m_EdgeRightIdx;
                
                public Guid TriRightID => m_TriRightID;
                public int EdgeRightIdx => m_EdgeRightIdx;

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

                public FrontierPt this[RL _dir] => _dir == RL.Left ? Left : Right;

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
                Theta = CircleSweep.AngleFromOrigin(_origin, _cPt);
            }
        }
        
        /// <summary>
        /// Angle to origin, x axis clockwise.
        /// </summary>
        public static float AngleFromOrigin(Vector2f _origin, Vector2f _pt)
        {
            var to = _pt - _origin;
            var from = Vector2f.right - _origin;
            return AngleCW(from, to);
        }

        public static float AngleCW(Vector2f _from, Vector2f _to)
        {
            var signedAngleRad = (float) Math.Atan2(_to.y, _to.x) - (float) Math.Atan2(_from.y, _from.x);
            return signedAngleRad >= 0 ? (float) (2d * Math.PI) - signedAngleRad : -signedAngleRad;
        }
    }
}