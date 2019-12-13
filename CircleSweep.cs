namespace ioDelaunay
{
    using System;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Linq;
    using System.Runtime.CompilerServices;

    public class CircleSweep : Delaunay.Triangulator
    {
        #region Fields

        private FastPriorityQueue<Frontier.FrontierPt> m_BasinPtQ = new FastPriorityQueue<Frontier.FrontierPt>(0);

        /// <summary>
        /// Object tracking frontier of circle sweep triangulation
        /// </summary>
        private Frontier m_Frontier;
        private Vector2 m_Origin;
        private PolartPt[] m_PolPos; //By Point Idx
        private int[] m_VertIdxsByR;

        #endregion Fields

        #region Enumerations

        private enum RL
        {
            Right,
            Left
        }

        #endregion Enumerations

        #region Methods

        protected override void Algorithm()
        {
            Init();

            var progState = "Triangulating Algorithm";
            var prog = 0f;
            var idxVertTot = m_VertIdxsByR.Length;
            
            for (var rIdx = 0; rIdx < m_VertIdxsByR.Length; ++rIdx)
            {
                prog = (float)rIdx / (float)m_VertIdxsByR.Length;
                progState = "Circle Sweep Vert " + rIdx + " of " + idxVertTot;
                D.Prog.Update(prog, progState);
                
                var ofVertIdx = m_VertIdxsByR[rIdx];

                // 1) Project
                var fntLt = m_Frontier.FindNextHigher(m_PolPos[ofVertIdx]).Left;

                // 2) Create Tri and Legalize
                new Delaunay.Triangle(fntLt.EdgeRight, ofVertIdx, D);
                var newFntPt = m_Frontier.Add(m_PolPos[ofVertIdx], fntLt);

                D.Legalize(newFntPt.EdgeRight.NextEdge);

                // 3) Walk Left
                Walk(RL.Left, newFntPt);

                // 4) Walk Right
                Walk(RL.Right, newFntPt);

                // 5) Check / Fill Basin Right
                FillBasin(RL.Right, newFntPt);

                // 6) Check / Fill Basin Left
                FillBasin(RL.Left, newFntPt);
            }

            // 7) Finalize
            D.Prog.Update(0,"Finalizing Hull...");
            FinalizeHull();
            D.Prog.Update(1,"Finalizing Hull...Done!");
        }

        /// <summary>
        /// Fills HullEdges in CW order
        /// </summary>
        protected override void Hull()
        {
            var fScan = m_Frontier.LastAddedFPt;
            var fStart = fScan;
            D.HullEdges.Clear();
            D.HullEdges.Add(fStart.EdgeRight);
            fScan = fScan.Right;
            while (fScan.VertIdx != fStart.VertIdx)
            {
                D.HullEdges.Add(fScan.EdgeRight);
                fScan = fScan.Right;
            }
        }

        private Vector2 CalcOrigin2(out int[] _firstTriIdxs)
        {
            var cent = D.BoundsRect.center;
            Func<Vector2, float> fPtDist = _p => Math.Abs(_p.x) + Math.Abs(_p.y);

            var lstPIdx = new List<int>(3)
            {
                0, 1, 2
            };
            var lstPDist = new List<float>(3)
                {fPtDist(D.Points[0]),fPtDist(D.Points[1]), fPtDist(D.Points[2])};
            
            lstPIdx.Sort((_a, _b) => lstPDist[_a].CompareTo(lstPDist[_b]) );
            lstPDist = new List<float>(3)
            {
                fPtDist(D.Points[lstPIdx[0]]),
                fPtDist(D.Points[lstPIdx[1]]),
                fPtDist(D.Points[lstPIdx[2]])
            };
            
            for(int pIdx = 3; pIdx < D.Points.Count; ++pIdx)
            {
                var pDist = fPtDist(D.Points[pIdx]);
                if (pDist > lstPDist[2]) continue;
                if (pDist < lstPDist[0])
                {
                    lstPDist[2] = lstPDist[1];
                    lstPIdx[2] = lstPIdx[1];
                    lstPDist[1] = lstPDist[0];
                    lstPIdx[1] = lstPIdx[0];
                    lstPDist[0] = pDist;
                    lstPIdx[0] = pIdx;
                    continue;
                }

                if (pDist < lstPDist[1])
                {
                    lstPDist[2] = lstPDist[1];
                    lstPIdx[2] = lstPIdx[1];
                    lstPDist[1] = pDist;
                    lstPIdx[1] = pIdx;
                    continue;

                }
                
                lstPDist[2] = pDist;
                lstPIdx[2] = pIdx;
                
            }
            
            
            
            _firstTriIdxs = lstPIdx.ToArray();
            //Find 1st non-linear set of points (valid triangle)
            var findNonLineIdx = 3;
            while (Geom.AreColinear(D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]],
                D.MinFloatingPointErr))
            {
                var dIdx = _firstTriIdxs[0];
                var dbgPt0 = D.Points[_firstTriIdxs[0]]; //TODO remove
                var dbgPt1 = D.Points[_firstTriIdxs[1]];
                var dbgPt2 = D.Points[_firstTriIdxs[2]];
                
                var mPntX = D.Points[dIdx].x;
                var mPntY = D.Points[dIdx].y;
                var rnd = new Random((int)DateTime.Now.Ticks);
                var rndDir = rnd.Next(4);
                if (rndDir == 0)
                    D.Points[dIdx].Set(mPntX + D.MinFloatingPointErr, mPntY);
                else if (rndDir == 1)
                    D.Points[dIdx].Set(mPntX - D.MinFloatingPointErr, mPntY);
                else if (rndDir == 2)
                    D.Points[dIdx].Set(mPntX, mPntY + D.MinFloatingPointErr);
                else 
                    D.Points[dIdx].Set(mPntX, mPntY - D.MinFloatingPointErr);
                    
            };

            //Force Clockwise
            var v0 = D.Points[_firstTriIdxs[0]];
            var v1 = D.Points[_firstTriIdxs[1]];
            var v2 = D.Points[_firstTriIdxs[2]];
            var vecL = v1 - v0;
            var vecR = v2 - v0;

            //Positive Cross Product greater than 180
            var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
            if (crossZ > 0)
            {
                var tempIdx = _firstTriIdxs[1];
                _firstTriIdxs[1] = _firstTriIdxs[2];
                _firstTriIdxs[2] = tempIdx;
            }

            var triPts = new[] {D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]]};
            var cc = Geom.CentroidOfPoly(triPts);
            return cc;
            
        }

        private Vector2 CalcOrigin(out int[] _firstTriIdxs)
        {
            
            var cent = D.BoundsRect.center;
            var pIdxArr = new int[D.Points.Count];
            var pDistArr = new float[D.Points.Count];
            for (int idx = 0; idx < pIdxArr.Length; ++idx)
            {
                var dx = Math.Abs(cent.x - D.Points[idx].x);
                var dy = Math.Abs(cent.y - D.Points[idx].y);
                pIdxArr[idx] = idx;
                pDistArr[idx] = dx + dy;
            }
            
            Array.Sort(pIdxArr, (_a, _b) => pDistArr[_a].CompareTo(pDistArr[_b]));
            
            _firstTriIdxs = new[] {pIdxArr[0], pIdxArr[1], pIdxArr[2]};
            //Find 1st non-linear set of points (valid triangle)
            var findNonLineIdx = 3;
            while (Geom.AreColinear(D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]],
                D.MinFloatingPointErr)) _firstTriIdxs[2] = pIdxArr[findNonLineIdx++];

            //Force Clockwise
            var v0 = D.Points[_firstTriIdxs[0]];
            var v1 = D.Points[_firstTriIdxs[1]];
            var v2 = D.Points[_firstTriIdxs[2]];
            var vecL = v1 - v0;
            var vecR = v2 - v0;

            //Positive Cross Product greater than 180
            var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
            if (crossZ > 0)
            {
                var tempIdx = _firstTriIdxs[1];
                _firstTriIdxs[1] = _firstTriIdxs[2];
                _firstTriIdxs[2] = tempIdx;
            }

            var triPts = new[] {D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]]};
            var cc = Geom.CentroidOfPoly(triPts);
            return cc;
        }
        private Vector2 CalcOriginOrig(out int[] _firstTriIdxs)
        {
            var cent = D.BoundsRect.center;
            var closest = new SortedList<float, int>
            {
                {(cent - D.Points[0]).sqrMagnitude, 0},
                {(cent - D.Points[1]).sqrMagnitude, 1},
                {(cent - D.Points[2]).sqrMagnitude, 2}
            };

            for (var idx = 3; idx < D.Points.Count; ++idx)
            {
                var pt = D.Points[idx];
                var distSqr = (cent - pt).sqrMagnitude;
                if (distSqr > closest.Keys[2])
                    continue;
                closest.Add(distSqr, idx);
            }

            _firstTriIdxs = new[] {closest.Values[0], closest.Values[1], closest.Values[2]};
            //Find 1st non-linear set of points (valid triangle)
            var findNonLineIdx = 3;
            while (Geom.AreColinear(D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]],
                D.MinFloatingPointErr)) _firstTriIdxs[2] = closest.Values[findNonLineIdx++];

            //Force Clockwise
            var v0 = D.Points[_firstTriIdxs[0]];
            var v1 = D.Points[_firstTriIdxs[1]];
            var v2 = D.Points[_firstTriIdxs[2]];
            var vecL = v1 - v0;
            var vecR = v2 - v0;

            //Positive Cross Product greater than 180
            var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
            if (crossZ > 0)
            {
                var tempIdx = _firstTriIdxs[1];
                _firstTriIdxs[1] = _firstTriIdxs[2];
                _firstTriIdxs[2] = tempIdx;
            }

            var triPts = new[] {D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]]};
            var cc = Geom.CentroidOfPoly(triPts);
            return cc;
        }

        //[MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int ComparePol(PolartPt _a, PolartPt _b)
        {
            var ra = _a.r;
            var rb = _b.r;
            if (ra < rb) return -1;
            if (ra > rb) return 1;
            if (_a.Theta < _b.Theta) return -1;
            return 1;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void FillBasin(RL _dir, Frontier.FrontierPt _fpt)
        {
            //Setup
            var fvi = _fpt;
            var fvrp = fvi[_dir][_dir];
            var polvi = m_PolPos[fvi.VertIdx];
            var polvrp = m_PolPos[fvrp.VertIdx];
            var pts = D.Points;

            var r2 = polvrp.r;
            var dr = polvi.r - r2;
            var dTheta = _dir == RL.Right ? polvrp.Theta - polvi.Theta : polvi.Theta - polvrp.Theta;

            //Check heuristic
            var h = dr / (r2 * dTheta);
            if (h < 2) return;

            //Find Basin Min
            var ptCount = 0;
            var fBasStart = fvrp;
            var fBasMin = fBasStart[_dir];
            while (m_PolPos[fBasMin.VertIdx].r > m_PolPos[fBasMin[_dir].VertIdx].r)
            {
                fBasMin = fBasMin[_dir];
                ptCount++;
            }

            if (ptCount == 0) return;

            //Find Basin End
            var fBasEnd = fBasMin;
            while (true)
            {
                var fbePos = pts[fBasEnd.VertIdx];

                var vecL = pts[fBasEnd[RL.Left].VertIdx] - fbePos;
                var vecR = pts[fBasEnd[RL.Right].VertIdx] - fbePos;

                //Positive Cross Product greater than 180
                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ > 0) break;

                fBasEnd = fBasEnd[_dir];
                ptCount++;
            }

            if (fBasStart[_dir] == fBasEnd) return;

            //Sort Basin points by R
            var fBasScan = fBasStart[_dir];
            m_BasinPtQ.Reset(ptCount);
            for(int sIdx = 0; sIdx < ptCount; ++sIdx)
            {
                m_BasinPtQ.Enqueue(fBasScan, m_PolPos[fBasScan.VertIdx].r);
                fBasScan = fBasScan[_dir];
            }

            //Triangluate Basin
            while (m_BasinPtQ.Count != 0)
            {
                //Find frontier point that will be removed
                var fOut = m_BasinPtQ.Dequeue();
                var fLt = fOut.Left;
                var fRt = fOut.Right;

                //Check that tri is valid (due to radius from Origin)
                var fOutPos = pts[fOut.VertIdx];
                var vecL = pts[fLt.VertIdx] - fOutPos;
                var vecR = pts[fRt.VertIdx] - fOutPos;

                //Check for lines
                //Positive Cross Product greater than 180
                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ >= 0) break;

                var twinLt = fOut[RL.Left].EdgeRight;
                var twinRt = fOut.EdgeRight;

                new Delaunay.Triangle(twinLt, twinRt, D);
                m_Frontier.Remove(fOut, twinLt.Twin.NextEdge);

                D.Legalize(twinLt, twinRt);
            }
        }

        private void FinalizeHull()
        {
            var fStart = m_Frontier.LastAddedFPt;
            var fScan = fStart;
            var firstScanDone = false;
            var maxHullAngle = Settings.ForceConvexHull
                ? Math.PI - float.Epsilon
                : Settings.MaxHullAngleDegrees * (float) (Math.PI / 180f);

            var pts = D.Points;
            while (fScan.VertIdx != fStart.VertIdx || !firstScanDone)
            {
                var fScanPos = pts[fScan.VertIdx];
                var vecL = pts[fScan.Left.VertIdx] - fScanPos;
                var vecR = pts[fScan.Right.VertIdx] - fScanPos;

                while (vecL.AngleCW(vecR) < maxHullAngle)
                {
                    var twinLt = fScan.Left.EdgeRight;
                    var twinRt = fScan.EdgeRight;
                    new Delaunay.Triangle(twinLt, twinRt, D);
                    fScan = fScan.Left;
                    if (fStart.VertIdx == fScan.Right.VertIdx)
                    {
                        fStart = fStart.Right;
                        firstScanDone = false;
                    }

                    m_Frontier.Remove(fScan.Right, twinLt.Twin.NextEdge);
                    D.Legalize(twinLt, twinRt);

                    var fScanPos2 = pts[fScan.VertIdx];
                    vecL = pts[fScan.Left.VertIdx] - fScanPos2;
                    vecR = pts[fScan.Right.VertIdx] - fScanPos2;
                }

                fScan = fScan.Right;
                if (!firstScanDone && fScan.VertIdx != fStart.VertIdx) firstScanDone = true;
            }
        }

        private void Init()
        {
            int[] firstTriIdxs;
            var pts = D.Points;
            var ptsCnt = pts.Count;
            m_Origin = CalcOrigin(out firstTriIdxs);
            Func<float, int> keyHash;
            var thetaGroups = Frontier.ThetaGroup.BuildThetaGroups(ptsCnt, out keyHash);
            var grpsCnt = thetaGroups.Length;
            m_PolPos = new PolartPt[ptsCnt]; //TODO make this dynamic to save memory?
            m_VertIdxsByR = new int[ptsCnt - 3];
            var idxOffset = 0;
            for (var pIdx = 0; pIdx < ptsCnt; ++pIdx)
            {
                //Store polar position
                var polPos = new PolartPt(pts[pIdx], m_Origin);
                m_PolPos[pIdx] = polPos;
                var theta = polPos.Theta;

                //Reference theta group
                var grpIdx = keyHash(theta);
                if (grpIdx >= grpsCnt) //Floating point error check TODO
                    grpIdx--;

                polPos.thetaGroup = thetaGroups[grpIdx];

                //Prepare r index reference (to be sorted later)
                if (firstTriIdxs.Contains(pIdx))
                {
                    idxOffset++;
                    continue;
                }

                m_VertIdxsByR[pIdx - idxOffset] = pIdx;
            }

            //Sort points by distance to origin
            //Array.Sort(m_VertIdxsByR, (_a, _b) => m_PolPos[_a].r.CompareTo(m_PolPos[_b].r));
            Array.Sort(m_VertIdxsByR, (_a, _b) => ComparePol(m_PolPos[_a], m_PolPos[_b]));

            //Init Frontier
            var firstTri = new Delaunay.Triangle(firstTriIdxs[0], firstTriIdxs[1], firstTriIdxs[2], D);

            m_Frontier = new Frontier(firstTri, this);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void Walk(RL _dir, Frontier.FrontierPt _fPt)
        {
            var fvi = _fPt;
            var pts = D.Points;
            while (true)
            {
                var fvn = fvi[_dir];
                var fLt = fvn[RL.Left];
                var fRt = fvn[RL.Right];

                var fvnPos = pts[fvn.VertIdx];

                var vecL = pts[fLt.VertIdx] - fvnPos;
                var vecR = pts[fRt.VertIdx] - fvnPos;
                //var angleCW = vecL.AngleCW(vecR);
                //if (angleCW > Math.PI / 2f) break;
                //Positive Cross Product greater than 180
                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ > 0) break;
                //Positive Dot product greater than 90
                var dot = vecL.x * vecR.x + vecL.y * vecR.y;
                if (dot < 0) break;

                var twinLt = fLt.EdgeRight;
                var twinRt = fvn.EdgeRight;
                new Delaunay.Triangle(twinLt, twinRt, D);
                m_Frontier.Remove(fvn, twinLt.Twin.NextEdge);
                D.Legalize(twinLt, twinRt);
            }
        }

        #endregion Methods

        #region Nested Types

        public static class Settings
        {
            #region Fields

            /// <summary>
            ///     Force outer hull to be convex.  If true, overrides MaxHullAngleDegrees
            /// </summary>
            public static bool ForceConvexHull = false;

            /// <summary>
            ///     When finishing hull, sets the max angle at which a tri will be added.
            /// </summary>
            public static float MaxHullAngleDegrees = 177f;

            #endregion Fields
        }

        private class Frontier
        {
            #region Fields

            public FrontierPt LastAddedFPt;

            private CircleSweep CS;

            #endregion Fields

            #region Constructors

            public Frontier(Delaunay.Triangle _firstTri, CircleSweep _cs)
            {
                CS = _cs;

                var ftPts = new List<FrontierPt>
                {
                    new FrontierPt(_firstTri.Edge0, CS),
                    new FrontierPt(_firstTri.Edge1, CS),
                    new FrontierPt(_firstTri.Edge2, CS)
                };

                ftPts[0].Right = ftPts[1];
                ftPts[0].Left = ftPts[2];
                ftPts[1].Right = ftPts[2];
                ftPts[1].Left = ftPts[0];
                ftPts[2].Right = ftPts[0];
                ftPts[2].Left = ftPts[1];

                CS.m_PolPos[ftPts[0].VertIdx].thetaGroup.Add(ftPts[0]);
                CS.m_PolPos[ftPts[1].VertIdx].thetaGroup.Add(ftPts[1]);
                CS.m_PolPos[ftPts[2].VertIdx].thetaGroup.Add(ftPts[2]);
            }

            #endregion Constructors

            #region Methods

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public FrontierPt Add(PolartPt _pt, FrontierPt _fLt)
            {
                var fRt = _fLt.Right;
                var newEdgeLt = _fLt.EdgeRight.Twin.NextEdge;
                var newEdgeRt = newEdgeLt.NextEdge;
                var fNew = new FrontierPt(newEdgeRt, CS);

                _fLt.Right = fRt.Left = fNew;
                _fLt.EdgeRight = newEdgeLt;

                fNew.Left = _fLt;
                fNew.Right = fRt;

                _pt.thetaGroup.Add(fNew);

                LastAddedFPt = fNew;
                return fNew;
            }

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public FrontierPt FindNextHigher(PolartPt _pt)
            {
                var grp = _pt.thetaGroup;
                var theta = _pt.Theta;
                if (grp.First == null)
                {
                    var scan = grp.Next;
                    while (scan.First == null)
                        scan = scan.Next;
                    return scan.First;
                }

                if (grp.Next.First == null)
                {
                    var scan = grp.First;
                    while (scan.Theta < theta && scan != grp.Last.Right)
                        scan = scan.Right;
                    return scan;
                }

                if (theta < grp.Mid)
                {
                    var scan = grp.First;
                    while (scan.Theta < theta && scan != grp.Next.First)
                        scan = scan.Right;
                    return scan;
                }
                else
                {
                    var scan = grp.Next.First.Left;
                    while (scan.Theta > theta && scan != grp.First.Left)
                        scan = scan.Left;
                    return scan.Right;
                }
            }

            
            public void Remove(FrontierPt _fPt, Delaunay.Triangle.HalfEdge _newEdgeRight)
            {
                var fpLeft = _fPt.Left;
                var fpRight = _fPt.Right;
                fpLeft.Right = fpRight;
                fpRight.Left = fpLeft;

                fpLeft.EdgeRight = _newEdgeRight;
                CS.m_PolPos[_fPt.VertIdx].thetaGroup.Remove(_fPt);
            }

            #endregion Methods

            #region Nested Types

            public class FrontierPt : FastPriorityQueueNode
            {
                #region Fields

                public Delaunay.Triangle.HalfEdge EdgeRight;
                public FrontierPt Left;
                public FrontierPt Right;
                public float Theta => CS.m_PolPos[VertIdx].Theta;
                public FrontierPt this[RL _dir] => _dir == RL.Left ? Left : Right;
                public int VertIdx => EdgeRight.OriginIdx;

                private CircleSweep CS;

                #endregion Fields

                #region Constructors

                public FrontierPt(Delaunay.Triangle.HalfEdge _edge, CircleSweep _cs)
                {
                    EdgeRight = _edge;
                    CS = _cs;
                }

                #endregion Constructors

                #region Methods

                public override string ToString()
                {
                    var er = (EdgeRight ?? (object) "").ToString();
                    return "Fnt Pt t: " + Theta + " v: " + VertIdx + " RtV: " + Right.VertIdx + " LtV: " +
                           Left.VertIdx + " EdgeRt: " + er;
                }

                #endregion Methods
            }

            public class ThetaGroup
            {
                #region Fields

                public FrontierPt First;
                public float Mid => Min + (Max - Min) / 2;
                public ThetaGroup Next;

                private int m_Count;
                private float Max => Next.First.Left.Theta;
                private float Min => First.Theta; //TODO dynamic math or static span?

                #endregion Fields

                #region Properties

                //TODO SLOW?
                public FrontierPt Last
                {
                    get
                    {
                        if (First == null) return null;
                        if (Next.First != null)
                            return Next.First.Left;
                        if (m_Count == 1) return First;
                        var grpScan = Next;
                        while (grpScan.First == null)
                            grpScan = grpScan.Next;
                        return grpScan.First.Left;
                    }
                }

                #endregion Properties

                #region Methods

                /// <summary>
                ///     We'll need the theta groups built for init before the frontier is constructed.
                /// </summary>
                /// <param name="_pntCount"></param>
                /// <returns></returns>
                public static ThetaGroup[] BuildThetaGroups(int _pntCount, out Func<float, int> _keyHash)
                {
                    //Init theta groups
                    var pi2 = 2d * Math.PI;
                    var k = 1 + (int) Math.Pow(_pntCount, 1f / 3f);
                    var incr = (float) (pi2 / k);
                    _keyHash = _theta =>
                    {
                        var div = _theta / incr;
                        var divInt = (int) div;
                        return divInt;
                    };

                    var thetaGroups = new ThetaGroup[k];
                    thetaGroups[0] = new ThetaGroup();

                    for (int tIdx = 1; tIdx < k; ++tIdx)
                    {
                        thetaGroups[tIdx] = new ThetaGroup();
                        thetaGroups[tIdx - 1].Next = thetaGroups[tIdx];
                    }

                    thetaGroups[k - 1].Next = thetaGroups[0];

                    return thetaGroups;
                }

                public void Add(FrontierPt _pt)
                {
                    if (First == null)
                    {
                        First = _pt;
                        m_Count++;
                        return;
                    }

                    var newTheta = _pt.Theta;
                    var firstTheta = First.Theta;
                    if (firstTheta > newTheta)
                        First = _pt;
                    else if (_pt.Right == First) First = _pt;

                    m_Count++;
                }

                public void Remove(FrontierPt _pt)
                {
                    if (_pt == First) First = m_Count == 1 ? null : First.Right;

                    m_Count--;
                }

                #endregion Methods
            }

            #endregion Nested Types
        }

        private class PolartPt
        {
            #region Fields

            public readonly float r;
            public readonly float Theta;

            public Frontier.ThetaGroup thetaGroup;

            #endregion Fields

            #region Constructors

            public PolartPt(Vector2 _cPt, Vector2 _origin)
            {
                var vec = _cPt - _origin;
                r = vec.magnitude; //TODO optimize (use sqr?)

                var from = Vector2.right - _origin;
                Theta = from.AngleCW(vec);
            }

            #endregion Constructors
        }

        #endregion Nested Types
    }
}