using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Runtime.CompilerServices;

namespace ioDelaunay
{
    public class CircleSweep : Delaunay.Triangulator
    {
        #region Enumerations

        public enum RL
        {
            Right,
            Left
        }

        #endregion Enumerations

        #region Properties

        public Vector2 Origin { get; private set; }

        #endregion Properties

        #region Nested Interfaces

        private interface ICircleSweepObj
        {
            #region Properties

            CircleSweep CS { get; }

            #endregion Properties
        }

        #endregion Nested Interfaces

        #region Fields

        public Rect Bounds => new Rect(D.BoundsRect);
        public Frontier frontier;

        // Change
        // Don't change
        private PolartPt[] m_PolPos; //By Point Idx
        private int[] m_VertIdxsByR;

        #endregion Fields

        #region Methods

        protected override void Algorithm()
        {
            //string timeStamp = $"{DateTime.Now:yyyyMMdd_hhmmss}";
            //var timer = new Stopwatch(); //TODO DEBUG

            //timer.Start();
            Init();
            //timer.Stop();
            //var initTime = timer.ElapsedMilliseconds;

            //timer.Reset();
            //timer.Start();
            for (var rIdx = 0; rIdx < m_VertIdxsByR.Length; ++rIdx)
            {
                var ofVertIdx = m_VertIdxsByR[rIdx];

                // 1) Project
                var fntLt = frontier.FindNextHigher(m_PolPos[ofVertIdx]).Left;
                //var fntVerts = frontier.Project(m_PolPos[ofVertIdx]);

                // 2) Create Tri and Legalize
                new Delaunay.Triangle(fntLt.EdgeRight, ofVertIdx, D);
                var newFntPt = frontier.Add(m_PolPos[ofVertIdx], fntLt);

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
            FinalizeHull();

            //timer.Stop(); //TODO DEBUG
            //var algTime = timer.ElapsedMilliseconds;
            //var totTime = initTime + algTime;
            //DebugVisualizer.LogToFile("DelTime" + timeStamp, "Total " + totTime + "ms : Init " + initTime + "ms : Alg " + algTime + "ms");
        }

        protected void FinalizeHull()
        {
            var fStart = frontier.LastAddedFPt;
            var fScan = fStart;
            var firstScanDone = false;
            var maxHullAngle = Settings.ForceConvexHull
                ? Math.PI - float.Epsilon
                : Settings.MaxHullAngleDegrees * (float) (Math.PI / 180f);

            var pts = D.Points;
            List<Delaunay.Triangle.HalfEdge> hull = new List<Delaunay.Triangle.HalfEdge>();
            while (fScan.VertIdx != fStart.VertIdx || !firstScanDone)
            {
                var fScanPos = pts[fScan.VertIdx];
                var vecL = pts[fScan.Left.VertIdx] - fScanPos;
                var vecR = pts[fScan.Right.VertIdx] - fScanPos;

                //var vecL = fScan.Left.Pos - fScan.Pos;
                //var vecR = fScan.Right.Pos - fScan.Pos;
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

                    frontier.Remove(fScan.Right, twinLt.m_Twin.NextEdge);
                    //D.Legalize(twinLt.Twin, twinRt.Twin);
                    D.Legalize(twinLt, twinRt);

                    var fScanPos2 = pts[fScan.VertIdx];
                    vecL = pts[fScan.Left.VertIdx] - fScanPos2;
                    vecR = pts[fScan.Right.VertIdx] - fScanPos2;

                    //vecL = fScan.Left.Pos - fScan.Pos;
                    //vecR = fScan.Right.Pos - fScan.Pos;
                }

                hull.Add(fScan.EdgeRight);
                fScan = fScan.Right;
                if (!firstScanDone && fScan.VertIdx != fStart.VertIdx) firstScanDone = true;
            }
        }

        protected override void Hull()
        {
            var fScan = frontier.LastAddedFPt;
            var fStart = fScan;
            D.HullIdxs = new List<int> {fStart.VertIdx};
            D.HullEdges = new List<Delaunay.Triangle.HalfEdge>();
            fScan = fScan.Right;
            while (fScan.VertIdx != fStart.VertIdx)
            {
                D.HullIdxs.Add(fScan.VertIdx);
                D.HullEdges.Add(fScan.EdgeRight);
                fScan = fScan.Right;
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static int BinarySearch(IList<float> _list, float _value)
        {
            var lower = 0;
            var upper = _list.Count - 1;

            while (lower <= upper)
            {
                var middle = lower + (upper - lower) / 2;
                //var middle = lower + ((upper - lower) >> 1);

                var target = _list[middle];
                if (target > _value)
                    upper = middle - 1;
                else if (target < _value)
                    lower = middle + 1;
                else
                    return middle;
            }

            return ~lower;
        }

        private Vector2 CalcOrigin(out int[] _firstTriIdxs)
        {
            var cent = Bounds.center;
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

        private FastPriorityQueue<Frontier.FrontierPt> m_BasinPtQ = new FastPriorityQueue<Frontier.FrontierPt>(0);
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

                //TODO DEBUG
                //var ltChk = fLt == fBasStart || fLt == fBasEnd;
                //var rtChk = fRt == fBasStart || fRt == fBasEnd;
                //var inChk = m_BasinPtQ.Contains(fBasStart) || m_BasinPtQ.Contains(fBasEnd);

                new Delaunay.Triangle(twinLt, twinRt, D);
                frontier.Remove(fOut, twinLt.m_Twin.NextEdge);

                D.Legalize(twinLt, twinRt);
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private void FillBasin2(RL _dir, Frontier.FrontierPt _fpt)
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
            var fBasStart = fvrp;
            var fBasMin = fBasStart[_dir];
            var ptList = new List<Frontier.FrontierPt>();
            while (m_PolPos[fBasMin.VertIdx].r > m_PolPos[fBasMin[_dir].VertIdx].r)
            {
                ptList.Add(fBasMin);
                fBasMin = fBasMin[_dir];
            }


            //Find Basin End 
            var fBasEnd = fBasMin;
            while (true)
            {
                var fbePos = pts[fBasEnd.VertIdx];

                var vecL = pts[fBasEnd.Left.VertIdx] - fbePos;
                var vecR = pts[fBasEnd.Right.VertIdx] - fbePos;
                
                //Positive Cross Product greater than 180
                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ > 0) break;

                ptList.Add(fBasEnd);
                fBasEnd = fBasEnd[_dir];
            }

            

            
            var listCnt = ptList.Count;
            if (listCnt == 0) return;
            
            if (fBasEnd == fBasMin) 
                listCnt--;
            
            ptList.Sort((_a, _b) => ComparePol(_a.PolPos, _b.PolPos));
            
            for (int pIdx = 0; pIdx < listCnt; ++pIdx)
            {
                var fOut = ptList[pIdx];
                var fr = fOut.Right;
                var fl = fOut.Left;

                //Check that tri is valid (due to radius from Origin)
                var fOutPos = pts[fOut.VertIdx];
                var vecL = pts[fl.VertIdx] - fOutPos;
                var vecR = pts[fr.VertIdx] - fOutPos;

                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ > 0) break;

                //Add Tri
                var twinLt = fOut[RL.Left].EdgeRight;
                var twinRt = fOut.EdgeRight;

                
                new Delaunay.Triangle(twinLt, twinRt, D);
                frontier.Remove(fOut, twinLt.m_Twin.NextEdge);

                D.Legalize(twinLt, twinRt);
            }
        }

        private void DebugCheckFrontier()
        {
            var proj = frontier.ProjectZero();
            var fPt = proj[1];
            var firstfPt = fPt;
            fPt = fPt.Right;
            var fList = new List<Frontier.FrontierPt>();
            while (fPt.VertIdx != firstfPt.VertIdx)
            {
                fList.Add(fPt.Left);
                var t0 = m_PolPos[fPt.Left.VertIdx].Theta;
                var t1 = m_PolPos[fPt.VertIdx].Theta;

                if (t1 < t0) Trace.WriteLine("Debug");

                fPt = fPt.Right;
            }
        }

        private void Init()
        {
            int[] firstTriIdxs;
            var pts = D.Points;
            var ptsCnt = pts.Count;
            Origin = CalcOrigin(out firstTriIdxs);
            Func<float, int> keyHash;
            var thetaGroups = Frontier.ThetaGroup.BuildThetaGroups(ptsCnt, out keyHash);
            var grpsCnt = thetaGroups.Length;
            m_PolPos = new PolartPt[ptsCnt]; //TODO make this dynamic to save memory?
            m_VertIdxsByR = new int[ptsCnt - 3];
            var idxOffset = 0;
            for (var pIdx = 0; pIdx < ptsCnt; ++pIdx)
            {
                //Store polar position
                var polPos = new PolartPt(pts[pIdx], Origin, this);
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

            frontier = new Frontier(firstTri, this);
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
                frontier.Remove(fvn, twinLt.m_Twin.NextEdge);
                D.Legalize(twinLt, twinRt);
            }
        }

        #endregion Methods

        #region Nested Types


        public class Frontier : ICircleSweepObj
        {
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

            #region Properties

            public CircleSweep CS { get; }

            #endregion Properties

            #region Fields

            public FrontierPt LastAddedFPt;

            #endregion Fields

            #region Methods
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            public FrontierPt Add(PolartPt _pt, FrontierPt _fLt)
            {
                var fRt = _fLt.Right;
                var newEdgeLt = _fLt.EdgeRight.m_Twin.NextEdge;
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

            public FrontierPt FindNextHigherAtZero() //DEBUG TODO 
            {
                var grp = CS.m_PolPos[0].thetaGroup;

                while (grp.First == null)
                    grp = grp.Next;

                var scan = grp.First;

                while (scan.Theta - scan.Left.Theta >= 0)
                    scan = scan.Right;
                return scan;
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
            
            public FrontierPt[] Project(PolartPt _pPt)
            {
                var fRt = FindNextHigher(_pPt);
                return new[] {fRt.Left, fRt};
            }

            public FrontierPt[] ProjectZero() //TODO DEBUG
            {
                var fRt = FindNextHigherAtZero();
                return new[] {fRt.Left, fRt};
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

            public class ThetaGroup
            {
                public FrontierPt First;

                public ThetaGroup Next;

                public float Min => First.Theta; //TODO dynamic math or static span?
                public float Mid => Min + (Max - Min) / 2;
                public float Max => Next.First.Left.Theta;

                public FrontierPt Last //TODO SLOW
                {
                    get
                    {
                        if (First == null) return null;
                        if (Next.First != null)
                            return Next.First.Left;
                        if (Count == 1) return First;
                        var grpScan = Next;
                        while (grpScan.First == null)
                            grpScan = grpScan.Next;
                        return grpScan.First.Left;
                    }
                }

                public int Count { get; private set; }


                /// <summary>
                ///     We'll need the theta groups built for init before the frontier is constructed.
                /// </summary>
                /// <param name="_pntCount"></param>
                /// <returns></returns>
                public static Dictionary<float, ThetaGroup> BuildThetaGroups(int _pntCount, out List<float> _keys, out Func<float, float> _keyHash)
                {
                    //Init theta groups
                    var pi2 = 2d * Math.PI;
                    var k = 1 + (int) Math.Pow(_pntCount, 1f / 3f);
                    var incr = (float) (pi2 / k);
                    var curIncr = incr;
                    //var prevTg = new ThetaGroup(0, incr);
                    _keys = new List<float> {0f};
                    _keyHash = _theta => (int) (_theta / incr);
                    var prevTg = new ThetaGroup();
                    var thetaGroups = new Dictionary<float, ThetaGroup>(k)
                    {
                        {0, prevTg}
                    };
                    for (var grpIdx = 1; grpIdx < k - 1; ++grpIdx)
                    {
                        _keys.Add(curIncr);
                        var nextIncr = curIncr + incr;

                        //var tg = new ThetaGroup(curIncr, nextIncr);
                        var tg = new ThetaGroup();
                        thetaGroups.Add(curIncr, tg);
                        prevTg.Next = tg;
                        //tg.Prev = prevTg;
                        prevTg = tg;
                        curIncr = nextIncr;
                    }

                    curIncr -= incr;
                    thetaGroups[curIncr].Next = thetaGroups[0];
                    //thetaGroups[0].Prev = thetaGroups[curIncr];

                    

                    return thetaGroups;
                }
                
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

                /*
                public ThetaGroup(float _min, float _max)
                {
                    Min = _min;
                    Max = _max;
                    Mid = Min + (Max - Min) / 2f;
                }
                */

                public void Add(FrontierPt _pt)
                {
                    if (First == null)
                    {
                        First = _pt;
                        Count++;
                        return;
                    }

                    var newTheta = _pt.Theta;
                    var firstTheta = First.Theta;
                    if (firstTheta > newTheta)
                        First = _pt;
                    else if (_pt.Right == First) First = _pt;

                    Count++;
                }


                public void Remove(FrontierPt _pt)
                {
                    if (_pt == First) First = Count == 1 ? null : First.Right;

                    Count--;
                }
            }

            public class FrontierPt : FastPriorityQueueNode, ICircleSweepObj
            {
                #region Constructors

                public FrontierPt(Delaunay.Triangle.HalfEdge _edge, CircleSweep _cs)
                {
                    EdgeRight = _edge;
                    CS = _cs;
                }

                #endregion Constructors

                #region Fields

                public int VertIdx => EdgeRight.OriginIdx;

                public Delaunay.Triangle.HalfEdge EdgeRight;
                public FrontierPt Left;
                public FrontierPt Right;
                public FrontierPt this[RL _dir] => _dir == RL.Left ? Left : Right;

                public float Theta => CS.m_PolPos[VertIdx].Theta;
                public float r => CS.m_PolPos[VertIdx].r;
                public PolartPt PolPos => CS.m_PolPos[VertIdx];

                #endregion Fields

                #region Properties

                public CircleSweep CS { get; }

                public int EdgeRightIdx { get; private set; }

                #endregion Properties

                #region Methods

                public void Dispose()
                {
                    Left = null;
                    Right = null;
                }

                public override string ToString()
                {
                    var er = (EdgeRight ?? (object) "").ToString();
                    return "Fnt Pt t: " + Theta + " v: " + VertIdx + " RtV: " + Right.VertIdx + " LtV: " +
                           Left.VertIdx + " EdgeRt: " + er;
                }

                #endregion Methods
            }

            #endregion Nested Types
        }

        public class PolartPt : ICircleSweepObj
        {
            #region Properties

            public CircleSweep CS { get; }

            #endregion Properties

            #region Fields

            public readonly float r;
            public readonly float Theta;
            public Frontier.ThetaGroup thetaGroup;

            #endregion Fields

            #region Constructors

            public PolartPt(float _radius, float _theta, CircleSweep _cs)
            {
                r = _radius;
                Theta = _theta;
                CS = _cs;
            }

            public PolartPt(Vector2 _cPt, Vector2 _origin, CircleSweep _cs)
            {
                CS = _cs;

                var vec = _cPt - _origin;
                r = vec.magnitude; //TODO optimize (use sqr?)

                var from = Vector2.right - _origin;
                Theta = from.AngleCW(vec);
                //var nx = _cPt.x - _origin.x;
                //var ny = _cPt.y - _origin.y;

                //r = (float) Math.Sqrt(nx * nx + ny * ny);
                //Theta = AngleFromOrigin(_origin, _cPt);
            }

            #endregion Constructors
        }

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

        #endregion Nested Types
    }
}