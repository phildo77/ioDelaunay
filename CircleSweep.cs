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

        /*
        private HashSet<Guid> GetTrisContainingAllVerts(params int[] _vertIdxs)
        {
            var trisContaining = new HashSet<Guid>(PolysContainingVert[_vertIdxs[0]]);
            for (var idx = 1; idx < _vertIdxs.Length; ++idx)
                trisContaining.IntersectWith(PolysContainingVert[_vertIdxs[idx]]);
            return trisContaining;
        }
        */
        protected override void Algorithm()
        {
            Init();

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
                    var newTri = D.AddTriToMesh(twinLt, twinRt);
                    if (newTri == null) //Straight line check
                        break;
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

                fScan = fScan.Right;
                if (!firstScanDone && fScan.VertIdx != fStart.VertIdx) firstScanDone = true;
            }
        }

        protected override void Hull()
        {
            var fScan = frontier.LastAddedFPt;
            var fStart = fScan;
            D.HullIdxs = new List<int> {fStart.VertIdx};
            fScan = fScan.Right;
            while (fScan.VertIdx != fStart.VertIdx)
            {
                D.HullIdxs.Add(fScan.VertIdx);
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
            var polvi = PolPos(fvi.VertIdx);
            var polvrp = PolPos(fvrp.VertIdx);
            var pts = D.Points;

            var r2 = polvrp.r;
            var dr = polvi.r - r2;
            var dTheta = _dir == RL.Right ? polvrp.Theta - polvi.Theta : polvi.Theta - polvrp.Theta;

            //Check heuristic
            var h = dr / (r2 * dTheta);
            if (h < 2) return;

            //Find Basin Min 
            var ptCount = 2;
            var fBasStart = fvrp;
            var fBasMin = fBasStart;
            while (PolPos(fBasMin.VertIdx).r > PolPos(fBasMin[_dir].VertIdx).r)
            {
                fBasMin = fBasMin[_dir];
                ptCount++;
            }


            //Find Basin End 
            var fBasEnd = fBasMin;
            while (true)
            {
                var fbePos = pts[fBasEnd.VertIdx];

                var vecL = pts[fBasEnd[RL.Left].VertIdx] - fbePos;
                var vecR = pts[fBasEnd[RL.Right].VertIdx] - fbePos;

                //var vecL = fBasEnd[RL.Left].Pos - fBasEnd.Pos;
                //var vecR = fBasEnd[RL.Right].Pos - fBasEnd.Pos;
                //var angleCW = vecL.AngleCW(vecR);

                //Positive Cross Product greater than 180
                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ > 0) break;

                fBasEnd = fBasEnd[_dir];
                ptCount++;
            }

            if (fBasEnd.VertIdx == fBasMin.VertIdx || fBasStart[_dir].VertIdx == fBasEnd.VertIdx) return;

            //Sort Basin points by R
            //var basinPts = new Dictionary<Frontier.FrontierPt, float> {{fBasEnd, PolPos(fBasEnd.VertIdx).r}};
            //var basinPtQ = new FastPriorityQueue<Frontier.FrontierPt>(ptCount);
            m_BasinPtQ.Reset(ptCount);
            var fBasScan = fBasStart;
            while (fBasScan.VertIdx != fBasEnd.VertIdx)
            {
                //basinPts.Add(fBasScan, PolPos(fBasScan.VertIdx).r);
                m_BasinPtQ.Enqueue(fBasScan, m_PolPos[fBasScan.VertIdx].r);
                fBasScan = fBasScan[_dir];
            }

            //var basinPtsByR = basinPts.OrderBy(_kvp => _kvp.Value).Select(_kvp => _kvp.Key).ToList();

            //Triangluate Basin

            if (m_BasinPtQ.Count < 3) return;
            var newVerts = new[]
            {
                m_BasinPtQ.Dequeue(),
                m_BasinPtQ.Dequeue(),
                m_BasinPtQ.Dequeue()
            };

            //while (basinPtsByR.Count > 2)
            while (m_BasinPtQ.Count > 2)
            {
                //Find frontier point that will be removed
                var fOut = newVerts[0];
                if (!newVerts.Contains(fOut[RL.Right]) || !newVerts.Contains(fOut[RL.Left]))
                    fOut = newVerts[1];

                //Check that tri is valid (due to radius from Origin)
                var fOutPos = pts[fOut.VertIdx];
                var vecL = pts[fOut.Left.VertIdx] - fOutPos;
                var vecR = pts[fOut.Right.VertIdx] - fOutPos;

                //var vecLt = fOut.Left.Pos - fOut.Pos;
                //var vecRt = fOut.Right.Pos - fOut.Pos;

                //Check for lines
                //Positive Cross Product greater than 180
                var crossZ = vecL.x * vecR.y - vecL.y * vecR.x;
                if (crossZ > 0) break;

                var twinLt = fOut[RL.Left].EdgeRight;
                var twinRt = fOut.EdgeRight;

                var newTri = D.AddTriToMesh(twinLt, twinRt);
                if (newTri == null) break;
                frontier.Remove(fOut, twinLt.m_Twin.NextEdge);

                //D.Legalize(twinLt.Twin, twinRt.Twin);
                D.Legalize(twinLt, twinRt);

                //basinPtsByR.Remove(fOut);

                newVerts[0] = newVerts[1];
                newVerts[1] = newVerts[2];
                newVerts[2] = m_BasinPtQ.Dequeue();
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
            Origin = CalcOrigin(out firstTriIdxs);
            var ptCnt = D.Points.Count;
            List<float> thetaKeys;
            var thetaGroups = Frontier.ThetaGroup.BuildThetaGroups(ptCnt, out thetaKeys);
            m_PolPos = new PolartPt[ptCnt]; //TODO make this dynamic to save memory?
            m_VertIdxsByR = new int[ptCnt - 3];
            var idxOffset = 0;
            for (var pIdx = 0; pIdx < D.Points.Count; ++pIdx)
            {
                //Store polar position
                var polPos = new PolartPt(D.Points[pIdx], Origin, this);
                m_PolPos[pIdx] = polPos;

                //Reference theta group
                var grpIdx = BinarySearch(thetaKeys, polPos.Theta);
                if (grpIdx < 0)
                {
                    grpIdx = ~grpIdx;
                    if (grpIdx == thetaGroups.Count)
                        grpIdx = 0;
                }

                polPos.thetaGroup = thetaGroups[thetaKeys[grpIdx]];

                //Prepare r index reference (to be sorted later)
                if (firstTriIdxs.Contains(pIdx))
                {
                    idxOffset++;
                    continue;
                }

                m_VertIdxsByR[pIdx - idxOffset] = pIdx;
            }

            //Sort points by distance to origin
            Array.Sort(m_VertIdxsByR, (_a, _b) => m_PolPos[_a].r.CompareTo(m_PolPos[_b].r));

            //Init Frontier
            var firstTri = new Delaunay.Triangle(firstTriIdxs[0], firstTriIdxs[1], firstTriIdxs[2], D);

            frontier = new Frontier(firstTri, thetaGroups, this);
        }

        private PolartPt PolPos(int _vertIdx)
        {
            return m_PolPos[_vertIdx];
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
                var newTri = D.AddTriToMesh(twinLt, twinRt);
                if (newTri == null) break;
                frontier.Remove(fvn, twinLt.m_Twin.NextEdge);

                //D.Legalize(twinLt.Twin, twinRt.Twin);
                D.Legalize(twinLt, twinRt);
            }
        }

        #endregion Methods

        #region Nested Types

        public class FrontierDynamic
        {
            public class ThetaGroup
            {
                public const int MAX_CHILD_COUNT = 10;
                public Dictionary<int, ThetaGroup> Children;
                public FrontierPt First;
                public int FPCount;
                public float ThetaRangeInv;
                public float ThetaStart;

                public ThetaGroup(float _thetaStart, float _thetaRange)
                {
                    ThetaStart = _thetaStart;
                    ThetaRangeInv = 1 / _thetaRange;
                }

                public int GetKey(float _theta)
                {
                    return (int) ((_theta - ThetaStart) * MAX_CHILD_COUNT * ThetaRangeInv);
                }

                public void Add(FrontierPt _newPt, FrontierPt _fPtRt)
                {
                    if (Children != null)
                    {
                        var key = GetKey(_newPt.Theta);
                        Children[key].Add(_newPt, _fPtRt);
                    }
                    else if (++FPCount >= MAX_CHILD_COUNT)
                    {
                        Children = new Dictionary<int, ThetaGroup>();
                        var thetaStart = ThetaStart;
                        var incr = 1 / (ThetaRangeInv * MAX_CHILD_COUNT);
                        for (var idx = 0; idx < MAX_CHILD_COUNT; ++idx)
                            Children.Add(idx, new ThetaGroup(thetaStart, idx * incr));

                        Children[GetKey(_newPt.Theta)].Add(_newPt, _fPtRt);
                    }
                    else
                    {
                        _newPt.Left = _fPtRt.Left;
                        _newPt.Right = _fPtRt;
                        _fPtRt.Left.Right = _newPt;
                        _fPtRt.Left = _newPt;
                        if (First.Theta >= _newPt.Theta)
                            if (_newPt.Left != First)
                                First = _newPt;
                    }
                }

                public FrontierPt[] Project(float _theta)
                {
                    if (Children != null) return Children[GetKey(_theta)].Project(_theta);

                    var fScan = First;
                    while (fScan.Theta > _theta)
                        fScan = fScan.Right;
                    return new[] {fScan.Left, fScan};
                }

                public void Remove(FrontierPt _pt)
                {
                    if (Children != null)
                    {
                        Children[GetKey(_pt.Theta)].Remove(_pt);
                    }
                    else
                    {
                        var fScan = First;
                        while (fScan.VertIdx != _pt.VertIdx)
                            fScan = fScan.Right;
                        fScan.Right.Left = fScan.Left;
                        fScan.Left.Right = fScan.Right;
                        FPCount--;
                    }
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
                    return "Fnt Pt v: " + VertIdx + " RtV: " + Right.VertIdx + " LtV: " + Left.VertIdx + " EdgeRt: " +
                           er;
                }

                #endregion Methods
            }
        }

        public class Frontier : ICircleSweepObj
        {
            #region Constructors

            public Frontier(Delaunay.Triangle _firstTri, Dictionary<float, ThetaGroup> _tgs, CircleSweep _cs)
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

                m_ThetaGroups = _tgs;

                CS.m_PolPos[ftPts[0].VertIdx].thetaGroup.Add(ftPts[0]);
                CS.m_PolPos[ftPts[1].VertIdx].thetaGroup.Add(ftPts[1]);
                CS.m_PolPos[ftPts[2].VertIdx].thetaGroup.Add(ftPts[2]);
            }

            #endregion Constructors

            #region Properties

            public CircleSweep CS { get; }

            #endregion Properties

            #region Fields

            private Dictionary<float, ThetaGroup> m_ThetaGroups;

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
                public static Dictionary<float, ThetaGroup> BuildThetaGroups(int _pntCount, out List<float> _keys)
                {
                    //Init theta groups
                    var k = 1 + (int) Math.Pow(_pntCount, 1f / 3f);
                    var incr = (float) (2d * Math.PI / k);
                    var curIncr = incr;
                    //var prevTg = new ThetaGroup(0, incr);
                    _keys = new List<float> {0f};
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

        //TODO PRIVATE?
        /*
        public class Frontier : ICircleSweepObj
        {
            #region Fields

            public int FPntCount => m_FPList.Count;


            private readonly FPList m_FPList;
            public FrontierPt LastAddedFPt;

            #endregion Fields

            #region Constructors

            public Frontier(Delaunay.Triangle _firstTri, SortedList<float, FPList.ThetaGroup> _tgs, CircleSweep _cs)
            {
                CS = _cs;
                m_FPList = new FPList(_cs, _tgs);

                var ftPts = new List<FrontierPt>();

                ftPts.Add(new FrontierPt(_firstTri.Edge0, CS));
                ftPts.Add(new FrontierPt(_firstTri.Edge1, CS));
                ftPts.Add(new FrontierPt(_firstTri.Edge2, CS));

                ftPts[0].Right = ftPts[1];
                ftPts[0].Left = ftPts[2];
                ftPts[1].Right = ftPts[2];
                ftPts[1].Left = ftPts[0];
                ftPts[2].Right = ftPts[0];
                ftPts[2].Left = ftPts[1];

                m_FPList.Add(ftPts[0]);
                m_FPList.Add(ftPts[1]);
                m_FPList.Add(ftPts[2]);
            }

            #endregion Constructors

            #region Properties

            public CircleSweep CS
            {
                get;
            }

            #endregion Properties

            #region Methods

            public FrontierPt Add(FrontierPt _fLt, FrontierPt _fRt)
            {
                var newEdgeLt = _fLt.EdgeRight.m_Twin.NextEdge;
                var newEdgeRt = newEdgeLt.NextEdge;
                var fNew = new FrontierPt(newEdgeRt, CS);

                
                _fLt.Right = _fRt.Left = fNew;
                _fLt.EdgeRight = newEdgeLt;

                fNew.Left = _fLt;
                fNew.Right = _fRt;

                m_FPList.Add(fNew); //TODO add epsilon offset for dupe?
                LastAddedFPt = fNew;
                return fNew;
            }


            public FrontierPt[] Project(PolartPt _pPt)
            {
                return m_FPList.Project(_pPt);
            }

            public void Remove(FrontierPt _fPt, Delaunay.Triangle.HalfEdge _newEdgeRight)
            {
                var fpLeft = _fPt.Left;
                var fpRight = _fPt.Right;
                fpLeft.Right = fpRight;
                fpRight.Left = fpLeft;

                m_FPList.Remove(_fPt);
                fpLeft.EdgeRight = _newEdgeRight;
            }

            #endregion Methods

            #region Nested Types

            public class FrontierPt : FastPriorityQueueNode, ICircleSweepObj
            {
                #region Fields

                public int VertIdx => EdgeRight.OriginIdx;

                public Delaunay.Triangle.HalfEdge EdgeRight;
                public FrontierPt Left;
                public FrontierPt Right;
                public FrontierPt this[RL _dir] => _dir == RL.Left ? Left : Right;

                public float Theta => CS.m_PolPos[VertIdx].Theta;
                public float r => CS.m_PolPos[VertIdx].r;

                #endregion Fields

                #region Constructors

                public FrontierPt(Delaunay.Triangle.HalfEdge _edge, CircleSweep _cs)
                {
                    EdgeRight = _edge;
                    CS = _cs;
                }

                #endregion Constructors

                #region Properties

                public CircleSweep CS
                {
                    get;
                }

                public int EdgeRightIdx
                {
                    get; private set;
                }

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
                    return "Fnt Pt v: " + VertIdx + " RtV: " + Right.VertIdx + " LtV: " + Left.VertIdx + " EdgeRt: " + er;
                }

                #endregion Methods
            }

            public class FPList : ICircleSweepObj
            {
                #region Fields

                private SortedList<float, ThetaGroup> m_ThetaGroups;

                #endregion Fields

                #region Constructors

                /// <summary>
                /// We'll need the theta groups built for init before the frontier is constructed.
                /// </summary>
                /// <param name="_pntCount"></param>
                /// <returns></returns>
                public static SortedList<float, ThetaGroup> BuildThetaGroups(int _pntCount)
                {
                    //Init theta groups
                    var k = 1 + (int) Math.Pow(_pntCount, 1f / 3f);
                    var prevTg = new ThetaGroup();
                    var tgList = new SortedList<float, ThetaGroup>(k)
                    {
                        {0, prevTg}
                    };
                    for (int grpIdx = 1; grpIdx < k - 1; ++grpIdx)
                    {
                        var tg = new ThetaGroup();
                        var theta = 2f * (float) Math.PI * grpIdx / k;
                        tgList.Add(theta, tg);
                        prevTg.Next = tg;
                        tg.Prev = prevTg;
                        prevTg = tg;
                    }

                    tgList.Values[tgList.Count - 1].Next = tgList.Values[0];
                    tgList.Values[0].Prev = tgList.Values[tgList.Count - 1];

                    return tgList;
                }
                
                public FPList(CircleSweep _cs, SortedList<float, ThetaGroup> _thetaGroups)
                {
                    CS = _cs;
                    Count = 0;
                    m_ThetaGroups = _thetaGroups;
                }

                #endregion Constructors

                #region Properties

                public int Count
                {
                    get; private set;
                }

                public CircleSweep CS
                {
                    get;
                }

                #endregion Properties

                #region Methods

                public void Add(FrontierPt _pt)
                {
                    CS.m_PolPos[_pt.VertIdx].thetaGroup.Add(_pt);
                    Count++;
                }

                public FrontierPt[] Project(PolartPt _ppt)
                {
                    var grp = _ppt.thetaGroup;
                    FrontierPt[] fLtRt;
                    if (grp.Count == 0)
                    {
                        while (grp.Count == 0)
                            grp = grp.Next;
                        var fPtRt = grp.FindFirstLeftMost();
                        fLtRt = new[] {fPtRt.Left, fPtRt};
                    }
                    else
                    {
                        var fPtRt = grp.FindNextHigher(_ppt.Theta);
                        if (fPtRt != null)
                            return new[] {fPtRt.Left, fPtRt};

                        var fPtLt = grp.FindLastRightMost();
                        return new[] {fPtLt, fPtLt.Right};
                        //TODO
                        grp = grp.Next;
                        while (grp.Count == 0)
                            grp = grp.Next;
                        fPtRt = grp.FindFirstLeftMost();
                        fLtRt = new[] {fPtRt.Left, fPtRt};
                    }

                    return fLtRt;
                }

                public void Remove(FrontierPt _pt)
                {
                    _pt.CS.m_PolPos[_pt.VertIdx].thetaGroup.Remove(_pt);
                    Count--;
                }

                #endregion Methods

                #region Other

                public class ThetaGroup
                {
                    
                    private SortedList<float, SortedList<float, FrontierPt>> m_FptGrp;
                    public ThetaGroup Next;
                    public ThetaGroup Prev; //TODO keep?

                    public int Count
                    {
                        get; private set;
                    }

                    public ThetaGroup()
                    {
                        m_FptGrp = new SortedList<float, SortedList<float, FrontierPt>>();
                    }

                    public void Add(FrontierPt _pt)
                    {
                        var theta = _pt.CS.m_PolPos[_pt.VertIdx].Theta;
                        if (!m_FptGrp.ContainsKey(theta))
                        {
                            m_FptGrp.Add(theta, new SortedList<float, FrontierPt> {{_pt.r, _pt}});
                        }
                        else
                            m_FptGrp[theta].Add(_pt.r, _pt);

                        Count++;
                    }

                    public void Remove(FrontierPt _pt)
                    {
                        var theta = _pt.CS.m_PolPos[_pt.VertIdx].Theta;
                        var grp = m_FptGrp[theta];
                        grp.Remove(_pt.r);
                        if (grp.Count == 0)
                            m_FptGrp.Remove(theta);
                        Count--;
                    }

                    public FrontierPt FindFirstLeftMost()
                    {
                        var firstList = m_FptGrp.Values[0];
                        for (var fGrpIdx = 0; fGrpIdx < firstList.Values.Count; fGrpIdx++)
                        {
                            var fPt = firstList.Values[fGrpIdx];
                            if (!firstList.Values.Contains(fPt.Left))
                                return fPt;
                        }

                        return null;
                    }

                    public FrontierPt FindLastRightMost()
                    {
                        var lastList = m_FptGrp.Values[m_FptGrp.Count - 1];
                        for (var fGrpIdx = 0; fGrpIdx < lastList.Values.Count; fGrpIdx++)
                        {
                            var fPt = lastList.Values[fGrpIdx];
                            if (!lastList.Values.Contains(fPt.Right))
                                return fPt;
                        }

                        return null;
                    }
                    
                    public FrontierPt FindNextHigher(float _theta)
                    {
                        var keys = m_FptGrp.Keys;
                        var idx = BinarySearch(keys, _theta);
                        if (idx < 0)
                        {
                            idx = ~idx;
                            if (idx == m_FptGrp.Count)
                                return null;
                        }

                        var fPtList = m_FptGrp.Values[idx].Values;
                        for (var index = 0; index < fPtList.Count; index++)
                        {
                            var fPt = fPtList[index];
                            if (!fPtList.Contains(fPt.Left))
                                return fPt;
                        }

                        return null;
                    }
                }

                #endregion Other
            }

            private class FPListOld : ICircleSweepObj
            {
                #region Fields

                public int Count => m_FPts.Count;

                private readonly Dictionary<int, FrontierPt> m_FptByVertIdx;

                // Sorted list <theta group, theta precise, r>
                private readonly SortedList<float, SortedList<float, FrontierPt>> m_FPts;

                #endregion Fields

                #region Constructors

                public FPListOld(CircleSweep _cs)
                {
                    CS = _cs;
                    m_FptByVertIdx = new Dictionary<int, FrontierPt>();
                    m_FPts = new SortedList<float, SortedList<float, FrontierPt>>();
                }

                #endregion Constructors

                #region Properties

                public CircleSweep CS
                {
                    get;
                }

                #endregion Properties

                #region Methods

                public void Add(FrontierPt _pt)
                {
                    var polPos = CS.m_PolPos[_pt.VertIdx];
                    if (!m_FPts.ContainsKey(polPos.Theta))
                        m_FPts.Add(polPos.Theta, new SortedList<float, FrontierPt>());
                    m_FPts[polPos.Theta].Add(polPos.r, _pt);
                    m_FptByVertIdx.Add(_pt.VertIdx, _pt);
                }

                public bool ContainsVertIdx(int _idx)
                {
                    return m_FptByVertIdx.ContainsKey(_idx);
                }

                public FrontierPt GetWithVertIdx(int _idx)
                {
                    var fpt = !m_FptByVertIdx.ContainsKey(_idx) ? null : m_FptByVertIdx[_idx];
                    if (fpt == null)
                        return null;
                    return fpt;
                }

                public FrontierPt[] Project(float _theta)
                {
                    var rtPtIdx = BinarySearch(m_FPts.Keys, _theta);
                    if (rtPtIdx < 0)
                    {
                        rtPtIdx = ~rtPtIdx;
                        if (rtPtIdx == m_FPts.Count)
                            rtPtIdx = 0;
                    }

                    var fPtsAtTheta = m_FPts.Values[rtPtIdx];
                    var right = fPtsAtTheta.Values[fPtsAtTheta.Count - 1];
                    if (fPtsAtTheta.Count != 1)
                        if (fPtsAtTheta.ContainsValue(right.Left))
                            right = right.Right;

                    var ltRt = new[] {right.Left, right};
                    return ltRt;
                }

                public void Remove(FrontierPt _pt)
                {
                    var polPos = CS.m_PolPos[_pt.VertIdx];
                    m_FPts[polPos.Theta].Remove(polPos.r);
                    if (m_FPts[polPos.Theta].Count == 0)
                        m_FPts.Remove(polPos.Theta);
                    m_FptByVertIdx.Remove(_pt.VertIdx);
                }

                #endregion Methods
            }

            #endregion Nested Types
        }
*/
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