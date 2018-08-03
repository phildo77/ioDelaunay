using System.Runtime.CompilerServices;
using Priority_Queue;

namespace ioDelaunay
{
    using System;
    using System.Collections;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Linq;

    using ioPolygonGraph;

    using Vectorf;

    public class CircleSweep : Delaunay.Triangulator
    {
        #region Fields

        public Rectf Bounds => new Rectf(D.BoundsRect);
        public Frontier frontier;

        // Change
        // Don't change
        private PolartPt[] m_PolPos; //By Point Idx
        private List<int> m_VertIdxsByR;

        #endregion Fields

        #region Enumerations

        public enum RL
        {
            Right,
            Left
        }

        #endregion Enumerations

        #region Nested Interfaces

        private interface ICircleSweepObj
        {
            #region Properties

            CircleSweep CS
            {
                get;
            }

            #endregion Properties
        }

        #endregion Nested Interfaces

        #region Properties

        public Vector2f Origin
        {
            get; private set;
        }

        #endregion Properties

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

            for (var rIdx = 0; rIdx < m_VertIdxsByR.Count; ++rIdx)
            {
                var ofVertIdx = m_VertIdxsByR[rIdx];

                // 1) Project
                var thetaHit = PolPos(ofVertIdx).Theta;
                var fntVerts = frontier.Project(thetaHit);

                // 2) Create Tri and Legalize
                var tri = D.AddTriToMesh(ofVertIdx, fntVerts[0].EdgeRight);
                var newFntPt = frontier.Add(ofVertIdx, tri, fntVerts[0], fntVerts[1]);
                
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
            var fStart = frontier.Project(0)[0];
            var fScan = fStart;
            var firstScanDone = false;
            var maxHullAngle = Settings.ForceConvexHull
                ? Math.PI - float.Epsilon
                : Settings.MaxHullAngleDegrees * (float) (Math.PI / 180f);

            while (fScan.VertIdx != fStart.VertIdx || !firstScanDone)
            {
                var vecL = fScan.Left.Pos - fScan.Pos;
                var vecR = fScan.Right.Pos - fScan.Pos;
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

                    frontier.Remove(fScan.Right, twinLt.Twin.NextEdge);
                    D.Legalize(twinLt.Twin, twinRt.Twin);
                    //LegalizeFrontier(changedVerts);

                    vecL = fScan.Left.Pos - fScan.Pos;
                    vecR = fScan.Right.Pos - fScan.Pos;
                }

                fScan = fScan.Right;
                if (!firstScanDone && fScan.VertIdx != fStart.VertIdx) firstScanDone = true;
            }
        }

        protected override void Hull()
        {
            var fpCnt = frontier.FPntCount;
            var curCnt = 0;
            var fScan = frontier.Project(0)[0];
            Frontier.FrontierPt fStart = fScan;
            D.HullIdxs = new List<int>() {fStart.VertIdx};
            fScan = fScan.Right;
            while (fScan.VertIdx != fStart.VertIdx)
            {
                D.HullIdxs.Add(fScan.VertIdx);
                fScan = fScan.Right;
            }
        }

        /// <summary>
        ///     Performs a binary search on the specified collection.
        /// </summary>
        /// <typeparam name="TItem">The type of the item.</typeparam>
        /// <typeparam name="TSearch">The type of the searched item.</typeparam>
        /// <param name="list">The list to be searched.</param>
        /// <param name="value">The value to search for.</param>
        /// <param name="comparer">The comparer that is used to compare the value with the list items.</param>
        /// <returns></returns>
        private static int BinarySearch<TItem, TSearch>(IList<TItem> list, TSearch value,
            Func<TSearch, TItem, int> comparer)
        {
            //if (list == null) throw new ArgumentNullException("list");
            //if (comparer == null) throw new ArgumentNullException("comparer");

            var lower = 0;
            var upper = list.Count - 1;

            while (lower <= upper)
            {
                var middle = lower + (upper - lower) / 2;
                var comparisonResult = comparer(value, list[middle]);
                if (comparisonResult < 0)
                    upper = middle - 1;
                else if (comparisonResult > 0)
                    lower = middle + 1;
                else
                    return middle;
            }

            return ~lower;
        }

        /// <summary>
        ///     Performs a binary search on the specified collection.
        /// </summary>
        /// <typeparam name="TItem">The type of the item.</typeparam>
        /// <param name="list">The list to be searched.</param>
        /// <param name="value">The value to search for.</param>
        /// <returns></returns>
        private static int BinarySearch<TItem>(IList<TItem> list, TItem value)
        {
            return BinarySearch(list, value, Comparer<TItem>.Default);
        }

        /// <summary>
        ///     Performs a binary search on the specified collection.
        /// </summary>
        /// <typeparam name="TItem">The type of the item.</typeparam>
        /// <param name="list">The list to be searched.</param>
        /// <param name="value">The value to search for.</param>
        /// <param name="comparer">The comparer that is used to compare the value with the list items.</param>
        /// <returns></returns>
        private static int BinarySearch<TItem>(IList<TItem> list, TItem value, IComparer<TItem> comparer)
        {
            return BinarySearch(list, value, comparer.Compare);
        }

        private Vector2f CalcOrigin(out int[] _firstTriIdxs)
        {
            var cent = Bounds.center;
            var closest = new SortedList<float, int>
            {
                { (cent - D.Points[0]).sqrMagnitude, 0},
                { (cent - D.Points[1]).sqrMagnitude, 1},
                { (cent - D.Points[2]).sqrMagnitude, 2}
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
            while (Geom.AreColinear(D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]]))
            {
                _firstTriIdxs[2] = closest[findNonLineIdx++];
            }

            //Force Clockwise
            var v0 = D.Points[_firstTriIdxs[0]];
            var v1 = D.Points[_firstTriIdxs[1]];
            var v2 = D.Points[_firstTriIdxs[2]];
            var angleCCW = Vector2f.SignedAngle(v1 - v0, v2 - v0);
            if (angleCCW > 0)
            {
                var tempIdx = _firstTriIdxs[1];
                _firstTriIdxs[1] = _firstTriIdxs[2];
                _firstTriIdxs[2] = tempIdx;
            }

            var triPts = new[] {D.Points[_firstTriIdxs[0]], D.Points[_firstTriIdxs[1]], D.Points[_firstTriIdxs[2]]};
            var cc = Geom.CentroidOfPoly(triPts);
            return cc;
        }

        private void FillBasin(RL _dir, Frontier.FrontierPt _fpt)
        {
            //Setup
            var fvi = _fpt;
            var fvrp = fvi[_dir][_dir];
            var polvi = PolPos(fvi.VertIdx);
            var polvrp = PolPos(fvrp.VertIdx);

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
                var vecL = fBasEnd[RL.Left].Pos - fBasEnd.Pos;
                var vecR = fBasEnd[RL.Right].Pos - fBasEnd.Pos;
                var angleCW = vecL.AngleCW(vecR);
                if (angleCW >= Math.PI - float.Epsilon) break;
                fBasEnd = fBasEnd[_dir];
                ptCount++;
            }

            if (fBasEnd.VertIdx == fBasMin.VertIdx || fBasStart[_dir].VertIdx == fBasEnd.VertIdx) return;

            //Sort Basin points by R
            //var basinPts = new Dictionary<Frontier.FrontierPt, float> {{fBasEnd, PolPos(fBasEnd.VertIdx).r}};
            var basinPtQ = new FastPriorityQueue<Frontier.FrontierPt>(ptCount);
            var fBasScan = fBasStart;
            while (fBasScan.VertIdx != fBasEnd.VertIdx)
            {
                //basinPts.Add(fBasScan, PolPos(fBasScan.VertIdx).r);
                basinPtQ.Enqueue(fBasScan, m_PolPos[fBasScan.VertIdx].r);
                fBasScan = fBasScan[_dir];
            }

            //var basinPtsByR = basinPts.OrderBy(_kvp => _kvp.Value).Select(_kvp => _kvp.Key).ToList();

            //Triangluate Basin

            if (basinPtQ.Count < 3) return;
            var newVerts = new[]
            {
                basinPtQ.Dequeue(),
                basinPtQ.Dequeue(),
                basinPtQ.Dequeue()
            };
            
            //while (basinPtsByR.Count > 2)
            while(basinPtQ.Count > 2)
            {
                //Find frontier point that will be removed
                Frontier.FrontierPt fOut = newVerts[0];
                if (!newVerts.Contains(fOut[RL.Right]) || !newVerts.Contains(fOut[RL.Left]))
                    fOut = newVerts[1];

                //Check that tri is valid (due to radius from Origin)
                var vecLt = fOut.Left.Pos - fOut.Pos;
                var vecRt = fOut.Right.Pos - fOut.Pos;
                //Check for lines
                var triAngle = vecLt.AngleCW(vecRt);
                if (triAngle >= Math.PI - float.Epsilon) break;

                var twinLt = fOut[RL.Left].EdgeRight;
                var twinRt = fOut.EdgeRight;

                var newTri = D.AddTriToMesh(twinLt, twinRt);
                if (newTri == null) break;
                frontier.Remove(fOut, twinLt.Twin.NextEdge);

                D.Legalize(twinLt.Twin, twinRt.Twin);

                //basinPtsByR.Remove(fOut);

                newVerts[0] = newVerts[1];
                newVerts[1] = newVerts[2];
                newVerts[2] = basinPtQ.Dequeue();
            }
        }

        /*
        private void DebugCheckFrontier()
        {
            var proj = frontier.Project(0);
            var fPt = proj[1];
            var firstfPt = fPt;
            fPt = fPt.Right;
            var fList = new List<Frontier.FrontierPt>();
            while (fPt.VertIdx != firstfPt.VertIdx)
            {
                fList.Add(fPt.Left);
                var t0 = m_PolPos[fPt.Left.VertIdx].Theta;
                var t1 = m_PolPos[fPt.VertIdx].Theta;

                if (t1 < t0)
                {
                    Trace.WriteLine("Debug");
                }

                fPt = fPt.Right;



            }
            
        }
*/
        private void Init()
        {
            int[] firstTriIdxs;
            Origin = CalcOrigin(out firstTriIdxs);
            m_PolPos = new PolartPt[D.Points.Count]; //TODO make this dynamic to save memory?
            m_VertIdxsByR = new List<int>(D.Points.Count - 3);
            for (int pIdx = 0; pIdx < D.Points.Count; ++pIdx)
            {
                m_PolPos[pIdx] = new PolartPt(D.Points[pIdx], Origin, this);
                if (firstTriIdxs.Contains(pIdx)) continue;
                m_VertIdxsByR.Add(pIdx);
            }

            m_VertIdxsByR.Sort((_a, _b) => m_PolPos[_a].r.CompareTo(m_PolPos[_b].r));
            
            //Sort Indexes By Distance to Origin
            /*
            var vertIdxsToSort = new List<int>();
            for (var idx = 0; idx < D.Points.Count; ++idx)
                vertIdxsToSort.Add(idx);
            vertIdxsToSort.Remove(firstTriIdxs[0]);
            vertIdxsToSort.Remove(firstTriIdxs[1]);
            vertIdxsToSort.Remove(firstTriIdxs[2]);
            //Sort CW
            m_VertIdxsByR = vertIdxsToSort.OrderBy(_idx => m_PolPos[_idx].r).ToArray();
*/
            //Init Frontier
            var firstTri = new Delaunay.Triangle(firstTriIdxs[0], firstTriIdxs[1], firstTriIdxs[2], D);

            frontier = new Frontier(firstTri, this);
        }

        private PolartPt PolPos(int _vertIdx)
        {
            return m_PolPos[_vertIdx];
        }

        private void Walk(RL _dir, Frontier.FrontierPt _fPt)
        {
            var fvi = _fPt;

            while (true)
            {
                var fvn = fvi[_dir];

                var vecL = fvn[RL.Left].Pos - fvn.Pos;
                var vecR = fvn[RL.Right].Pos - fvn.Pos;

                var angleCW = vecL.AngleCW(vecR);
                if (angleCW > Math.PI / 2f) break;

                var twinLt = fvn[RL.Left].EdgeRight;
                var twinRt = fvn.EdgeRight;
                var newTri = D.AddTriToMesh(twinLt, twinRt);
                if (newTri == null) break;
                frontier.Remove(fvn, twinLt.Twin.NextEdge);

                D.Legalize(twinLt.Twin, twinRt.Twin);
            }
        }

        #endregion Methods

        #region Nested Types

        
        //TODO PRIVATE?
        public class Frontier : ICircleSweepObj
        {
            #region Fields

            public int FPntCount => m_FPList.Count;


            private readonly FPList m_FPList;

            #endregion Fields

            #region Constructors

            public Frontier(Delaunay.Triangle _firstTri, CircleSweep _cs)
            {
                CS = _cs;
                m_FPList = new FPList(_cs);

                var ftPts = new List<FrontierPt>();
                for (int eIdx = 0; eIdx < 3; ++eIdx)
                {
                    var edge = _firstTri.Edge(eIdx);
                    ftPts.Add(new FrontierPt(edge.OriginIdx, edge, CS));
                }

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

            public FrontierPt Add(int _vIdx, Delaunay.Triangle _newTri, FrontierPt _fLt,
                FrontierPt _fRt)
            {
                var newEdge = _newTri.EdgeWithOrigin(_vIdx);
                var fNew = new FrontierPt(_vIdx, newEdge, CS);

                //TODO Debug
                var debugthetaNew = CS.m_PolPos[fNew.VertIdx].Theta;
                var debugTL = CS.m_PolPos[_fLt.VertIdx].Theta;
                var debugTR = CS.m_PolPos[_fRt.VertIdx].Theta;
                if(debugthetaNew == debugTL || debugthetaNew == debugTR)
                    Trace.WriteLine("Debug");
                
                _fLt.Right = _fRt.Left = fNew;
                _fLt.EdgeRight = _newTri.EdgeWithOrigin(_fLt.VertIdx);

                fNew.Left = _fLt;
                fNew.Right = _fRt;

                m_FPList.Add(fNew); //TODO add epsilon offset for dupe?
                return fNew;
            }


            public FrontierPt[] Project(float _theta)
            {
                return m_FPList.Project(_theta);
            }

            public void Remove(FrontierPt _fPt, PolygonGraph.Poly.HalfEdge _newEdgeRight)
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

                public readonly int VertIdx;

                public PolygonGraph.Poly.HalfEdge EdgeRight;
                public FrontierPt Left;
                public Vector2f Pos => CS.D.Points[VertIdx];
                public FrontierPt Right;
                public FrontierPt this[RL _dir] => _dir == RL.Left ? Left : Right;
                public FPList.ThetaGroup ParentGroup; //For fast removal

                public float Theta => CS.m_PolPos[VertIdx].Theta;
                public float r => CS.m_PolPos[VertIdx].r;

                #endregion Fields

                #region Constructors

                public FrontierPt(int vertIdx, PolygonGraph.Poly.HalfEdge _edgeRight, CircleSweep _cs)
                {
                    VertIdx = vertIdx;
                    EdgeRight = _edgeRight;
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

                public FPList(CircleSweep _cs)
                {
                    CS = _cs;
                    Count = 0;
                    
                    //Init theta groups
                    var k = 1 + (int) Math.Pow(_cs.D.Points.Count, 1f / 3f);
                    var prevTg = new ThetaGroup(_cs);
                    var tgList = new SortedList<float, ThetaGroup>(k)
                    {
                        {0, prevTg}
                    };
                    for (int grpIdx = 1; grpIdx < k - 1; ++grpIdx)
                    {
                        var tg = new ThetaGroup(_cs);
                        var theta = 2f * (float) Math.PI * grpIdx / k;
                        tg.DebugTheta = theta;
                        tgList.Add(theta, tg);
                        prevTg.Next = tg;
                        tg.Prev = prevTg;
                        prevTg = tg;
                    }

                    tgList.Values[tgList.Count - 1].Next = tgList.Values[0];
                    tgList.Values[0].Prev = tgList.Values[tgList.Count - 1];

                    m_ThetaGroups = tgList;
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
                    var polPos = CS.m_PolPos[_pt.VertIdx];
                    var grp = GetThetaGroup(polPos.Theta);
                    grp.Add(_pt);
                    Count++;
                }

                public FrontierPt[] Project(float _theta)
                {
                    var grp = GetThetaGroup(_theta);
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
                        var fPtRt = grp.FindNextHigher(_theta);
                        if (fPtRt != null)
                            return new[] {fPtRt.Left, fPtRt};
                        grp = grp.Next;
                        while (grp.Count == 0)
                            grp = grp.Next;
                        fPtRt = grp.FindFirstLeftMost();
                        fLtRt = new[] {fPtRt.Left, fPtRt};
                    }

                    return fLtRt;
                    /*
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
                    */
                }

                public void Remove(FrontierPt _pt)
                {
                    /*
                    var theta = CS.m_PolPos[_pt.VertIdx].Theta;
                    var grp = GetThetaGroup(theta);
                    grp.Remove(_pt);
                    */
                    _pt.ParentGroup.Remove(_pt);
                    _pt.ParentGroup = null;
                    Count--;
                }

                private ThetaGroup GetThetaGroup(float _theta)
                {
                    var grpIdx = BinarySearch(m_ThetaGroups.Keys, _theta);
                    if (grpIdx >= 0) return m_ThetaGroups.Values[grpIdx];
                    grpIdx = ~grpIdx;
                    if (grpIdx == m_ThetaGroups.Count)
                        grpIdx = 0;
                    return m_ThetaGroups.Values[grpIdx].Prev;

                }

                #endregion Methods

                #region Other

                public class ThetaGroup
                {
                    
                    private SortedList<float, SortedList<float, FrontierPt>> m_FptGrp;
                    public ThetaGroup Next;
                    public ThetaGroup Prev;
                    public readonly CircleSweep CS;
                    public float DebugTheta;

                    public int Count
                    {
                        get; private set;
                    }

                    public ThetaGroup(CircleSweep _cs)
                    {
                        m_FptGrp = new SortedList<float, SortedList<float, FrontierPt>>();
                        CS = _cs;
                    }

                    public void Add(FrontierPt _pt)
                    {
                        var theta = CS.m_PolPos[_pt.VertIdx].Theta;
                        if (!m_FptGrp.ContainsKey(theta))
                            m_FptGrp.Add(theta, new SortedList<float, FrontierPt> {{_pt.r, _pt}});
                        else
                            m_FptGrp[theta].Add(_pt.r, _pt);

                        _pt.ParentGroup = this;

                        Count++;
                    }

                    public void Remove(FrontierPt _pt)
                    {
                        var theta = CS.m_PolPos[_pt.VertIdx].Theta;
                        m_FptGrp[theta].Remove(_pt.r);
                        if (m_FptGrp[theta].Count == 0)
                            m_FptGrp.Remove(theta);
                        Count--;
                    }

                    public FrontierPt FindFirstLeftMost()
                    {
                        var firstList = m_FptGrp.Values[0];
                        foreach (var fPt in firstList.Values)
                        {
                            if (!firstList.Values.Contains(fPt.Left))
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

                        var fPtList = m_FptGrp[keys[idx]];
                        foreach (var fPt in fPtList.Values)
                        {
                            if (!fPtList.Values.Contains(fPt.Left))
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

        public class PolartPt : ICircleSweepObj
        {
            #region Fields

            public readonly float r;
            public readonly float Theta;

            #endregion Fields

            #region Constructors

            public PolartPt(float _radius, float _theta, CircleSweep _cs)
            {
                r = _radius;
                Theta = _theta;
                CS = _cs;
            }

            public PolartPt(Vector2f _cPt, Vector2f _origin, CircleSweep _cs)
            {
                CS = _cs;

                var vec = _cPt - _origin;
                r = vec.magnitude;  //TODO optimize (use sqr?)

                var from = Vector2f.right - _origin;
                Theta = from.AngleCW(vec);
                //var nx = _cPt.x - _origin.x;
                //var ny = _cPt.y - _origin.y;

                //r = (float) Math.Sqrt(nx * nx + ny * ny);
                //Theta = AngleFromOrigin(_origin, _cPt);
            }

            #endregion Constructors

            #region Properties

            public CircleSweep CS
            {
                get;
            }

            #endregion Properties
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