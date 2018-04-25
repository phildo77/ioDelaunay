﻿using System;
using System.Collections.Generic;
using System.Linq;
using ioPolygonGraph;
using Vectorf;

namespace ioDelaunay
{
    public class CircleSweep : Delaunay.Triangulator
    {
        public enum RL
        {
            Right,
            Left
        }

        public interface ICircleSweepObj
        {
            CircleSweep CS { get; }
        }

        public Rectf Bounds => new Rectf(D.BoundsRect);
        public Frontier frontier;

        public Vector2f Origin { get; private set; }

        // Change

        // Don't change
        private PolartPt[] m_PolPos; //By Point Idx
        private int[] m_VertIdxsByR;

        /// <summary>
        ///     Angle to origin, x axis clockwise.
        /// </summary>
        public static float AngleFromOrigin(Vector2f _origin, Vector2f _pt)
        {
            var to = _pt - _origin;
            var from = Vector2f.right - _origin;
            return from.AngleCW(to);
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
        public static int BinarySearch<TItem, TSearch>(IList<TItem> list, TSearch value,
            Func<TSearch, TItem, int> comparer)
        {
            if (list == null) throw new ArgumentNullException("list");
            if (comparer == null) throw new ArgumentNullException("comparer");

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
        public static int BinarySearch<TItem>(IList<TItem> list, TItem value)
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
        public static int BinarySearch<TItem>(IList<TItem> list, TItem value, IComparer<TItem> comparer)
        {
            return BinarySearch(list, value, comparer.Compare);
        }

        private HashSet<Guid> GetTrisContainingAllVerts(params int[] _vertIdxs)
        {
            var trisContaining = new HashSet<Guid>(PolysContainingVert[_vertIdxs[0]]);
            for (var idx = 1; idx < _vertIdxs.Length; ++idx)
                trisContaining.IntersectWith(PolysContainingVert[_vertIdxs[idx]]);
            return trisContaining;
        }


        protected override void Algorithm()
        {
            Init();
            
            for (var rIdx = 3; rIdx < m_VertIdxsByR.Length; ++rIdx)
            {
                //var debug1 = frontier.DebugScanForTwin();
                //if (debug1.Count != 0)
                //    Console.WriteLine(debug1.ToString());
                var ofVertIdx = m_VertIdxsByR[rIdx];
                var ofVert = Vertices[ofVertIdx];

                // 1) Project
                var thetaHit = PolPos(ofVert.Idx).Theta;
                var fntVerts = frontier.Project(thetaHit);

                // 2) Create Tri and Legalize
                //debug1 = frontier.DebugScanForTwin();
                //if (debug1.Count != 0)
                //    Console.WriteLine(debug1.ToString());
                var tri = D.AddTriToMesh(ofVertIdx, fntVerts[0].EdgeRight);

                var newFntPt = frontier.Add(ofVert, tri, fntVerts[0], fntVerts[1]);

                var fiData = D.Legalize(tri.ID);
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
            Hull();
        }

        private void LegalizeFrontier(HashSet<int> _affectedVerts)
        {
            foreach (var vertIdx in _affectedVerts)
            {
                if (!frontier.ContainsVert(vertIdx)) continue;
                var fPt = frontier.GetHaving(vertIdx);
                var rtIdx = fPt.Right.VertIdx;

                var commonTris = GetTrisContainingAllVerts(fPt.VertIdx, rtIdx).ToArray();

                fPt.EdgeRight = Tri(commonTris[0]).EdgeWithOrigin(vertIdx);
            }
        }

        private Vector2f CalcOrigin()
        {
            var cent = Bounds.center;
            var closest = new SortedList<float, int>();
            for (var idx = 0; idx < Points.Length; ++idx)
            {
                var pt = Points[idx];
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

            var closestPoints = new[] {closest.Values[0], closest.Values[1], closest.Values[2]};
            var triPts = new[] {Points[closestPoints[0]], Points[closestPoints[1]], Points[closestPoints[2]]};
            var cc = Geom.CentroidOfPoly(triPts);
            return cc;
        }

        private void FillBasin(RL _dir, int _viIdx)
        {
            //Setup
            var fvi = frontier.GetHaving(_viIdx);
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
                var angleCW = vecL.AngleCW(vecR);
                if (angleCW >= Math.PI - float.Epsilon) break;
                fBasEnd = fBasEnd[_dir];
            }

            if (fBasEnd.VertIdx == fBasMin.VertIdx || fBasStart[_dir].VertIdx == fBasEnd.VertIdx) return;

            //Sort Basin points by R
            var basinPts = new Dictionary<int, float> {{fBasEnd.VertIdx, PolPos(fBasEnd.VertIdx).r}};
            var fBasScan = fBasStart;
            while (fBasScan.VertIdx != fBasEnd.VertIdx)
            {
                basinPts.Add(fBasScan.VertIdx, PolPos(fBasScan.VertIdx).r);
                fBasScan = fBasScan[_dir];
            }

            var basinPtsByR = basinPts.OrderBy(_kvp => _kvp.Value).Select(_kvp => _kvp.Key).ToList();

            //Triangluate Basin

            while (basinPtsByR.Count > 2)
            {
                var newVerts = new[]
                {
                    basinPtsByR[0],
                    basinPtsByR[1],
                    basinPtsByR[2]
                };

                //Find frontier point that will be removed
                var fOut = frontier.GetHaving(newVerts[0]);
                if (!newVerts.Contains(fOut[RL.Right].VertIdx) || !newVerts.Contains(fOut[RL.Left].VertIdx))
                    fOut = frontier.GetHaving(newVerts[1]);

                //Check that tri is valid (due to radius from Origin)
                var vecLt = fOut.Left.Vert.Pos - fOut.Vert.Pos;
                var vecRt = fOut.Right.Vert.Pos - fOut.Vert.Pos;
                if (vecLt.AngleCW(vecRt) >= Math.PI - float.Epsilon) break;

                var twinLt = fOut[RL.Left].EdgeRight;
                var twinRt = fOut.EdgeRight;

                var newTri = D.AddTriToMesh(twinLt, twinRt);
                if (newTri == null) break;
                frontier.Remove(fOut);

                var changedVerts = D.Legalize(newTri.ID);
                LegalizeFrontier(changedVerts);

                basinPtsByR.Remove(fOut.VertIdx);
            }
        }

        protected override void Hull()
        {
            var hullIdxs = new List<int>();
            var fStart = frontier.Project(0)[0];
            var fScan = fStart;
            var firstScanDone = false;
            var maxHullAngle = Settings.ForceConvexHull
                ? Math.PI - float.Epsilon
                : Settings.MaxHullAngleDegrees * (float) (Math.PI / 180f);
            while (fScan.VertIdx != fStart.VertIdx || !firstScanDone)
            {
                var vecL = fScan.Left.Vert.Pos - fScan.Vert.Pos;
                var vecR = fScan.Right.Vert.Pos - fScan.Vert.Pos;
                while (vecL.AngleCW(vecR) < maxHullAngle)
                {
                    var twinLt = fScan.Left.EdgeRight;
                    var twinRt = fScan.EdgeRight;
                    var newTri = D.AddTriToMesh(twinLt, twinRt);
                    if (newTri == null) //Straight line check
                        break;
                    fScan = fScan.Left;
                    if (fStart.VertIdx == fScan.Right.VertIdx)
                        fStart = fScan.Left;
                    frontier.Remove(fScan.Right);
                    var changedVerts = D.Legalize(newTri.ID);
                    LegalizeFrontier(changedVerts);

                    vecL = fScan.Left.Vert.Pos - fScan.Vert.Pos;
                    vecR = fScan.Right.Vert.Pos - fScan.Vert.Pos;
                }

                hullIdxs.Add(fScan.VertIdx);
                fScan = fScan.Right;
                if (!firstScanDone) firstScanDone = true;
            }

            D.HullIdxs = hullIdxs.ToArray();
        }

        private PolygonGraph.Vertex[] GetVerts(int[] _vertIdxs)
        {
            return _vertIdxs.Select(_idx => Vertices[_idx]).ToArray();
        }

        private void Init()
        {
            Origin = CalcOrigin();
            m_PolPos = Points.Select(_pt => new PolartPt(_pt, Origin, this)).ToArray();

            m_VertIdxsByR = new int[Vertices.Length];
            var tempVertIdxs = new List<int>();
            for (var idx = 0; idx < Vertices.Length; ++idx)
                tempVertIdxs.Add(idx);

            m_VertIdxsByR = tempVertIdxs.OrderBy(_idx => m_PolPos[_idx].r).ToArray();

            //Init Frontier
            var triVertIdxs = new[] {m_VertIdxsByR[0], m_VertIdxsByR[1], m_VertIdxsByR[2]};

            var firstTri = new Delaunay.Triangle(triVertIdxs, D);

            frontier = new Frontier(firstTri, this);
        }

        private PolartPt PolPos(int _vertIdx)
        {
            return m_PolPos[_vertIdx];
        }

        private PolartPt PolPos(PolygonGraph.Vertex _vert)
        {
            return m_PolPos[_vert.Idx];
        }

        private void Walk(RL _dir, int _viIdx)
        {
            var fvi = frontier.GetHaving(_viIdx);

            while (true)
            {
                var fvn = fvi[_dir];

                var vecL = fvn[RL.Left].Vert.Pos - fvn.Vert.Pos;
                var vecR = fvn[RL.Right].Vert.Pos - fvn.Vert.Pos;

                var angleCW = vecL.AngleCW(vecR);
                if (angleCW > Math.PI / 2f) break;

                var twinLt = fvn[RL.Left].EdgeRight;
                var twinRt = fvn.EdgeRight;
                var newTri = D.AddTriToMesh(twinLt, twinRt);
                if (newTri == null) break;
                frontier.Remove(fvn);
                var changedVerts = D.Legalize(newTri.ID);
                LegalizeFrontier(changedVerts);
            }
        }

        //TODO PRIVATE?
        public class Frontier : ICircleSweepObj
        {
            private readonly FPList m_FPList;

            public Frontier(Delaunay.Triangle _firstTri, CircleSweep _cs)
            {
                CS = _cs;
                m_FPList = new FPList(_cs);

                var verts = _firstTri.VertIdxs.Select(_id => CS.Vertices[_id]).ToArray();

                var ftPts = _firstTri.Verts
                    .Select(_vert => new FrontierPt(_vert.Idx, _firstTri.EdgeWithOrigin(_vert.Idx), CS)).ToArray();

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

            public CircleSweep CS { get; }

            public FrontierPt Add(PolygonGraph.Vertex _newVert, Delaunay.Triangle _newTri, FrontierPt _fLt,
                FrontierPt _fRt)
            {
                var newEdge = _newTri.EdgeWithOrigin(_newVert.Idx);
                var fNew = new FrontierPt(_newVert.Idx, newEdge, CS);

                _fLt.Right = _fRt.Left = fNew;
                _fLt.EdgeRight = _newTri.EdgeWithOrigin(_fLt.VertIdx);

                fNew.Left = _fLt;
                fNew.Right = _fRt;

                m_FPList.Add(fNew); //TODO add epsilon offset for dupe?
                return fNew;
            }

            public bool ContainsVert(int _vertIdx)
            {
                return m_FPList.ContainsVertIdx(_vertIdx);
            }

            public FrontierPt GetHaving(int _vertIdx)
            {
                return m_FPList.GetWithVertIdx(_vertIdx);
            }

            public FrontierPt[] Project(float _theta)
            {
                return m_FPList.Project(_theta);
            }

            public void Remove(FrontierPt _fPt)
            {
                var fpLeft = _fPt.Left;
                var fpRight = _fPt.Right;
                fpLeft.Right = fpRight;
                fpRight.Left = fpLeft;

                m_FPList.Remove(_fPt);

                var commonTris = CS.PolysContainingVert[fpLeft.VertIdx]
                    .Intersect(CS.PolysContainingVert[fpRight.VertIdx]).ToArray();
                if (commonTris.Length != 1)
                {
                    var tris = commonTris.Select(_id => CS.Tri(_id)).ToArray();

                    throw new Exception("WTF?" + tris); //TODO DEBUG
                }

                fpLeft.EdgeRight = CS.Tri(commonTris[0]).EdgeWithOrigin(fpLeft.VertIdx);
                _fPt.Dispose();
            }

            /*
            public Guid DebugGetTriangleAtFrontierRt(FrontierPt _fPt)
            {
                var trisLt = CS.PolysContainingVert[_fPt.VertIdx].Select(_id => CS.Tri(_id)).ToArray();
                var trisRt = CS.PolysContainingVert[_fPt.Right.VertIdx].Select(_id => CS.Tri(_id)).ToArray();
                var common = trisLt.Intersect(trisRt);
                var commonTris = CS.PolysContainingVert[_fPt.VertIdx].Intersect(CS.PolysContainingVert[_fPt.Right.VertIdx]).ToArray();
                if (commonTris.Length != 1)
                {
                    var tris = commonTris.Select(_id => CS.Tri(_id)).ToArray();

                    throw new Exception("WTF?" + tris.ToString()); //TODO DEBUG
                }

                return commonTris[0];
            }
            */
            public class FrontierPt : ICircleSweepObj
            {
                public FrontierPt(int vertIdx, PolygonGraph.Poly.HalfEdge _edgeRight, CircleSweep _cs)
                {
                    VertIdx = vertIdx;
                    EdgeRight = _edgeRight;
                    CS = _cs;
                }

                public readonly int VertIdx;

                public int EdgeRightIdx { get; private set; }

                public FrontierPt Left;
                public FrontierPt Right;
                public FrontierPt this[RL _dir] => _dir == RL.Left ? Left : Right;
                public Guid TriRightID { get; private set; }

                public PolygonGraph.Vertex Vert => CS.Vertices[VertIdx];

                public CircleSweep CS { get; }

                public PolygonGraph.Poly.HalfEdge EdgeRight
                {
                    get { return CS.Tri(TriRightID).Edge(EdgeRightIdx); }
                    set
                    {
                        TriRightID = value.PolyID;
                        EdgeRightIdx = value.Poly.EdgeWithOrigin(value.OriginIdx).EdgeIdx;
                    }
                }

                public void Dispose()
                {
                    Left = null;
                    Right = null;
                }

                public override string ToString()
                {
                    var v = (Vert ?? (object) "").ToString();
                    var er = (EdgeRight ?? (object) "").ToString();
                    return "Fnt Pt v: " + v + " RtV: " + Right.Vert.Idx + " LtV: " + Left.Vert.Idx + " EdgeRt: " + er;
                }
            }

            private class FPList : ICircleSweepObj
            {
                public FPList(CircleSweep _cs)
                {
                    CS = _cs;
                    m_FPts = new SortedList<float, SortedList<float, FrontierPt>>();
                    m_FptByVertIdx = new Dictionary<int, FrontierPt>();
                }

                public CircleSweep CS { get; }

                private readonly Dictionary<int, FrontierPt> m_FptByVertIdx;
                private readonly SortedList<float, SortedList<float, FrontierPt>> m_FPts;

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
            }
        }

        public static class Settings
        {
            /// <summary>
            ///     Force outer hull to be convex.  If true, overrides MaxHullAngleDegrees
            /// </summary>
            public static bool ForceConvexHull = false;

            /// <summary>
            ///     When finishing hull, sets the max angle at which a tri will be added.
            /// </summary>
            public static float MaxHullAngleDegrees = 135f;
        }

        private class PolartPt : ICircleSweepObj
        {
            public CircleSweep CS { get; }

            public readonly float r;
            public readonly float Theta;

            public PolartPt(float _radius, float _theta, CircleSweep _cs)
            {
                r = _radius;
                Theta = _theta;
                CS = _cs;
            }

            public PolartPt(Vector2f _cPt, Vector2f _origin, CircleSweep _cs)
            {
                CS = _cs;
                var nx = _cPt.x - _origin.x;
                var ny = _cPt.y - _origin.y;

                r = (float) Math.Sqrt(nx * nx + ny * ny);
                Theta = AngleFromOrigin(_origin, _cPt);
            }
        }
    }
}