using System.Collections.Generic;
using System.Linq;
using ioSS.Util.Maths;
using ioSS.Util.Maths.Geometry;

namespace ioSS.Delaunay
{
    public class Voronoi
    {
        public Rect Bounds;
        public Delaunay D;

        //public Settings settings = new Settings();


        public Dictionary<int, Site> SitesByDIdx = new Dictionary<int, Site>();

        public Voronoi(Delaunay _d)
        {
            D = _d;
            Bounds = _d.BoundsRect;
        }

        private void BuildInner()
        {
            var tri = D.LastTri;
            var dIdxsDone = new HashSet<int>();

            var hullIdxs = new HashSet<int>(D.HullEdges.Select(_edge => _edge.OriginIdx));

            var progState = "Voronoi Build Inner Sites...";
            var prog = 0f;
            var trisTotal = D.Triangles().Length; //TODO not fast
            var triProgCnt = 0;

            //Build Inner Sites
            while (tri.PrevTri != null)
            {
                prog = (float) triProgCnt++ / trisTotal;
                progState = "Voronoi Inner Tri " + triProgCnt + " of " + trisTotal;
                D.Prog.Update(prog, progState);

                var edges = new[] {tri.Edge0, tri.Edge1, tri.Edge2};
                for (var eIdx = 0; eIdx < 3; ++eIdx)
                {
                    var edge = edges[eIdx];
                    var edgeOIdx = edge.OriginIdx;
                    if (dIdxsDone.Contains(edgeOIdx)) continue;
                    dIdxsDone.Add(edgeOIdx);
                    if (hullIdxs.Contains(edgeOIdx)) continue;

                    var ptsCW = new List<Vector2>();
                    var twins = new List<Site.HalfEdge>();
                    var scanEdge = edge;
                    do
                    {
                        var cc = new Vector2(scanEdge.Triangle.CCX, scanEdge.Triangle.CCY);
                        ptsCW.Add(cc);
                        Bounds.Encapsulate(cc);
                        //Get and set twin
                        scanEdge = scanEdge.NextEdge.NextEdge;
                        Site.HalfEdge nbrScan = null;
                        if (SitesByDIdx.ContainsKey(scanEdge.OriginIdx))
                        {
                            var nbrSite = SitesByDIdx[scanEdge.OriginIdx];
                            //Get nbr Origin match
                            nbrScan = nbrSite.FirstEdge;
                            while (nbrScan.NextEdge.Origin != cc)
                                nbrScan = nbrScan.NextEdge;
                        }

                        twins.Add(nbrScan);
                        scanEdge = scanEdge.Twin;
                    } while (scanEdge != edge);


                    var site = new Site(ptsCW, twins, true);
                    SitesByDIdx.Add(edgeOIdx, site);
                }

                tri = tri.PrevTri;
            }
        }

        private void BuildOuter()
        {
            var progState = "Voronoi Build Outer...";
            var prog = 0f;
            var infEdgeLen = (D.BoundsRect.width + D.BoundsRect.height) / 4f; //TODO
            var hullEdges = D.HullEdges;
            Site.HalfEdge lastTwin = null;
            for (var heIdx = 0; heIdx < hullEdges.Count; ++heIdx)
            {
                prog = (float) heIdx / hullEdges.Count;
                D.Prog.Update(prog, progState);

                var edge = hullEdges[heIdx];
                var nextEdgeOriginPos = edge.NextEdge.OriginPos;
                var edgeOIdx = edge.OriginIdx;

                var ptsCW = new List<Vector2>();
                var twins = new List<Site.HalfEdge> {null};
                var scanEdge = edge;

                //Project first edge perpendicular to hull edge
                var perpHullVec = GetPerpendicularLeft(edge.OriginPos, nextEdgeOriginPos).normalized;
                perpHullVec *= infEdgeLen;

                var cc1 = new Vector2(scanEdge.Triangle.CCX, scanEdge.Triangle.CCY);
                var firstEdgePos = cc1 + perpHullVec;
                Bounds.Encapsulate(firstEdgePos);
                ptsCW.Add(firstEdgePos);

                //Build middle edges

                while (true)
                {
                    var cc = new Vector2(scanEdge.Triangle.CCX, scanEdge.Triangle.CCY);
                    ptsCW.Add(cc);
                    Bounds.Encapsulate(cc);
                    //Get and set twin
                    scanEdge = scanEdge.NextEdge.NextEdge;
                    if (scanEdge.Twin == null) break;
                    Site.HalfEdge nbrScan = null;
                    if (SitesByDIdx.ContainsKey(scanEdge.OriginIdx))
                    {
                        var nbrSite = SitesByDIdx[scanEdge.OriginIdx];
                        //Get nbr Origin match
                        nbrScan = nbrSite.FirstEdge;
                        while (nbrScan.NextEdge.Origin != cc)
                            nbrScan = nbrScan.NextEdge;
                    }

                    twins.Add(nbrScan);
                    scanEdge = scanEdge.Twin;
                }

                //Add last twin
                twins.Add(lastTwin);

                //Project last edge perpendicular to hull edge
                perpHullVec = GetPerpendicularLeft(scanEdge.OriginPos, scanEdge.NextEdge.OriginPos).normalized;
                perpHullVec *= infEdgeLen;

                var ccLast = new Vector2(scanEdge.Triangle.CCX, scanEdge.Triangle.CCY);
                var lastEdgePos = ccLast + perpHullVec;
                Bounds.Encapsulate(lastEdgePos);
                ptsCW.Add(lastEdgePos);
                twins.Add(null);

                var site = new Site(ptsCW, twins, false); //TODO store dIDX?
                lastTwin = twins[0];
                SitesByDIdx.Add(edgeOIdx, site);
            }
        }

        private Vector2 GetPerpendicularLeft(Vector2 _a, Vector2 _b) //TODO remove
        {
            var hullVec = _b - _a;
            return new Vector2(-hullVec.y, hullVec.x);
        }

        public void Build()
        {
            SitesByDIdx.Clear();
            BuildInner();
            BuildOuter();
        }

        public void TrimSitesToBndry(Rect _bndry, bool _closeAtTrim = true)
        {
            var trimBnds = _bndry;

            //Scan for sites needing trim and for removal
            var sitesToTrim = new HashSet<int>();
            var sitesToRemove = new HashSet<int>();

            //Progress
            var idxCnt = 0;
            var idxCntTot = SitesByDIdx.Keys.Count;
            var progState = "Voronoi Trimming Sites - Scanning";
            var prog = 0f;

            foreach (var dIdx in SitesByDIdx.Keys)
            {
                prog = (float) idxCnt++ / idxCntTot;
                progState = "Voronoi Trimming Sites - Scanning " + idxCnt + " of " + idxCntTot;
                D.Prog.Update(prog, progState);

                var site = SitesByDIdx[dIdx];
                var wasClosed = site.Closed;
                site.Closed = true;

                var scanEdge = site.FirstEdge;
                var hasIn = false;
                var hasOut = false;
                do
                {
                    if (trimBnds.Contains(scanEdge.Origin))
                        hasIn = true;
                    else
                        hasOut = true;
                    scanEdge = scanEdge.NextEdge;
                } while (scanEdge != site.FirstEdge);

                if (hasIn && hasOut)
                    sitesToTrim.Add(dIdx);
                else if (hasOut && !hasIn)
                    sitesToRemove.Add(dIdx);

                site.Closed = wasClosed;
            }

            var bndyOrigin = new Dictionary<BndSide, Vector2>
            {
                {BndSide.Up, trimBnds.min},
                {BndSide.Right, new Vector2(trimBnds.xMin, trimBnds.yMax)},
                {BndSide.Down, trimBnds.max},
                {BndSide.Left, new Vector2(trimBnds.xMax, trimBnds.yMin)}
            };

            //Trim Sites

            //Progress
            var idxTrimCur = 0;
            var idxTrimTot = sitesToTrim.Count;

            foreach (var dIdx in sitesToTrim)
            {
                prog = (float) idxTrimCur / idxTrimTot;
                progState = "Voronoi Trimming Sites - Trimming " + idxTrimCur + " of " + idxTrimTot;
                D.Prog.Update(prog, progState);

                var site = SitesByDIdx[dIdx];
                site.Closed = true;

                var scanEdge = site.FirstEdge;
                while (!trimBnds.Contains(scanEdge.Origin))
                    scanEdge = scanEdge.NextEdge;

                //Find exit bnds intersection
                while (trimBnds.Contains(scanEdge.NextEdge.Origin))
                    scanEdge = scanEdge.NextEdge;

                var exitEdge = scanEdge;
                var intFrom = exitEdge.Origin;
                var intVec = exitEdge.NextEdge.Origin - intFrom;
                BndSide exitSide;

                var exitIntPt = GetIntersectionToBndy(intFrom, intVec, trimBnds, out exitSide);

                //Find enter bnds interection
                while (!trimBnds.Contains(scanEdge.NextEdge.Origin))
                    scanEdge = scanEdge.NextEdge;

                var enterEdge = scanEdge;
                intFrom = enterEdge.NextEdge.Origin;
                intVec = enterEdge.Origin - intFrom;
                BndSide enterSide;

                var enterIntPt = GetIntersectionToBndy(intFrom, intVec, trimBnds, out enterSide);

                if (exitSide == enterSide || !_closeAtTrim)
                {
                    var newBndEdge = new Site.HalfEdge(exitIntPt);
                    exitEdge.NextEdge = newBndEdge;
                    newBndEdge.NextEdge = enterEdge;
                    enterEdge.Origin = enterIntPt;
                    site.FirstEdge = enterEdge;
                }
                else
                {
                    var newBndEdgeA = new Site.HalfEdge(exitIntPt);
                    var newBndEdgeB = new Site.HalfEdge(bndyOrigin[enterSide]);
                    exitEdge.NextEdge = newBndEdgeA;
                    newBndEdgeA.NextEdge = newBndEdgeB;
                    enterEdge.Origin = enterIntPt;
                    newBndEdgeB.NextEdge = enterEdge;
                    site.FirstEdge = enterEdge;
                }


                site.Closed = _closeAtTrim;
            }

            //Clean up sites outside of boundary
            foreach (var dIdx in sitesToRemove) //TODO need to do more than remove from sites list? (Memory)
            {
                var site = SitesByDIdx[dIdx];

                foreach (var edge in site.Edges)
                {
                    edge.NextEdge = null;
                    edge.Twin = null;
                }

                site.FirstEdge = null;

                SitesByDIdx.Remove(dIdx);
            }
        }

        public void LloydRelax(Rect _trimBndy, int _iters = 1)
        {
            var progState = "Voronoi Lloyd Relax";
            var prog = 0f;
            var idxSiteCur = 0;
            var idxSiteTot = SitesByDIdx.Keys.Count;

            for (var iter = 0; iter < _iters; ++iter)
            {
                var centroids = new List<Vector2>();
                foreach (var siteIdx in SitesByDIdx.Keys)
                {
                    prog = (float) idxSiteCur++ / idxSiteTot;
                    progState = "Voronoi Lloyd Relax " + idxSiteCur + " of " + idxSiteTot;
                    D.Prog.Update(prog, progState);

                    var site = SitesByDIdx[siteIdx];
                    var centroid = Geom.CentroidOfPoly(site.Edges.Select(_edge => _edge.Origin));
                    centroids.Add(centroid);
                }

                D.Triangulate(centroids);
                Build();
                TrimSitesToBndry(_trimBndy);
            }
        }

        //TODO INEFFICIENT
        private Vector2 GetIntersectionToBndy(Vector2 _fromPt, Vector2 _fromDir, Rect _bnd,
            out BndSide _bndSide)
        {
            _bndSide = BndSide.Invalid;
            var vBndCorners = new[]
            {
                _bnd.min,
                new Vector2(_bnd.xMin, _bnd.yMax),
                _bnd.max,
                new Vector2(_bnd.xMax, _bnd.yMin)
            };

            var vBndVecs = new[]
            {
                vBndCorners[1] - vBndCorners[0], //Up
                vBndCorners[2] - vBndCorners[1], //Right
                vBndCorners[3] - vBndCorners[2], //Down
                vBndCorners[0] - vBndCorners[3] //Left
            };
            var minDist = float.PositiveInfinity;
            var intPt = Vector2.positiveInfinity;
            for (var bEdgeIdx = 0; bEdgeIdx < 4; ++bEdgeIdx) //TODO this is inefficient
            {
                var intsct = Intersect(_fromDir, vBndVecs[bEdgeIdx], _fromPt, vBndCorners[bEdgeIdx]);
                //if (intsct.sqrMagnitude == float.PositiveInfinity) continue;
                var distSqr = (intsct - _fromPt).sqrMagnitude;
                if (distSqr < minDist)
                {
                    minDist = distSqr;
                    intPt = intsct;
                    _bndSide = (BndSide) bEdgeIdx;
                }
            }

            return intPt;
        }

        private static Vector2 Intersect(Vector2 _rayA, Vector2 _rayB, Vector2 _originA, Vector2 _originB)
        {
            var dx = _originB.x - _originA.x;
            var dy = _originB.y - _originA.y;
            var det = _rayB.x * _rayA.y - _rayB.y * _rayA.x;
            if (det == 0)
                return Vector2.positiveInfinity;
            var u = (dy * _rayB.x - dx * _rayB.y) / det;
            var v = (dy * _rayA.x - dx * _rayA.y) / det;
            if (u < 0 || v < 0)
                return Vector2.positiveInfinity;
            return new Vector2(_originA.x + _rayA.x * u, _originA.y + _rayA.y * u);
        }

        private enum BndSide
        {
            Invalid = -1,
            Up = 0,
            Right = 1,
            Down = 2,
            Left = 3
        }


        public class Site
        {
            public HalfEdge FirstEdge;

            private bool m_Closed;

            public Site(List<Vector2> _ptsCW, List<HalfEdge> _twins, bool _closed)
            {
                m_Closed = _closed;
                var prevEdge = FirstEdge = new HalfEdge(_ptsCW[0]);
                var prevTwin = _twins[0];
                if (prevTwin != null)
                {
                    prevEdge.Twin = prevTwin;
                    prevTwin.Twin = prevEdge;
                }

                HalfEdge edge = null;
                HalfEdge twin = null;
                for (var eIdx = 1; eIdx < _ptsCW.Count; ++eIdx)
                {
                    edge = new HalfEdge(_ptsCW[eIdx]);
                    twin = _twins[eIdx];
                    if (twin != null)
                    {
                        edge.Twin = twin;
                        twin.Twin = edge;
                    }

                    prevEdge.NextEdge = edge;
                    prevEdge = edge;
                }

                if (m_Closed)
                    edge.NextEdge = FirstEdge;
            }

            public bool Closed
            {
                get => m_Closed;
                set
                {
                    if (m_Closed == value) return;
                    //Find last edge
                    var edgeScan = FirstEdge;
                    var target = m_Closed ? FirstEdge : null;
                    while (edgeScan.NextEdge != target)
                        edgeScan = edgeScan.NextEdge;

                    edgeScan.NextEdge = m_Closed ? null : FirstEdge;

                    m_Closed = value;
                }
            }


            public List<HalfEdge> Edges
            {
                get
                {
                    var eList = new List<HalfEdge> {FirstEdge};
                    var eScan = FirstEdge.NextEdge;
                    while (eScan != FirstEdge && eScan != null)
                    {
                        eList.Add(eScan);
                        eScan = eScan.NextEdge;
                    }

                    return eList;
                }
            }

            public class HalfEdge
            {
                public HalfEdge NextEdge;
                public Vector2 Origin;
                public HalfEdge Twin;

                public HalfEdge(Vector2 _origin)
                {
                    Origin = _origin;
                }
            }
        }

        public class Settings
        {
            /// <summary>
            ///     Close outer voronoi sites?
            /// </summary>
            public bool CloseOuterSites = true;

            private bool m_AddBoundaryCorners = true;


            /// <summary>
            ///     Add corners to voronoi sites at boundary corners?
            ///     Ignored if CloseOuterSites is false.
            /// </summary>
            public bool AddBoundryCorners
            {
                get => CloseOuterSites && m_AddBoundaryCorners;

                set => m_AddBoundaryCorners = value;
            }
        }
    }
}