using System.Collections.Generic;

namespace ioDelaunay
{
    public class Voronoi2
    {
        public Delaunay D;
        public Rect Bounds;
        public Rect? TrimBounds = null;
        

        public Dictionary<int, Site> SitesByDIdx = new Dictionary<int, Site>();
        public Site LastTrimSite = null;
        
        public Voronoi2(Delaunay _d)
        {
            D = _d;
            Bounds = _d.BoundsRect;
        }

        private void BuildInner()
        {
            var tri = D.LastTri;
            var dIdxsDone = new HashSet<int>();

            //Build Inner Sites
            while (tri.PrevTri != null)
            {
                var edges = new[] {tri.Edge0, tri.Edge1, tri.Edge2};
                for (int eIdx = 0; eIdx < 3; ++eIdx)
                {
                    var edge = edges[eIdx];
                    var edgeOIdx = edge.OriginIdx;
                    if (dIdxsDone.Contains(edgeOIdx)) continue;
                    dIdxsDone.Add(edgeOIdx);
                    if (D.HullIdxs.Contains(edgeOIdx)) continue;

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
                        scanEdge = scanEdge.m_Twin;
                        
                    } while (scanEdge != edge);


                    var site = new Site(ptsCW, twins, true);
                    SitesByDIdx.Add(edgeOIdx, site);
                }

                tri = tri.PrevTri;
            }
        }

        private void BuildOuter()
        {
            var infEdgeLen = (D.BoundsRect.width + D.BoundsRect.height) / 4f;  //TODO
            var hullEdges = D.HullEdges;
            Site.HalfEdge lastTwin = null;
            for (int heIdx = 0; heIdx < hullEdges.Count; ++heIdx)
            {
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
                    if (scanEdge.m_Twin == null) break;
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
                    scanEdge = scanEdge.m_Twin;
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
                
                var site = new Site(ptsCW, twins, false);
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
            BuildInner();
            BuildOuter();
            if(TrimBounds != null)
                TrimToBoundary();
                
        }

        void TrimToBoundary()
        {
            var trimBnds = TrimBounds.Value;
            var hullIdx = 0;
            
            //Find Site that needs trimming
            var done = false;
            var siteScan = SitesByDIdx[D.HullIdxs[hullIdx]];
            while (!done && hullIdx < D.HullIdxs.Count)
            {
                var edgeScan = siteScan.FirstEdge;
                while (edgeScan != null)
                {
                    if (!trimBnds.Contains(edgeScan.Origin))
                    {
                        done = true;
                        break;
                    }
                    edgeScan = edgeScan.NextEdge;
                }
                siteScan = SitesByDIdx[D.HullIdxs[++hullIdx]];
            }

            if (hullIdx == D.HullIdxs.Count) return; //Nothing to trim?
            
            


        }
        
        private Vector2 GetIntersectionToBndy(Vector2 _fromPt, Vector2 _fromDir, Rect _bnd)
        {
            BndSide unused;
            return GetIntersectionToBndy(_fromPt, _fromDir, _bnd, out unused);
        }
        
        //TODO INEFFICIENT?
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
            public Voronoi2 V;
            public HalfEdge FirstEdge;
            public readonly bool Closed;
            
            public List<HalfEdge> Edges
            {
                get
                {
                    var eList = new List<HalfEdge> {FirstEdge};
                    var eScan = FirstEdge.NextEdge;
                    while(eScan != FirstEdge && eScan != null)
                    {
                        eList.Add(eScan);
                        eScan = eScan.NextEdge;
                    }

                    return eList;
                }
            }

            public Site(List<Vector2> _ptsCW, List<HalfEdge> _twins, bool _closed)
            {
                Closed = _closed;
                var prevEdge = FirstEdge = new HalfEdge(_ptsCW[0]);
                var prevTwin = _twins[0];
                if (prevTwin != null)
                {
                    prevEdge.Twin = prevTwin;
                    prevTwin.Twin = prevEdge;
                }

                HalfEdge edge = null;
                HalfEdge twin = null;
                for (int eIdx = 1; eIdx < _ptsCW.Count; ++eIdx)
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

                if(Closed)
                    edge.NextEdge = FirstEdge;

            }
            
            public class HalfEdge
            {
                public Vector2 Origin;
                public HalfEdge NextEdge;
                public Site Site;
                public HalfEdge Twin;

                public HalfEdge(Vector2 _origin)
                {
                    Origin = _origin;
                }
            }
        }

        
    }
}