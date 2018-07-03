using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Security.Cryptography;
using ioPolygonGraph;
using Vectorf;

namespace ioDelaunay
{
    public partial class Delaunay
    {
        public class Voronoi : PolygonGraph, IDelaunayObj
        {
            public class Settings
            {
                /// <summary>
                /// Close outer voronoi sites?
                /// </summary>
                public bool CloseOuterSites = true;
                
                /// <summary>
                /// Percent beyond Delaunay bounds at which trim will occur if trim is enabled.
                /// Must be greater than 1.0
                /// </summary>
                public float BoundaryExpansionPct
                {
                    get { return m_BoundaryExpansionPct; }
                    set
                    {
                        var pct = value;
                        m_BoundaryExpansionPct = pct < 1.0 ? 1.0f : pct;
                    }
                }

                private float m_BoundaryExpansionPct = 1.2f;
                
                /// <summary>
                /// Trim sites at some pct away from delaunay boundary extents.
                /// Note that there is an increased chance of voronoi site edge intersection at
                /// distance based on Delaunay hull angle limit.
                /// </summary>
                public bool TrimSitesAtBoundary = true;
                private bool m_AddBoundaryCorners = true;

                /// <summary>
                /// Add corners to voronoi sites at boundary corners?
                /// Ignored if Trim is false or if CloseOuterSites is false.
                /// </summary>
                public bool AddBoundryCorners
                {
                    get
                    {
                        return TrimSitesAtBoundary && CloseOuterSites && m_AddBoundaryCorners;
                    }

                    set { m_AddBoundaryCorners = value; }
                }
            }

            public Settings settings;
            
            private readonly Dictionary<int, Guid> m_TriIDByCentIdx;
            private readonly Dictionary<Guid, int> m_CentIdxByTriID; //Delaunay triangle centers;
            private readonly Dictionary<int, Guid> m_SiteIDByDVertIdx;
            
            public Site[] Sites => m_Polys.Values.Cast<Site>().ToArray();

            public Voronoi(Delaunay _d, Settings _settings = null) : base()
            {
                settings = _settings;
                D = _d;

                m_CentIdxByTriID = new Dictionary<Guid, int>();
                m_TriIDByCentIdx = new Dictionary<int, Guid>();
                m_SiteIDByDVertIdx = new Dictionary<int, Guid>();

                Init();

            }

            private void Init()
            {
                m_CentIdxByTriID.Clear();
                m_TriIDByCentIdx.Clear();
                m_SiteIDByDVertIdx.Clear();
                Points.Clear();
                m_Polys.Clear();
                m_PolysContainingVert.Clear();
                m_BoundsRect = Rectf.zero;
                
                var delTris = D.Triangles;
                var centers = new List<Vector2f>();

                for (int tIdx = 0; tIdx < delTris.Length; ++tIdx)
                {
                    var tri = delTris[tIdx];
                    float r;
                    Vector2f center;
                    tri.CircumCircle(out center, out r);
                    centers.Add(center);
                    m_CentIdxByTriID.Add(tri.ID, tIdx);
                    m_TriIDByCentIdx.Add(tIdx, tri.ID);
                }

                AddVertices(centers);
            }
            

            public Delaunay D { get; }


            private Rectf m_DBounds = Rectf.zero;
            
            public Rectf DBounds
            {
                get
                {
                    if (m_DBounds == Rectf.zero)
                    {
                        var scale = settings.BoundaryExpansionPct < 1f ? 1f : settings.BoundaryExpansionPct;
                        m_DBounds = new Rectf(D.BoundsRect);
                        var center = m_DBounds.center;
                        m_DBounds.height = m_DBounds.height * scale;
                        m_DBounds.width = m_DBounds.width * scale;
                        m_DBounds.center = center;
                    }

                    return m_DBounds;
                }
            }
            public void BuildSites()
            {
                
                BuildSites(DBounds);
            }
            
            private void BuildSites(Rectf _bounds)
            {
                var sIDsOutBnds = new List<Guid>();
                
                
                //Inner sites
                for(int delIdx = 0; delIdx < D.Points.Count; ++delIdx)
                {
                    if (D.HullIdxs.Contains(delIdx)) continue;
                    var centers = GetCenterIdxsAtSite(delIdx);
                    var centersCW = SortCW(delIdx, centers.ToArray());

                    var site = new Site(centersCW, delIdx, true, this);
                    foreach (var vIdx in site.VertIdxs)
                    {
                        if (!_bounds.Contains(Points[vIdx]))
                        {
                            sIDsOutBnds.Add(site.ID);
                            break;
                        }
                    }
                }
                
                Site prevSite = null;
                int firstSiteBackVertIdx = -1;
                var infiniteEdgeLen = (m_BoundsRect.width + m_BoundsRect.height) / 4f;
                for (int idx = 0; idx < D.HullIdxs.Length; ++idx)
                {
                    var hIdxPrev = D.HullIdxs[idx == 0 ? D.HullIdxs.Length - 1 : idx - 1];
                    var hIdxCur = D.HullIdxs[idx];
                    var hIdxNext = D.HullIdxs[idx == D.HullIdxs.Length - 1 ? 0 : idx + 1];

                    var centers = GetCenterIdxsAtSite(hIdxCur);
                    
                    //Get ref vec for CW sort perp to neighbor hull site vec
                    var vhprev = D.Points[hIdxPrev];
                    var vhnext = D.Points[hIdxNext];
                    var nbrHullVec = vhnext - vhprev;
                    var nbrPerpVec = new Vector2f(-nbrHullVec.y, nbrHullVec.x);
                    
                    var refVec = nbrPerpVec.normalized;
                    var centersCW = SortCW(hIdxCur, centers.ToArray(), refVec).ToList();
                    
                    if (idx == D.HullIdxs.Length - 1) //Handle last site
                    {
                        centersCW.Insert(0, firstSiteBackVertIdx);
                        centersCW.Add(prevSite.Edge(0).OriginIdx);
                    }
                    else
                    {
                        //Get forward perp line
                        var hvp0 = D.Points[hIdxCur];
                        var hvp1 = D.Points[hIdxNext];
                        var hullVec = (hvp1 - hvp0).normalized;
                        var perpVec = new Vector2f(-hullVec.y, hullVec.x);
                    
                        //Hull Pt
                        var outFromPt = Points[centersCW[0]];

                        AddVertex(outFromPt + perpVec * infiniteEdgeLen);
                        centersCW.Insert(0, Points.Count - 1);
                    
                    
                        if (prevSite == null)
                        {
                            var prevHIdx = D.HullIdxs[D.HullIdxs.Length - 1];
                            hvp0 = D.Points[prevHIdx];
                            hvp1 = D.Points[hIdxCur];
                            hullVec = (hvp1 - hvp0).normalized;
                            perpVec = new Vector2f(-hullVec.y, hullVec.x);
                        
                            //Back HullPt
                            var outFromPtBack = Points[centersCW[centersCW.Count - 1]];
                        
                            AddVertex(outFromPtBack + perpVec * infiniteEdgeLen);
                        
                            firstSiteBackVertIdx = Points.Count - 1;
                            centersCW.Add(firstSiteBackVertIdx);
                        }
                        else
                        {
                            centersCW.Add(prevSite.Edge(0).OriginIdx);
                        }
                    }
                    
                    prevSite = new Site(centersCW.ToArray(), hIdxCur, settings.CloseOuterSites, this);
                    
                    //Record sites having verts outside the boundary
                    foreach (var vIdx in prevSite.VertIdxs)
                    {
                        if (_bounds.Contains(Points[vIdx])) continue;
                        sIDsOutBnds.Add(prevSite.ID);
                        break;

                    }
                }
                
                /*                
                {
                    //TODO DEBUG
                    DebugVisualizer.Visualize(D, this, "DelVorNoTrim");
                }
                */

                if(settings.TrimSitesAtBoundary)
                    TrimSitesToBndry(sIDsOutBnds, _bounds);
            }

            
            private void TrimSitesToBndry(List<Guid> _siteIDs, Rectf _bnd)
            {
                var vertIdxsToRemove = new HashSet<int>();
                Guid lastSiteID = Guid.Empty;
                int firstInVertIdx = -1;
                var prevOutVertIdx = -1;
                Site prevSite = null;
                var curSite = (Site) m_Polys[_siteIDs[0]];
                BndSide curBndy = BndSide.Invalid;
                BndSide firstBndy = BndSide.Invalid;
                var bndyOrigin = new Dictionary<BndSide, Vector2f>
                {
                    {BndSide.Up, _bnd.min},
                    {BndSide.Right, new Vector2f(_bnd.xMin, _bnd.yMax)},
                    {BndSide.Down, _bnd.max},
                    {BndSide.Left, new Vector2f(_bnd.xMax, _bnd.yMin)}
                };
                
                while(true)
                {
                    curSite.Closed = true;

                    //Get in bndy Transition for SiteA and capture verts to keep and twins
                    var edgesA = curSite.Edges;
                    var edgeIn = edgesA[0];
                    var newVertsCW = new List<int>();
                    while (_bnd.Contains(edgeIn.NextEdge.OriginPos))
                        edgeIn = edgeIn.NextEdge;
                    while (!_bnd.Contains(edgeIn.NextEdge.OriginPos))
                        edgeIn = edgeIn.NextEdge;

                    var edgeOut = edgeIn.NextEdge;
                    while (_bnd.Contains(edgeOut.NextEdge.OriginPos))
                    {
                        newVertsCW.Add(edgeOut.OriginIdx);
                        edgeOut = edgeOut.NextEdge;
                    }
                    Site nextSite = null;
                    if(curSite.ID != lastSiteID)
                        nextSite = (Site)edgeOut.Twin.Poly;

                    newVertsCW.Add(edgeOut.OriginIdx);
                        
                    //Get out bndy Transition for SiteA and capture verts to delete
                    var edgeClear = edgeOut.NextEdge;
                    vertIdxsToRemove.Add(edgeClear.OriginIdx);
                    while (!_bnd.Contains(edgeClear.NextEdge.OriginPos))
                    {
                        edgeClear = edgeClear.NextEdge;
                        vertIdxsToRemove.Add(edgeClear.OriginIdx);
                    }
                    
                    //Store first site info for last site connect.
                    if (prevSite == null)
                    {
                        
                        //Get site A in bndy intersection
                        var intPtIn = GetIntersectionToBndy(edgeIn.OriginPos, edgeIn.AsVector, _bnd, out firstBndy);
                        curBndy = firstBndy;
                        AddVertex(intPtIn);
                        prevOutVertIdx = firstInVertIdx = Points.Count - 1;
                        lastSiteID = edgeIn.Twin.PolyID;
                    }
                    
                    if (curSite.ID == lastSiteID)
                    {
                        newVertsCW.Add(firstInVertIdx);
                        newVertsCW.Insert(0, prevOutVertIdx);
                        
                        if (curBndy != firstBndy && settings.AddBoundryCorners)
                        {
                            var corner = bndyOrigin[firstBndy];
                            AddVertex(corner);
                            newVertsCW.Add(Points.Count - 1);
                        }
                        
                    }
                    else
                    {
                        //Get site out bndy intersection
                        BndSide nextBdy = curBndy;
                        var intPtOut = GetIntersectionToBndy(edgeOut.OriginPos, edgeOut.AsVector, _bnd, out nextBdy);
                        AddVertex(intPtOut);
                        newVertsCW.Add(Points.Count - 1);
                        newVertsCW.Insert(0, prevOutVertIdx);
                        prevOutVertIdx = Points.Count - 1;
                        
                        if (nextBdy != curBndy && settings.AddBoundryCorners)
                        {
                            var corner = bndyOrigin[curBndy];
                            AddVertex(corner);
                            newVertsCW.Add(Points.Count - 1);
                            curBndy = nextBdy;
                        }
                        
                    }

                    curSite.Reform(newVertsCW.ToArray());
                    curSite.Closed = settings.CloseOuterSites;
                    
                    if (curSite.ID != lastSiteID)
                    {
                        prevSite = curSite;
                        curSite = nextSite;
                        continue;
                    }
                    
                    break;

                }

                DebugVisualizer.Visualize(D,this,"VorOuter");
                CleanVerts();
                
                DebugVisualizer.Visualize(D,this,"VorPostVertRemove");
            }
            

            private Vector2f GetIntersectionToBndy(Vector2f _fromPt, Vector2f _fromDir, Rectf _bnd)
            {
                BndSide unused;
                return GetIntersectionToBndy(_fromPt, _fromDir, _bnd, out unused);
            }
            
            //TODO INEFFICIENT?
            private Vector2f GetIntersectionToBndy(Vector2f _fromPt, Vector2f _fromDir, Rectf _bnd, out BndSide _bndSide)
            {
                _bndSide = BndSide.Invalid;
                var vBndCorners = new Vector2f[]
                {
                    _bnd.min,
                    new Vector2f(_bnd.xMin, _bnd.yMax),
                    _bnd.max,
                    new Vector2f(_bnd.xMax, _bnd.yMin) 
                };

                var vBndVecs = new Vector2f[]
                {
                    vBndCorners[1] - vBndCorners[0], //Up
                    vBndCorners[2] - vBndCorners[1], //Right
                    vBndCorners[3] - vBndCorners[2], //Down
                    vBndCorners[0] - vBndCorners[3]  //Left
                };
                var minDist = float.PositiveInfinity;
                var intPt = Vector2f.positiveInfinity;
                for (int bEdgeIdx = 0; bEdgeIdx < 4; ++bEdgeIdx) //TODO this is inefficient
                {
                    var intsct = Intersect(_fromDir, vBndVecs[bEdgeIdx], _fromPt, vBndCorners[bEdgeIdx]);
                    //if (intsct.sqrMagnitude == float.PositiveInfinity) continue;
                    var distSqr = (intsct - _fromPt).sqrMagnitude;
                    if (distSqr < minDist)
                    {
                        minDist = distSqr;
                        intPt = intsct;
                        _bndSide = (BndSide)bEdgeIdx;
                    }
                }

                return intPt;
            }
            
            private enum BndSide : int { Invalid = -1, Up = 0, Right = 1, Down = 2, Left = 3}

            public void LloydRelax(int _iters = 1)
            {
                for (int iter = 0; iter < _iters; ++iter)
                {
                    var centroids = new List<Vector2f>();
                    foreach (var polykvp in m_Polys)
                    {
                        var site = (Site) polykvp.Value;
                        var centroid = Geom.CentroidOfPoly(site.VertIdxs.Select(_vIdx => Points[_vIdx]).ToArray()); //Todo make IENUM
                        centroids.Add(centroid);
    
                    }
                    //TODO DEBUG
                    D.Triangulate(centroids);
                    Init();
                    BuildSites(DBounds);
                }

            }
            
            private static Vector2f Intersect(Vector2f _rayA, Vector2f _rayB, Vector2f _originA, Vector2f _originB)
            {
                var dx = _originB.x - _originA.x;
                var dy = _originB.y - _originA.y;
                var det = _rayB.x * _rayA.y - _rayB.y * _rayA.x;
                if (det == 0) 
                    return Vector2f.positiveInfinity;
                var u = (dy * _rayB.x - dx * _rayB.y) / det;
                var v = (dy * _rayA.x - dx * _rayA.y) / det;
                if (u < 0 || v < 0) 
                    return Vector2f.positiveInfinity;
                return new Vector2f(_originA.x + _rayA.x * u, _originA.y + _rayA.y * u);
            }

            public class Site : Poly, IVoronoiObj
            {
                public readonly int VertDelIdx;
                
                public Site(int[] _vertIdxs, int _vertDelIdx, bool _closed, Voronoi _v) : base(
                    _vertIdxs, _closed, _v)
                {
                    V = _v;
                    VertDelIdx = _vertDelIdx;
                    V.m_SiteIDByDVertIdx.Add(_vertDelIdx, ID);
                }

                public Voronoi V { get; }
            }

            private interface IVoronoiObj
            {
                Voronoi V { get; }
            }

            private HashSet<int> GetCenterIdxsAtSite(int _siteVertIdx)
            {
                var triIDs = D.m_PolysContainingVert[_siteVertIdx];
                var centerIdxs = new HashSet<int>();
                foreach (var triID in triIDs)
                    centerIdxs.Add(m_CentIdxByTriID[triID]);

                return centerIdxs;
            }


            private int[] SortCW(int _siteVertIdx, int[] _vorVertIdxs)
            {
                return SortCW(_siteVertIdx, _vorVertIdxs, Vector2f.right);
            }
            
            private int[] SortCW(int _siteVertIdx, int[] _vorVertIdxs, Vector2f _refVec)
            {
                var sitePos = D.Points[_siteVertIdx];
                var centerIdxsCW = new SortedList<float, int>();
                foreach (var vorVertIdx in _vorVertIdxs)
                {
                    var triCentPos = Points[vorVertIdx];
                    var vecCent = (triCentPos - sitePos).normalized;
                    var theta = _refVec.AngleCW(vecCent);
                    centerIdxsCW.Add(theta, vorVertIdx);

                }

                return centerIdxsCW.Values.ToArray();
            }
            
        }
    }
}