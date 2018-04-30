using System;
using System.Collections.Generic;
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
                public bool CloseOuterSites = true;
                public float BoundaryExpansionPct = 1.2f;
            }

            public Settings settings;
            
            private readonly Dictionary<int, Guid> m_TriIDByCentIdx;
            private readonly Dictionary<Guid, int> m_CentIdxByTriID; //Delaunay triangle centers;
            private readonly Dictionary<int, int[]> m_HullIdxMap;
            private readonly Dictionary<int, Guid> m_SiteIDByDVertIdx;
            private readonly HashSet<Guid> m_HullSites;
            
            public Site[] Sites => m_Polys.Values.Cast<Site>().ToArray();

            private Voronoi(Vector2f[] _points, Dictionary<int, int[]> _hullIdxMap, Delaunay _d, Settings _settings = null) : base(_points)
            {
                D = _d;
                m_SiteIDByDVertIdx = new Dictionary<int, Guid>();
                m_HullSites = new HashSet<Guid>();
                m_HullIdxMap = _hullIdxMap;
                if (_settings == null)
                    _settings = new Settings();
                settings = _settings;
                m_CentIdxByTriID = new Dictionary<Guid, int>();
                m_TriIDByCentIdx = new Dictionary<int, Guid>();
                var triIDs = _d.m_Polys.Keys.ToArray();
                for (var tIdx = 0; tIdx < triIDs.Length; ++tIdx)
                {
                    m_CentIdxByTriID.Add(triIDs[tIdx], tIdx);
                    m_TriIDByCentIdx.Add(tIdx, triIDs[tIdx]);
                }
            }

            public Voronoi(Delaunay _d, Settings _settings = null) : base()
            {
                settings = _settings;
                D = _d;

                var delTris = D.Triangles;
                var centers = new List<Vector2f>();

                m_CentIdxByTriID = new Dictionary<Guid, int>();
                m_TriIDByCentIdx = new Dictionary<int, Guid>();
                m_SiteIDByDVertIdx = new Dictionary<int, Guid>();
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


            public void BuildSites2()
            {
                
                
                
                
                //Inner sites
                for(int delIdx = 0; delIdx < D.m_Vertices.Count; ++delIdx)
                {
                    if (D.HullIdxs.Contains(delIdx)) continue;
                    var centers = GetCenterIdxsAtSite(delIdx);
                    var centersCW = SortCW(delIdx, centers.ToArray());

                    new Site(centersCW, delIdx, true, this);

                    ConnectSites(delIdx);
                }

                
                
                //Hull Sites
                //Set up intersect boundary
                if (settings.BoundaryExpansionPct < 1f)
                    settings.BoundaryExpansionPct = 1f;

                var vBndLLVec = m_BoundsRect.min - m_BoundsRect.center;
                var bndCorners = new Vector2f[]
                {
                    (vBndLLVec) * settings.BoundaryExpansionPct + m_BoundsRect.center,
                    (new Vector2f(m_BoundsRect.xMin, m_BoundsRect.yMax) - m_BoundsRect.center) * settings.BoundaryExpansionPct + m_BoundsRect.center,
                    (new Vector2f(m_BoundsRect.xMax, m_BoundsRect.yMax) - m_BoundsRect.center) * settings.BoundaryExpansionPct + m_BoundsRect.center,
                    (new Vector2f(m_BoundsRect.xMax, m_BoundsRect.yMin) - m_BoundsRect.center) * settings.BoundaryExpansionPct + m_BoundsRect.center
                };

                var bndVecs = new Vector2f[]
                {
                    bndCorners[1] - bndCorners[0], //Left
                    bndCorners[2] - bndCorners[1], //Up
                    bndCorners[3] - bndCorners[2], //Right
                    bndCorners[0] - bndCorners[3]  //Down
                };
                
                Site prevSite = null;
                int firstSiteBackVertIdx = -1;
                BndSide curBndSide = BndSide.Invalid;
                
                for (int idx = 0; idx < D.HullIdxs.Length; ++idx)
                {
                    var hIdx = D.HullIdxs[idx];
                    
                    var centers = GetCenterIdxsAtSite(hIdx);
                    var refVec = (D.m_Vertices[hIdx].Pos - D.BoundsRect.center).normalized;
                    var centersCW = SortCW(hIdx, centers.ToArray(), refVec).ToList();

                    var nextHIdx = idx == D.HullIdxs.Length - 1 ? D.HullIdxs[0] : D.HullIdxs[idx + 1];
                    if (idx == D.HullIdxs.Length - 1)
                    {
                        centersCW.Insert(0, firstSiteBackVertIdx);
                        centersCW.Add(prevSite.Edge(0).OriginIdx);
                        new Site(centersCW.ToArray(), hIdx, settings.CloseOuterSites, this);
                        ConnectSites(hIdx);
                        return;
                    }
                    
                    //Get perp line
                    var hvp0 = D.m_Vertices[hIdx];
                    var hvp1 = D.m_Vertices[nextHIdx];
                    var hullVec = (hvp1.Pos - hvp0.Pos).normalized;
                    var perpVec = new Vector2f(-hullVec.y, hullVec.x);
                    
                    //Hull Pt
                    var outFromPt = m_Vertices[centersCW[0]].Pos;

                    //Get boundary intersect - TODO Inefficient
                    BndSide newBndSide = BndSide.Invalid;
                    var intPt = GetIntersectionToBndy(outFromPt, perpVec, bndVecs, bndCorners, out newBndSide);
                    


                    AddVertex(intPt);
                    centersCW.Insert(0, m_Vertices.Count - 1);
                    
                    
                    if (prevSite == null)
                    {
                        var prevHIdx = D.HullIdxs[D.HullIdxs.Length - 1];
                        hvp0 = D.m_Vertices[prevHIdx];
                        hvp1 = D.m_Vertices[hIdx];
                        hullVec = (hvp1.Pos - hvp0.Pos).normalized;
                        perpVec = new Vector2f(-hullVec.y, hullVec.x);
                        
                        //Back HullPt
                        var outFromPtBack = m_Vertices[centersCW[centersCW.Count - 1]].Pos;
                        
                        //Get bndy intersection - TODO Inefficient
                        var backBndSide = BndSide.Invalid;
                        intPt = GetIntersectionToBndy(outFromPtBack, perpVec, bndVecs, bndCorners, out backBndSide);
                        AddVertex(intPt);
                        firstSiteBackVertIdx = m_Vertices.Count - 1;
                        centersCW.Add(firstSiteBackVertIdx);
                        
                        //Check for corner
                        if (backBndSide != newBndSide && settings.CloseOuterSites)
                        {
                            AddVertex(bndCorners[(int) newBndSide]);
                            centersCW.Add(m_Vertices.Count - 1);
                        }

                        curBndSide = newBndSide;
                    }
                    else
                    {
                        
                        centersCW.Add(prevSite.Edge(0).OriginIdx);
                        //Check for corner
                        if (newBndSide != curBndSide && settings.CloseOuterSites)
                        {
                            AddVertex(bndCorners[(int) newBndSide]);
                            centersCW.Add(m_Vertices.Count - 1);
                            curBndSide = newBndSide;
                        }
                    }
                        
                    prevSite = new Site(centersCW.ToArray(), hIdx, settings.CloseOuterSites, this);
                    
                    ConnectSites(hIdx);


                }
            }

            //TODO INEFFICIENT
            private Vector2f GetIntersectionToBndy(Vector2f _fromPt, Vector2f _fromDir, 
                Vector2f[] _bndVecs, Vector2f[] _bndCorners, out BndSide _bndDir)
            {
                _bndDir = BndSide.Invalid;
                var minDist = float.PositiveInfinity;
                var intPt = Vector2f.positiveInfinity;
                for (int bEdgeIdx = 0; bEdgeIdx < 4; ++bEdgeIdx) //TODO this is inefficient
                {
                    float dist = Single.PositiveInfinity;
                    var intsct = Intersect(_fromDir, _bndVecs[bEdgeIdx], _fromPt, _bndCorners[bEdgeIdx]);
                    var distSqr = (intsct - _fromPt).sqrMagnitude;
                    if (distSqr < minDist)
                    {
                        minDist = distSqr;
                        intPt = intsct;
                        _bndDir = (BndSide)bEdgeIdx;
                    }
                }

                return intPt;

            }
            
            private enum BndSide : int { Invalid = -1, Left = 0, Up = 1, Right = 2, Down = 3}
            
            private void ConnectSites(int _delIdx)
            {
                var site = (Site)m_Polys[m_SiteIDByDVertIdx[_delIdx]];
                var nbrSiteIdxs = GetDVertNbrsCWAtSite(_delIdx);
                //Assign twins
                foreach (var nbrIdx in nbrSiteIdxs)
                {
                    if (!m_SiteIDByDVertIdx.ContainsKey(nbrIdx)) continue;
                    var siteID = m_SiteIDByDVertIdx[nbrIdx];
                    var nbrSite = (Site)m_Polys[siteID];

                    var comVertIdxs = new HashSet<int>(nbrSite.VertIdxs).Intersect(site.VertIdxs).ToArray();

                    var thisEdge = site.EdgeWithOrigin(comVertIdxs[0]);
                    var nbrEdge = nbrSite.EdgeWithOrigin(comVertIdxs[1]);
                    
                    if (nbrEdge.NextEdge == null || thisEdge.NextEdge == null || !comVertIdxs.Contains(thisEdge.NextEdge.OriginIdx))
                    {
                        thisEdge = site.EdgeWithOrigin(comVertIdxs[1]);
                        nbrEdge = site.EdgeWithOrigin(comVertIdxs[0]);
                    }

                    thisEdge.Twin = nbrEdge.Twin;
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
                
                public Vertex VertDel => V.D.m_Vertices[VertDelIdx];

                public Site(int[] _vertIdxs, int _vertDelIdx, bool _closed, Voronoi _v) : base(
                    _vertIdxs, _closed, _v)
                {
                    V = _v;
                    VertDelIdx = _vertDelIdx;
                    V.m_SiteIDByDVertIdx.Add(_vertDelIdx, ID);
                }

                public Voronoi V { get; }
            }

            public interface IVoronoiObj
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

            public int[] GetDVertNbrsCWAtSite(int _siteVertIdx)
            {
                var nbrVertIdxs = new HashSet<int>();
                var nbrTriIDs = D.m_PolysContainingVert[_siteVertIdx];
                foreach (var triID in nbrTriIDs)
                    foreach (var vertIdx in D.m_Polys[triID].VertIdxs)
                    {
                        if (_siteVertIdx == vertIdx) continue;
                        nbrVertIdxs.Add(vertIdx);
                    }

                var centSitePos = D.m_Vertices[_siteVertIdx].Pos;
                var vecRef = Vector2f.right - centSitePos;
                var nbrVertIdxsCW = new SortedList<float, int>();
                foreach (var nbrVertIdx in nbrVertIdxs)
                {
                    var vert = D.m_Vertices[nbrVertIdx];
                    var vecNbr = vert.Pos - centSitePos;
                    var theta = vecRef.AngleCW(vecNbr);
                    nbrVertIdxsCW.Add(theta, vert.Idx);
                }

                return nbrVertIdxsCW.Values.ToArray();

            }


            public int[] SortCW(int _siteVertIdx, int[] _vorVertIdxs)
            {
                return SortCW(_siteVertIdx, _vorVertIdxs, Vector2f.right);
            }
            
            public int[] SortCW(int _siteVertIdx, int[] _vorVertIdxs, Vector2f _refVec)
            {
                var sitePos = D.m_Vertices[_siteVertIdx].Pos;
                var centerIdxsCW = new SortedList<float, int>();
                foreach (var vorVertIdx in _vorVertIdxs)
                {
                    var center = m_Vertices[vorVertIdx];
                    var vecCent = (center.Pos - sitePos).normalized;
                    var theta = _refVec.AngleCW(vecCent);
                    centerIdxsCW.Add(theta, center.Idx);

                }

                return centerIdxsCW.Values.ToArray();
            }
            
        }
    }
}