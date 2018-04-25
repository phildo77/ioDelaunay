using System;
using System.Collections.Generic;
using System.Linq;
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
                for(int delIdx = 0; delIdx < D.m_Vertices.Count; ++delIdx)
                {
                    if (D.HullIdxs.Contains(delIdx)) continue;
                    var dVert = D.m_Vertices[delIdx];
                    var centers = GetCenterIdxsAtSite(delIdx);
                    var centersCW = SortCW(delIdx, centers.ToArray());

                    var newSite = new Site(centersCW, delIdx, true, this);
                    
                    m_SiteIDByDVertIdx.Add(delIdx, newSite.ID);

                    var nbrSiteIdxs = GetDVertNbrsCWAtSite(delIdx);

                    foreach (var nbrIdx in nbrSiteIdxs)
                    {
                        if (!m_SiteIDByDVertIdx.ContainsKey(nbrIdx)) continue;
                        var siteID = m_SiteIDByDVertIdx[nbrIdx];
                        var nbrSite = (Site)m_Polys[siteID];

                        var comVertIdxs = new HashSet<int>(nbrSite.VertIdxs).Intersect(newSite.VertIdxs).ToArray();

                        var thisEdge = newSite.EdgeWithOrigin(comVertIdxs[0]);
                        var nbrEdge = nbrSite.EdgeWithOrigin(comVertIdxs[1]);
                        if (!comVertIdxs.Contains(thisEdge.NextEdge.OriginIdx))
                        {
                            thisEdge = newSite.EdgeWithOrigin(comVertIdxs[1]);
                            nbrEdge = newSite.EdgeWithOrigin(comVertIdxs[0]);
                        }

                        thisEdge.Twin = nbrEdge.Twin;
                    }

                }
            }
            /*
            public static Voronoi Create(Delaunay _d, Settings _settings = null)
            {
                if(_settings == null)
                    _settings = new Settings();
                var points = new List<Vector2f>();
                var triIDs = _d.m_Polys.Keys.ToArray();
                for (var tIdx = 0; tIdx < _d.m_Polys.Count; ++tIdx)
                {
                    var triID = triIDs[tIdx];
                    var tri = (Triangle) _d.m_Polys[triID];

                    float r;
                    Vector2f center;
                    tri.CircumCircle(out center, out r);
                    points.Add(center);
                }


                var siteFwdHullVertIdxs = new Dictionary<int, int[]>();
                
                //Add / Create Hull verts
                var bounds = _d.BoundsRect;
                var boundsSize = bounds.size * _settings.BoundaryExpansionPct;
                var boundsCenter = bounds.center;
                var newBounds = new Rectf(bounds.position, boundsSize);
                newBounds.center = boundsCenter;
                bounds = newBounds;
                var ll = bounds.min;
                var ur = bounds.max;
                var lr = new Vector2f(bounds.xMax, bounds.yMin);
                var ul = new Vector2f(bounds.xMin, bounds.yMax);
                points.Add(ll);
                points.Add(ul);
                points.Add(ur);
                points.Add(lr);
                
                var bVecLeft = ul - ll;
                var bVecUp = ur - ul;
                var bVecRight = lr - ur;
                var bVecDown = ll - lr;
                var curSideIdx = -1;
                var hullCornerIdxs = new[] {points.Count - 4, points.Count - 3, points.Count - 2, points.Count - 1};
                for (int hIdx = 0; hIdx < _d.HullIdxs.Length; ++hIdx)
                {
                    var vIdx = _d.HullIdxs[hIdx];
                    var nextIdx = hIdx == _d.HullIdxs.Length - 1 ? 0 : hIdx + 1;
                    var v0 = _d.m_Vertices[vIdx];
                    var v1 = _d.m_Vertices[_d.HullIdxs[nextIdx]];
                    var hullVec = v1.Pos - v0.Pos;
                    var halfVec = hullVec / 2f;
                    var biVec = new Vector2f(-hullVec.y, hullVec.x).normalized;
                    var intVecs = new List<Vector2f>()
                    {
                        Intersect(biVec, bVecLeft, halfVec, ll),
                        Intersect(biVec, bVecUp, halfVec, ul),
                        Intersect(biVec, bVecRight, halfVec, ur),
                        Intersect(biVec, bVecDown, halfVec, lr)
                    };

                    var intMags = intVecs.Select(_int => _int.sqrMagnitude).ToArray();
                    var lowestMag = float.PositiveInfinity;
                    var lowestIdx = 0;
                    for (int sideIdx = 0; sideIdx < 4; ++sideIdx)
                    {
                        if (!(intMags[sideIdx] < lowestMag)) continue;
                        lowestIdx = sideIdx;
                        lowestMag = intMags[sideIdx];
                    }

                    points.Add(intVecs[lowestIdx]);

                    if (curSideIdx == -1)
                    {
                        curSideIdx = lowestIdx;
                        siteFwdHullVertIdxs.Add(vIdx, new [] { points.Count - 1});
                    }
                    else
                    {
                        if (curSideIdx != lowestIdx)
                        {
                            curSideIdx = lowestIdx;
                            siteFwdHullVertIdxs.Add(vIdx,new [] { hullCornerIdxs[curSideIdx], points.Count - 1 });
                        }
                        else
                        {
                            siteFwdHullVertIdxs.Add(vIdx, new[] {points.Count - 1});
                        }
                    }
                }

                return new Voronoi(points.ToArray(), siteFwdHullVertIdxs, _d, _settings);
            }
            */
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

            /*
            public void BuildHullSites()
            {
                var prevSite = Guid.Empty;
                var lastCorner = -1;
                var firstSite = Guid.Empty;
                for (int hIdx = 0; hIdx < D.HullIdxs.Length; ++hIdx)
                {
                    var vertIdxs = new List<int>();
                    var backIdx = hIdx == 0 ? D.HullIdxs.Length - 1 : hIdx - 1;
                    var prevHullIdx = m_HullIdxMap[backIdx][0];
                    vertIdxs.Add(prevHullIdx);
                    if (m_HullIdxMap[hIdx].Length == 2)
                    {
                        lastCorner = hIdx;
                        vertIdxs.Add(m_HullIdxMap[hIdx][1]);
                    }
                    vertIdxs.Add(m_HullIdxMap[hIdx][0]);

                    var centerIdxs = GetCenterIdxsCWAtSite(D.HullIdxs[hIdx]);
                    for (int cIdx = 0; cIdx < centerIdxs.Length; ++cIdx)
                        vertIdxs.Add(centerIdxs[cIdx]);

                    var newSite = new Site(vertIdxs.ToArray(), D.HullIdxs[hIdx], true, this);
                    m_SiteIDByDVertIdx.Add(D.HullIdxs[hIdx], newSite.ID);
                    m_HullSites.Add(newSite.ID);
                    
                    if (prevSite == Guid.Empty)
                    {
                        prevSite = firstSite = newSite.ID;
                        continue;
                    }

                    var backEdge = newSite.Edge(vertIdxs.Count - 1);
                    backEdge.Twin = m_Polys[prevSite].Edge(lastCorner == backIdx ? 2 : 1);

                    if (hIdx == D.HullIdxs.Length - 1)
                    {
                        var newEdgeIdx = 1;
                        if (lastCorner == hIdx)
                            newEdgeIdx = 2;
                        var firstSiteVCnt = m_Polys[firstSite].VertIdxs.Length;
                        newSite.Edge(newEdgeIdx).Twin = m_Polys[firstSite].Edge(firstSiteVCnt - 1);
                    }
                }
            }
            */

            public int[] SortCW(int _siteVertIdx, int[] _vorVertIdxs)
            {
                var sitePos = D.m_Vertices[_siteVertIdx].Pos;
                var vecRef = (Vector2f.right - sitePos).normalized;
                var centerIdxsCW = new SortedList<float, int>();
                foreach (var vorVertIdx in _vorVertIdxs)
                {
                    var center = m_Vertices[vorVertIdx];
                    var vecCent = (center.Pos - sitePos).normalized;
                    var theta = vecRef.AngleCW(vecCent);
                    centerIdxsCW.Add(theta, center.Idx);

                }

                return centerIdxsCW.Values.ToArray();
            }
            
            public void BuildSites()
            {
                var hullIdxs = new HashSet<int>(D.HullIdxs);
                for (int vIdx = 0; vIdx < D.m_Vertices.Count; ++vIdx)
                {
                    if (hullIdxs.Contains(vIdx)) continue; // TODO DEBUG
                    var centVerts = GetCenterIdxsAtSite(vIdx);
                    if (hullIdxs.Contains(vIdx))
                    {
                        //Hull site
                        centVerts.UnionWith(m_HullIdxMap[vIdx]);
                    }

                    var centVertsCW = SortCW(vIdx, centVerts.ToArray());
                    
                    
                    var newSite = new Site(centVertsCW, vIdx, true, this);

                    m_SiteIDByDVertIdx.Add(vIdx, newSite.ID);

                    var nbrSiteIdxs = GetDVertNbrsCWAtSite(vIdx);

                    foreach (var nbrIdx in nbrSiteIdxs)
                    {
                        if (!m_SiteIDByDVertIdx.ContainsKey(nbrIdx)) continue;
                        var siteID = m_SiteIDByDVertIdx[nbrIdx];
                        var nbrSite = (Site)m_Polys[siteID];

                        var comVertIdxs = new HashSet<int>(nbrSite.VertIdxs).Intersect(newSite.VertIdxs).ToArray();

                        var thisEdge = newSite.EdgeWithOrigin(comVertIdxs[0]);
                        var nbrEdge = nbrSite.EdgeWithOrigin(comVertIdxs[1]);
                        if (!comVertIdxs.Contains(thisEdge.NextEdge.OriginIdx))
                        {
                            thisEdge = newSite.EdgeWithOrigin(comVertIdxs[1]);
                            nbrEdge = newSite.EdgeWithOrigin(comVertIdxs[0]);
                        }

                        thisEdge.Twin = nbrEdge.Twin;
                    }
                }
            }
        }
    }
}