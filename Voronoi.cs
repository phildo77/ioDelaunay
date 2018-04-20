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
            private readonly Dictionary<int, Guid> m_TriIDByVertIdx;
            private readonly Dictionary<Guid, int> m_VertIdxsByTriID; //Delaunay triangle centers;

            private Voronoi(Vector2f[] _points, Delaunay _d) : base(_points)
            {
                D = _d;
                m_VertIdxsByTriID = new Dictionary<Guid, int>();
                m_TriIDByVertIdx = new Dictionary<int, Guid>();
                var triIDs = _d.m_Polys.Keys.ToArray();
                for (var tIdx = 0; tIdx < triIDs.Length; ++tIdx)
                {
                    m_VertIdxsByTriID.Add(triIDs[tIdx], tIdx);
                    m_TriIDByVertIdx.Add(tIdx, triIDs[tIdx]);
                }
            }

            public Delaunay D { get; }

            public static Voronoi Create(Delaunay D)
            {
                var points = new List<Vector2f>();
                var triIDs = D.m_Polys.Keys.ToArray();
                for (var tIdx = 0; tIdx < D.m_Polys.Count; ++tIdx)
                {
                    var triID = triIDs[tIdx];
                    var tri = (Triangle) D.m_Polys[triID];

                    float r;
                    Vector2f center;
                    tri.CircumCircle(out center, out r);
                    points.Add(center);
                }

                return null;
            }

            public class Site : Poly, IVoronoiObj
            {
                public readonly int Idx;

                public Site(int[] _vertIdxsOrdered, int _idx, bool _closed, PolygonGraph _g, Voronoi _v) : base(
                    _vertIdxsOrdered, _closed, _g)
                {
                    V = _v;
                    Idx = _idx;
                }

                public Voronoi V { get; }
            }

            public interface IVoronoiObj
            {
                Voronoi V { get; }
            }
        }
    }
}