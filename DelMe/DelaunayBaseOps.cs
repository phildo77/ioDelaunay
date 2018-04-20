using System;
using System.Collections.Generic;
using System.Linq;
using Vectorf;

namespace ioDelaunay
{
    public abstract partial class DelaunayBase
    {
        public Triangle AddTriToMesh(int _vertIdx, HalfEdge _joiningEdge)
        {
            var newVerts = new[]
            {
                _vertIdx,
                _joiningEdge.NextEdge.OriginIdx,
                _joiningEdge.OriginIdx
            };
            
            var newTri = new Triangle(newVerts, this);
            var newEdge = newTri.GetHalfEdgeWithOrigin(newVerts[1]); //TODO Check both edge verts?
            newEdge.Twin = _joiningEdge;
            return newTri;

        }

        public Triangle AddTriToMesh(HalfEdge _twinLt, HalfEdge _twinRt)
        {
            //Verify validity
            if(_twinLt.NextEdge.OriginIdx != _twinRt.OriginIdx)
                throw new Exception("AddTriToMesh - twins arent touching");
            
            
            
            var newVerts = new[]
            {
                _twinLt.OriginIdx,
                _twinRt.NextEdge.OriginIdx,
                _twinRt.OriginIdx
            };
            
            if(newVerts[0] == newVerts[1] || newVerts[0] == newVerts[2] || newVerts[1] == newVerts[2])
                throw new Exception("Dupe verts!");
            
            
            var newTri = new Triangle(newVerts, this);
            var newEdgeLt = newTri.GetHalfEdgeWithOrigin(newVerts[2]);
            newEdgeLt.Twin = _twinLt;
            var newEdgeRt = newTri.GetHalfEdgeWithOrigin(newVerts[1]);
            newEdgeRt.Twin = _twinRt;
            return newTri;
        }
        
        //Edge Flip
        private HashSet<int> FlipEdge(Guid _triAID, Guid _triBID)
        {
            var triA = m_Triangles[_triAID];
            var triB = m_Triangles[_triBID];
            
            //Find twin
            var oeA2 = triA.HalfEdge(0);
            var first = oeA2;
            while(true)
            {
                if (oeA2.Twin != null)
                    if (oeA2.Twin.TriID == _triBID)
                        break;
                oeA2 = oeA2.NextEdge;
                if(oeA2 == first)
                    throw new Exception("Flip Edge - Tris not neighbors");
            }

            var oeB2 = oeA2.Twin;
            
            var twinA0 = oeA2.NextEdge.Twin;
            var twinA2 = oeB2.NextEdge.NextEdge.Twin;

            var twinB0 = oeB2.NextEdge.Twin;
            var twinB2 = oeA2.NextEdge.NextEdge.Twin;
            
            var nvA0 = oeA2.NextEdge.OriginIdx;
            var nvA1 = oeA2.NextEdge.NextEdge.OriginIdx;


            var nvB0 = oeB2.NextEdge.OriginIdx;
            var nvB1 = oeB2.NextEdge.NextEdge.OriginIdx;

            var newEdgesA = new[]
            {
                new HalfEdge(triA, nvA0, this),
                new HalfEdge(triA, nvA1, this),
                new HalfEdge(triA, nvB1, this)
            };

            var newEdgesB = new[]
            {
                new HalfEdge(triB, nvB0, this),
                new HalfEdge(triB, nvB1, this),
                new HalfEdge(triB, nvA1, this)
            };

            triA.SetHalfEdges(newEdgesA);
            triB.SetHalfEdges(newEdgesB);
            
            m_TrisContainingVert[nvB0].Remove(_triAID);
            m_TrisContainingVert[nvA0].Remove(_triBID);
            m_TrisContainingVert[nvB1].Add(_triAID);
            m_TrisContainingVert[nvA1].Add(_triBID);

            newEdgesA[0].Twin = twinA0;
            newEdgesA[2].Twin = twinA2;
            newEdgesB[0].Twin = twinB0;
            newEdgesB[2].Twin = twinB2;
            newEdgesA[1].Twin = newEdgesB[1];

            var affectedVertIdxs = new HashSet<int>()
            {
                newEdgesA[0].OriginIdx,
                newEdgesA[1].OriginIdx,
                newEdgesA[2].OriginIdx,
                newEdgesB[0].OriginIdx
            };

            return affectedVertIdxs; 

        }
        
        public HashSet<int> Legalize(params Guid[] _triIDs)
        {
            var affectedVerts = new HashSet<int>();
            var toCheck = new HashSet<Guid>();
            toCheck.UnionWith(_triIDs);
            m_CheckedTris.ExceptWith(_triIDs);
            while (toCheck.Count > 0)
                Legalize(ref affectedVerts, ref toCheck);
            return affectedVerts;
        }
        
        private void Legalize(ref HashSet<int> _affectedVerts, ref HashSet<Guid> _toCheck)
        {
            var curTriID = _toCheck.First();
            _toCheck.Remove(curTriID);
            for(int edgeIdx = 0; edgeIdx < 3; ++edgeIdx)
            {
                var edge = m_Triangles[curTriID].HalfEdge(edgeIdx);
                if (edge.Twin == null) continue;
                
                var nbrID = edge.Twin.TriID;
                var vA2 = edge.NextEdge.NextEdge.Origin;
                var vAB0 = edge.Origin;
                var vAB1 = edge.NextEdge.Origin;
                var vB2 = edge.Twin.NextEdge.NextEdge.Origin;
                
                Vector2f ccCent;
                float ccRad;
                if(!Geom.Circumcircle(vA2.Pos, vAB0.Pos, vAB1.Pos, out ccCent, out ccRad))
                    throw new Exception("TODO - THIS IS PROBABLY A LINE"); //TODO check for line?

                var checkDistSqr = (vB2.Pos - ccCent).sqrMagnitude;
                if (checkDistSqr >= (ccRad * ccRad))
                {
                    if (!m_CheckedTris.Contains(edge.Twin.TriID))
                        _toCheck.Add(edge.Twin.TriID);
                    continue;
                }
                
                _affectedVerts.UnionWith(FlipEdge(edge.TriID, edge.Twin.TriID));

                var newChecks = new HashSet<Guid> {curTriID, nbrID};
                newChecks.UnionWith(m_Triangles[curTriID].NeighborIDs);
                newChecks.UnionWith(m_Triangles[nbrID].NeighborIDs);

                m_CheckedTris.ExceptWith(newChecks);
                _toCheck.UnionWith(newChecks);
                return;
                
            }

            m_CheckedTris.Add(curTriID);
        }
    }
}