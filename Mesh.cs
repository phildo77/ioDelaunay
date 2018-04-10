using Vectorf;

namespace ioDelaunay
{
    
    public class Mesh
    {
        public readonly Vector2f[] Vertices;
        public readonly int[] Triangles;

        public Mesh(Vector2f[] _points, int[] _tris)
        {
            Vertices = _points;
            Triangles = _tris;
        }
    }
}