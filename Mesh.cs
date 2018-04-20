using Vectorf;

namespace ioDelaunay
{
    public class Mesh
    {
        public readonly int[] Triangles;
        public readonly Vector2f[] Vertices;

        public Mesh(Vector2f[] _points, int[] _tris)
        {
            Vertices = _points;
            Triangles = _tris;
        }
    }
}