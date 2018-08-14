
namespace ioDelaunay
{
    public class Mesh
    {
        public readonly int[] Triangles;
        public readonly Vector2[] Vertices;

        public Mesh(Vector2[] _points, int[] _tris)
        {
            Vertices = _points;
            Triangles = _tris;
        }
    }
}