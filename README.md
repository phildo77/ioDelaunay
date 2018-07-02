# ioDelaunay
Delaunay Triangulation and Voronoi in C#

Work in progress / In development / Alpha

Usage Example:

```
		var siteCnt = 10000;
		var width = 5000;
		var height = 5000;
		var points = new List<Vector2f>();
		
		var seed = DateTime.Now.Millisecond;
		var rand = new Random(seed);

		//Ceate random points to triangulate
		for (int i = 0; i < siteCnt; i++)
		{
			points.Add(new Vector2f(
				(float)rand.NextDouble() * width,
				(float)rand.NextDouble() * height)
			);
		}
		
		//Create Delaunay object
		Delaunay del = new Delaunay(points.ToArray());

		//Set triagulator as Circle Sweep and triangulate
		var cs = new CircleSweep();
		del.triangulator = cs;
		del.Triangulate();

		//Create Voronoi Object
		var vSet = new Delaunay.Voronoi.Settings();
		vSet.CloseOuterSites = false;
		Delaunay.Voronoi vor = new Delaunay.Voronoi(del,vSet);;
		
		vor.BuildSites();
		
		//Perform Lloyd Relaxation algorithm
		vor.LloydRelax(2);

		Mesh mesh = vor.D.Mesh;

		var verts = mesh.Vertices;
		var tris = mesh.Triangles;

		//Do something with mesh, etc. etc.
```
