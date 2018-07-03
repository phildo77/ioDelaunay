# ioDelaunay
Delaunay Triangulation and Voronoi in C#

Work in progress / In development / Alpha

Usage Example:

```
	    var siteCnt = 10000;
            var width = 3000;
            var height = 3000;
            var points = new List<Vector2f>();

            var seed = DateTime.Now.Millisecond;
            var rand = new Random(seed);

            //Ceate random points in map area
            for (int i = 0; i < siteCnt; i++)
            {
                points.Add(new Vector2f(
                    (float)rand.NextDouble() * width,
                    (float)rand.NextDouble() * height)
                );
            }

            Delaunay del = Delaunay.Create<CircleSweep>(points.ToArray());
            del.Triangulate();
            var vSet = new Delaunay.Voronoi.Settings
            {
                CloseOuterSites = true,
            	TrimSitesAtBoundary = true
            };
            Delaunay.Voronoi vor = new Delaunay.Voronoi(del,vSet);
            vor.BuildSites(); 

	    vor.LloydRelax(2);
            var mesh = del.Mesh;

	    //Do something with mesh, etc. etc.
```
