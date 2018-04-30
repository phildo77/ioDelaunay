using System;
using System.Collections.Generic;
using System.Diagnostics;
using ioDelaunay;
using NUnit.Framework;
using Vectorf;
namespace ioDelaunayTest
{
    [TestFixture]
    public class Tests
    {
        public Vector2f tV0 = Vector2f.zero;
        public Vector2f tV1 = Vector2f.right;
        public Vector2f tV2 = Vector2f.up;
        public Vector2f tV3 = Vector2f.one;
        
        
        [Test]
        public void TestCircleSweep()
        {
            
            var siteCnt = 50;
            var width = 1000f;
            var height = 1000f;
            var points = new List<Vector2f>();

            var seed = DateTime.Now.Millisecond;
            var rand = new Random(470);
            Trace.WriteLine("Circle Sweep - Random Seed: " + 470);
            //Ceate random points in map area
            for (int i = 0; i < siteCnt; i++)
            {
                points.Add(new Vector2f(
                    (float)rand.NextDouble() * width,
                    (float)rand.NextDouble() * height)
                );
            }

            var del = new Delaunay(points.ToArray());
            var cs = new CircleSweep();
            del.triangulator = cs;
            del.Triangulate();
            var vSet = new Delaunay.Voronoi.Settings();
            vSet.CloseOuterSites = false;
            var vor = new Delaunay.Voronoi(del,vSet);

            try
            {
                vor.BuildSites2();
            }
            catch (Exception e)
            {
                DebugVisualizer.Visualize(del, null, "ExceptionDEL");
                DebugVisualizer.Visualize(vor);
                DebugVisualizer.Visualize(del, vor, "DelVor");
                throw;
            }
            var mesh = del.Mesh;
            Console.WriteLine(mesh.ToString());

            //DebugVisualizer.Visualize(del);
            //DebugVisualizer.Visualize(vor);
            DebugVisualizer.Visualize(del, vor, "DelVor");
            //DebugVisualizer.Visualize(del, "DelcircumCircles2");
            //DebugVisualizer.TestCirumcircle();
            
            Assert.True(true);
            
        }
    }

    public static class TestExt
    {
        private static float m_Eps = 0.00001f;
        public static bool IsSimilarTo(this float _a, float _b)
        {
            return Math.Abs(_a - _b) < float.Epsilon;
        }
        
    }
}