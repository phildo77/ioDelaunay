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
            
            var siteCnt = 1000;
            var width = 1500;
            var height = 1500;
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

            Delaunay del = null;
            Delaunay.Voronoi vor = null;
            try
            {
                del = new Delaunay(points.ToArray());
                var cs = new CircleSweep();
                del.triangulator = cs;
                del.Triangulate();
                var vSet = new Delaunay.Voronoi.Settings();
                vSet.CloseOuterSites = false;
                vor = new Delaunay.Voronoi(del,vSet);
            }
            catch (Exception e)
            {
                Console.WriteLine(e);
                throw;
            }
            

            try
            {
                vor.BuildSites();
            }
            catch (Exception e)
            {
                DebugVisualizer.Visualize(del, null, "ExceptionDEL");
                //DebugVisualizer.Visualize(vor);
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


            for (int lIdx = 0; lIdx < 20; ++lIdx)
            {
                try
                {
                    vor.LloydRelax();
                    DebugVisualizer.Visualize(del, vor, "DelVorLloyd " + lIdx);
                }
                catch (Exception _e)
                {
                    DebugVisualizer.Visualize(del, vor, "DelVorLloydEx " + lIdx);
                }
            }
            
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