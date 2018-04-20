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
            
            var siteCnt = 1000000;
            var width = 10000f;
            var height = 10000f;
            var points = new List<Vector2f>();

            var seed = DateTime.Now.Millisecond;
            var rand = new Random(seed);
            Trace.WriteLine("Circle Sweep - Random Seed: " + seed);
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


            try
            {
                del.Triangulate();

            }
            catch (Exception e)
            {
                //DebugVisualizer.Visualize(del);
                throw;
            }
            var mesh = del.Mesh;
            Console.WriteLine(mesh.ToString());

            //DebugVisualizer.Visualize(del);
            
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