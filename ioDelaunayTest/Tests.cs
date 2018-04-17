using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
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
            var width = 500f;
            var height = 500f;
            var points = new List<Vector2f>();

            var rand = new Random();
            //Ceate random points in map area
            for (int i = 0; i < siteCnt; i++)
            {
                points.Add(new Vector2f(
                    (float)rand.NextDouble() * width,
                    (float)rand.NextDouble() * height)
                );
            }

            var cs = new CircleSweep(points.ToArray());

            try
            {
                var mesh = cs.Triangulate();
                Console.WriteLine(mesh.ToString());
            }
            catch (Exception e)
            {
                DebugVisualizer.Visualize(cs);
                throw;
            }

            DebugVisualizer.Visualize(cs);
            
            Assert.True(true);
            
        }
           

        /*
        [Test]
        public void AngleFromOrigin()
        {
            var pt1 = Vector2f.one;
            var pt2 = Vector2f.up;
            var pt3 = Vector2f.left;
            var pt4 = Vector2f.right;
            var pt5 = Vector2f.down;
            var pt6 = -Vector2f.one;

            var origin = Vector2f.zero;

            var a1 = CircleSweep.AngleFromOrigin(origin, pt1);
            var deg225 = 7f * (float)Math.PI / 4f;

            var a2 = CircleSweep.AngleFromOrigin(origin, pt2);
            var deg270 = 3f * (float)Math.PI / 2f;
            
            var a3 = CircleSweep.AngleFromOrigin(origin, pt3);
            var deg180 = (float) Math.PI;
            
            var a4 = CircleSweep.AngleFromOrigin(origin, pt4);
            var deg360 = 2f * (float)Math.PI;
            
            var a5 = CircleSweep.AngleFromOrigin(origin, pt5);
            var deg90 = (float) Math.PI / 2f;

            var a6 = CircleSweep.AngleFromOrigin(origin, pt6);
            var deg135 = 3f * (float) Math.PI / 4f;

            var pass = a1.IsSimilarTo(deg225);
            if (!a2.IsSimilarTo(deg270) ) pass = false;
            if (!a3.IsSimilarTo(deg180)) pass = false;
            if (!a4.IsSimilarTo(deg360)) pass = false;
            if (!a5.IsSimilarTo(deg90)) pass = false;
            if (!a6.IsSimilarTo(deg135)) pass = false;

            Assert.IsTrue(pass);

        }
        */
    
        
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