using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using Vectorf;

namespace ioDelaunay
{
    public static class DebugVisualizer
    {
        public static int fontSize = 6;
        public static bool showVertIdxs = true;
        private static readonly Color m_ColorMesh = Color.White;
        private static readonly Color m_ColorFront = Color.Red;
        private static Color m_ColorFont = Color.BurlyWood;
        private static readonly Color m_ColorCircles = Color.Aquamarine;
        private static readonly Color m_ColorVor = Color.DodgerBlue;

        public static void Visualize(Delaunay _d, Delaunay.Voronoi _v = null, string _fileName = "debugMesh")
        {
            //var bitmap = new Bitmap((int) (_d.BoundsRect.width * 1.2f), (int) (_d.BoundsRect.height * 1.2f));
            var bitmap = new Bitmap((int) (1200), (int) (1200));
            var originOffset = _d.BoundsRect.min;
            var vectTextDrawn = new HashSet<int>();


            //Draw Mesh
            foreach (var tri in _d.Triangles)
                for (var idx = 0; idx < 3; ++idx)
                {
                    var edge = tri.Edge(idx);
                    var x1 = edge.Origin.Pos.x - originOffset.x;
                    var y1 = edge.Origin.Pos.y - originOffset.y;
                    var x2 = edge.NextEdge.Origin.Pos.x - originOffset.x;
                    var y2 = edge.NextEdge.Origin.Pos.y - originOffset.y;

                    using (var g = Graphics.FromImage(bitmap))
                    {
                        g.SmoothingMode = SmoothingMode.None;
                        g.InterpolationMode = InterpolationMode.Low;
                        g.PixelOffsetMode = PixelOffsetMode.None;
                        var pen = new Pen(m_ColorMesh);
                        g.DrawLine(pen, x1, y1, x2, y2);

                        if (!showVertIdxs) continue;
                        var textRect = new RectangleF(new PointF(x1, y1), new SizeF(45, 20));

                        g.DrawString(edge.OriginIdx.ToString(), new Font("Small Fonts", fontSize), Brushes.BurlyWood,
                            textRect);
                    }
                }

            //Draw Frontier
            var cs = (CircleSweep) _d.triangulator;
            var fPt = cs.frontier.Project(0)[0];
            var fpStart = fPt;
            var firstMoveDone = false;
            while (fPt.VertIdx != fpStart.VertIdx || firstMoveDone == false)
            {
                var x1 = fPt.Vert.Pos.x - originOffset.x;
                var y1 = fPt.Vert.Pos.y - originOffset.y;
                var x2 = fPt.Right.Vert.Pos.x - originOffset.x;
                var y2 = fPt.Right.Vert.Pos.y - originOffset.y;

                using (var g = Graphics.FromImage(bitmap))
                {
                    g.SmoothingMode = SmoothingMode.None;
                    g.InterpolationMode = InterpolationMode.Low;
                    g.PixelOffsetMode = PixelOffsetMode.None;
                    var pen = new Pen(m_ColorFront);
                    g.DrawLine(pen, x1, y1, x2, y2);
                }

                fPt = fPt.Right;
                firstMoveDone = true;
            }

            
            
            //Draw Circumcircles
            /*
            foreach (var tri in _d.Triangles)
            {
                float r;
                Vector2f center;
                tri.CircumCircle(out center, out r);
                var rul = new Vector2f(center.x - r, center.y - r);
                var rect = new Rectangle((int)rul.x, (int)rul.y, (int)(2 * r), (int)(2 * r));
                using (var g = Graphics.FromImage(bitmap))
                {
                    
                    
                    g.SmoothingMode = SmoothingMode.None;
                    g.InterpolationMode = InterpolationMode.Low;
                    g.PixelOffsetMode = PixelOffsetMode.None;

                    var pen = new Pen(m_ColorCircles);
                    g.DrawEllipse(pen, (center.x - r) - originOffset.x, (center.y - r) - originOffset.y, r * 2, r * 2);
                    //g.DrawRectangle(pen, rect);
                    //g.DrawLine(pen, center.x, center.y, rect.Left, rect.Top);
                }
            }
            */
            
            //Draw Origin
            using (var g = Graphics.FromImage(bitmap))
            {
                g.SmoothingMode = SmoothingMode.None;
                g.InterpolationMode = InterpolationMode.Low;
                g.PixelOffsetMode = PixelOffsetMode.None;
                var pen = new Pen(m_ColorFront);
                var x1 = cs.Origin.x - originOffset.x;
                var y1 = cs.Origin.y - originOffset.y;
                g.FillRectangle(Brushes.Cyan, x1, y1, 2, 2);
            }
            
            //Voronoi on top
            if (_v != null)
            {
                var sites = _v.Sites;
                foreach (var site in sites)
                    for (var idx = 0; idx < site.VertIdxs.Length; ++idx)
                    {
                    
                        try
                        {
                            var edge = site.Edge(idx);
                            var x1 = edge.Origin.Pos.x - originOffset.x;
                            var y1 = edge.Origin.Pos.y - originOffset.y;
                            var x2 = edge.NextEdge.Origin.Pos.x - originOffset.x;
                            var y2 = edge.NextEdge.Origin.Pos.y - originOffset.y;

                            using (var g = Graphics.FromImage(bitmap))
                            {
                                g.SmoothingMode = SmoothingMode.None;
                                g.InterpolationMode = InterpolationMode.Low;
                                g.PixelOffsetMode = PixelOffsetMode.None;
                                var pen = new Pen(m_ColorVor);
                                g.DrawLine(pen, x1, y1, x2, y2);

                                if (!showVertIdxs) continue;
                                var textRect = new RectangleF(new PointF(x1, y1), new SizeF(45, 20));

                                g.DrawString(edge.OriginIdx.ToString(), new Font("Small Fonts", fontSize), Brushes.BurlyWood,
                                    textRect);
                            }
                        }
                        catch (Exception e)
                        {
                            continue;
                        }
                    
                    }
            }
            

            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".bmp";
            bitmap.Save(path);
        }
        
        public static void Visualize(Delaunay.Voronoi _v, string _fileName = "debugVoronoi2")
        {
            var bitmap = new Bitmap((int) (_v.BoundsRect.width * 1.2f), (int) (_v.BoundsRect.height * 1.2f));
            var originOffset = _v.BoundsRect.min;
            var vectTextDrawn = new HashSet<int>();


            //Draw Sites
            var sites = _v.Sites;
            foreach (var site in sites)
                for (var idx = 0; idx < site.VertIdxs.Length; ++idx)
                {
                    
                    try
                    {
                        var edge = site.Edge(idx);
                        var x1 = edge.Origin.Pos.x - originOffset.x;
                        var y1 = edge.Origin.Pos.y - originOffset.y;
                        var x2 = edge.NextEdge.Origin.Pos.x - originOffset.x;
                        var y2 = edge.NextEdge.Origin.Pos.y - originOffset.y;

                        using (var g = Graphics.FromImage(bitmap))
                        {
                            g.SmoothingMode = SmoothingMode.None;
                            g.InterpolationMode = InterpolationMode.Low;
                            g.PixelOffsetMode = PixelOffsetMode.None;
                            var pen = new Pen(m_ColorMesh);
                            g.DrawLine(pen, x1, y1, x2, y2);

                            if (!showVertIdxs) continue;
                            var textRect = new RectangleF(new PointF(x1, y1), new SizeF(45, 20));

                            g.DrawString(edge.OriginIdx.ToString(), new Font("Small Fonts", fontSize), Brushes.BurlyWood,
                                textRect);
                        }
                    }
                    catch (Exception e)
                    {
                        continue;
                    }
                    
                }

            
            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".bmp";
            bitmap.Save(path);
        }

        public static void TestCirumcircle()
        {
            var bitmap = new Bitmap(1000, 1000);
            var origin = Vector2f.one * 50f;
            var v0 = origin + Vector2f.zero * 50f;
            var v1 = origin + Vector2f.up * 50f;
            var v2 = origin + Vector2f.right * 50f;
            float r;
            Vector2f center;
            Geom.Circumcircle(v0, v1, v2, out center, out r);
            
            using (var g = Graphics.FromImage(bitmap))
            {
                g.SmoothingMode = SmoothingMode.None;
                g.InterpolationMode = InterpolationMode.Low;
                g.PixelOffsetMode = PixelOffsetMode.None;
                var pen = new Pen(m_ColorMesh);

                g.DrawLine(pen, v0.x, v0.y, v1.x, v1.y);
                g.DrawLine(pen, v1.x, v1.y, v2.x, v2.y);
                g.DrawLine(pen, v2.x, v2.y, v0.x, v0.y);


                g.DrawEllipse(pen, center.x - r, center.y - r, r * 2, r * 2);
                
                var path = AppDomain.CurrentDomain.BaseDirectory + "TestCircumcircle" + ".bmp";
                bitmap.Save(path);
            }
            
        }
    }
}