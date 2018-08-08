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
        public static bool Enabled = false;
        public static Vector2f OriginOffsetOverride = Vector2f.positiveInfinity;
        public static void Visualize(Delaunay _d, Delaunay.Voronoi _v = null, string _fileName = "debugMesh")
        {
            if (!Enabled) return;
            //var bitmap = new Bitmap((int) (_d.BoundsRect.width * 1.2f), (int) (_d.BoundsRect.height * 1.2f));
            var bitmap = new Bitmap((int) (1500), (int) (1500));
            
            var originOffset = _d.BoundsRect.min;
            originOffset.x += 0 - bitmap.Width / 2f;
            originOffset.y += 0 - bitmap.Height / 2f;
            if (!float.IsInfinity(OriginOffsetOverride.x))
            {
                originOffset = _d.BoundsRect.min;
                originOffset.x += OriginOffsetOverride.x - bitmap.Width / 2f;
                originOffset.y += OriginOffsetOverride.y - bitmap.Height / 2f;
            }
            
            var vectTextDrawn = new HashSet<int>();


            //Draw Mesh
            foreach (var tri in _d.Triangles)
                for (var idx = 0; idx < 3; ++idx)
                {
                    var edge = tri.Edge(idx);
                    var x1 = edge.OriginPos.x - originOffset.x;
                    var y1 = edge.OriginPos.y - originOffset.y;
                    var x2 = edge.NextEdge.OriginPos.x - originOffset.x;
                    var y2 = edge.NextEdge.OriginPos.y - originOffset.y;

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
                var pts = _d.Points;
                var x1 = pts[fPt.VertIdx].x - originOffset.x;
                var y1 = pts[fPt.VertIdx].y - originOffset.y;
                var x2 = pts[fPt.Right.VertIdx].x - originOffset.x;
                var y2 = pts[fPt.Right.VertIdx].y - originOffset.y;
                
                //var x1 = fPt.Pos.x - originOffset.x;
                //var y1 = fPt.Pos.y - originOffset.y;
                //var x2 = fPt.Right.Pos.x - originOffset.x;
                //var y2 = fPt.Right.Pos.y - originOffset.y;

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
                    for (var idx = 0; idx < site.VertIdxs.Count; ++idx)
                    {
                        if ((idx == (site.VertIdxs.Count - 1)) && !site.Closed) break;
                        try
                        {
                            var edge = site.Edge(idx);
                            var x1 = edge.OriginPos.x - originOffset.x;
                            var y1 = edge.OriginPos.y - originOffset.y;
                            var x2 = edge.NextEdge.OriginPos.x - originOffset.x;
                            var y2 = edge.NextEdge.OriginPos.y - originOffset.y;

                            using (var g = Graphics.FromImage(bitmap))
                            {
                                g.SmoothingMode = SmoothingMode.None;
                                g.InterpolationMode = InterpolationMode.Low;
                                g.PixelOffsetMode = PixelOffsetMode.None;
                                var pen = new Pen(m_ColorVor);
                                g.DrawLine(pen, x1, y1, x2, y2);

                                if (!showVertIdxs) continue;
                                var textRect = new RectangleF(new PointF(x1, y1), new SizeF(45, 20));

                                g.DrawString(edge.OriginIdx.ToString(), new Font("Small Fonts", fontSize), Brushes.Blue,
                                    textRect);
                            }
                        }
                        catch (Exception e)
                        {
                            continue;
                        }
                    
                    }
            }
            
            //Draw Voronoi Bounds Rect
            if (_v != null)
            {
                var dBnds = new Rectf(_v.DBounds);
                dBnds.center = dBnds.center - originOffset;
                
            
                using (var g = Graphics.FromImage(bitmap))
                {
                    g.SmoothingMode = SmoothingMode.None;
                    g.InterpolationMode = InterpolationMode.Low;
                    g.PixelOffsetMode = PixelOffsetMode.None;
                    var pen = new Pen(Color.Orange);
                    //g.DrawRectangle(pen, dBnds.xMin, dBnds.yMin, dBnds.width, dBnds.height);
                }                
            }
            
            //Temp
            {
                var cent = ((CircleSweep)(_d.triangulator)).Origin - originOffset;
                var v1 = _d.Points[326757] - originOffset;
                
                using (var g = Graphics.FromImage(bitmap))
                {
                    g.SmoothingMode = SmoothingMode.None;
                    g.InterpolationMode = InterpolationMode.Low;
                    g.PixelOffsetMode = PixelOffsetMode.None;
                    var pen = new Pen(Color.Yellow);
                    g.DrawLine(pen, cent.x, cent.y, v1.x, v1.y);

                }
            }
            
            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".bmp";
            bitmap.Save(path);
        }

        public static void SetZoom(Vector2f _center, Vector2f _size)
        {
            
        }

        public static Color RandColor(Random _r)
        {
            int r = (int)((0.2f + _r.NextDouble() * 0.8f) * 255);
            int g = (int)((0.2f + _r.NextDouble() * 0.8f) * 255);
            int b = (int)((0.2f + _r.NextDouble() * 0.8f) * 255);
            return Color.FromArgb(r, g, b);
        }
        
        public static void Visualize(Delaunay.Voronoi _v, string _fileName = "debugVoronoi2")
        {
            if (!Enabled) return;
            var bitmap = new Bitmap((int) (_v.BoundsRect.width * 1.2f), (int) (_v.BoundsRect.height * 1.2f));
            var originOffset = _v.BoundsRect.min;
            var vectTextDrawn = new HashSet<int>();


            //Draw Sites
            var rand = new Random((int)DateTime.Now.Ticks);
            var sites = _v.Sites;
            foreach (var site in sites)
                for (var idx = 0; idx < site.VertIdxs.Count; ++idx)
                {
                    
                    try
                    {
                        var edge = site.Edge(idx);
                        var x1 = edge.OriginPos.x - originOffset.x;
                        var y1 = edge.OriginPos.y - originOffset.y;
                        var x2 = edge.NextEdge.OriginPos.x - originOffset.x;
                        var y2 = edge.NextEdge.OriginPos.y - originOffset.y;

                        using (var g = Graphics.FromImage(bitmap))
                        {
                            g.SmoothingMode = SmoothingMode.None;
                            g.InterpolationMode = InterpolationMode.Low;
                            g.PixelOffsetMode = PixelOffsetMode.None;
                            var pen = new Pen(RandColor(rand));
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
        
        public static Bitmap CropImage(Bitmap source, Rectangle section)
        {
            // An empty bitmap which will hold the cropped image
            Bitmap bmp = new Bitmap(section.Width, section.Height);

            Graphics g = Graphics.FromImage(bmp);

            // Draw the given area (section) of the source image
            // at location 0,0 on the empty bitmap (bmp)
            g.DrawImage(source, 0, 0, section, GraphicsUnit.Pixel);

            return bmp;
        }
    }
}