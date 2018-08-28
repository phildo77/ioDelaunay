using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.Linq;
using System.Text;

namespace ioDelaunay
{
    public static class DebugVisualizer
    {
        public static int fontSize = 5;
        public static bool showVertIdxs = false;
        private static readonly Color m_ColorMesh = Color.White;
        private static readonly Color m_ColorFront = Color.Red;
        private static Color m_ColorFont = Color.BurlyWood;
        private static readonly Color m_ColorCircles = Color.Aquamarine;
        private static readonly Color m_ColorVor = Color.DodgerBlue;
        public static bool Enabled = false;
        public static Vector2 OriginOffsetOverride = Vector2.positiveInfinity;
        public static Random rnd = new Random((int) DateTime.Now.Ticks);

        private static HashSet<Delaunay.Triangle> RecGetTris(Delaunay.Triangle.HalfEdge _edge, int _depth)
        {
            var tris = new HashSet<Delaunay.Triangle> {_edge.Triangle};
            if (_depth - 1 == 0)
                return tris;

            if (_edge.NextEdge.Twin != null)
                tris.UnionWith(RecGetTris(_edge.NextEdge.Twin, _depth - 1));
            if (_edge.NextEdge.NextEdge.Twin != null)
                tris.UnionWith(RecGetTris(_edge.NextEdge.NextEdge.Twin, _depth - 1));

            return tris;
        }

        public static void Visualize(string _fileName, Delaunay.Triangle.HalfEdge _edge, int _depth = 6)
        {
            if (!Enabled) return;
            var trisToRender = new HashSet<Delaunay.Triangle> {_edge.Triangle};

            if (_edge.Twin != null)
                trisToRender.UnionWith(RecGetTris(_edge.Twin, _depth));

            if (_edge.NextEdge.Twin != null)
                trisToRender.UnionWith(RecGetTris(_edge.NextEdge.Twin, _depth));

            if (_edge.NextEdge.NextEdge.Twin != null)
                trisToRender.UnionWith(RecGetTris(_edge.NextEdge.NextEdge.Twin, _depth));

            var triList = trisToRender.ToList();
            var bounds = new Rect(Rect.zero);
            var pts = triList[0].D.Points;
            var tris = new Vector2[triList.Count * 3];

            for (var tIdx = 0; tIdx < triList.Count; ++tIdx)
            {
                tris[tIdx * 3] = pts[triList[tIdx].Edge0.OriginIdx];
                tris[tIdx * 3 + 1] = pts[triList[tIdx].Edge1.OriginIdx];
                tris[tIdx * 3 + 2] = pts[triList[tIdx].Edge2.OriginIdx];
            }

            var minX = tris.Min(_tri => _tri.x);
            var maxX = tris.Max(_tri => _tri.x);
            var minY = tris.Min(_tri => _tri.y);
            var maxY = tris.Max(_tri => _tri.y);

            for (var tIdx = 0; tIdx < triList.Count; ++tIdx)
                tris[tIdx] = new Vector2(tris[tIdx].x - minX, tris[tIdx].y - minY);

            var scale = 2f;
            var extend = 2f;
            var bitmap = new Bitmap((int) ((maxX - minX) * extend) * (int) scale,
                (int) ((maxY - minY) * extend) * (int) scale);

            Func<Vector2, Vector2> getBMPos = _vec =>
            {
                var vx = (_vec.x - minX + extend / 4f) * scale;
                var vy = (_vec.y - minY + extend / 4f) * scale;
                return new Vector2(vx, vy);
            };

            //Draw Mesh
            foreach (var tri in triList)
                for (var idx = 0; idx < 3; ++idx)
                {
                    var edge = tri.Edge(idx);
                    var x1 = (edge.OriginPos.x - minX + extend / 4f) * scale;
                    var y1 = (edge.OriginPos.y - minY + extend / 4f) * scale;
                    var x2 = (edge.NextEdge.OriginPos.x - minX + extend / 4f) * scale;
                    var y2 = (edge.NextEdge.OriginPos.y - minY + extend / 4f) * scale;

                    using (var g = Graphics.FromImage(bitmap))
                    {
                        g.SmoothingMode = SmoothingMode.None;
                        g.InterpolationMode = InterpolationMode.Low;
                        g.PixelOffsetMode = PixelOffsetMode.None;
                        var pen = new Pen(m_ColorMesh);
                        g.DrawLine(pen, x1, y1, x2, y2);

                        if (!showVertIdxs) continue;
                        var textRect = new RectangleF(new PointF(x1, y1), new SizeF(60, 20));

                        var txtBrush = new SolidBrush(RandColor(rnd));


                        g.DrawString(edge.OriginIdx.ToString(), new Font("Small Fonts", fontSize), txtBrush,
                            textRect);
                    }
                }

            //Draw CircumCircles
            /*
            foreach (var tri in triList)
            {
                var ccos = getBMPos(new Vector2f(tri.CCX, tri.CCY));
                var r = (float) Math.Sqrt(tri.CCRSq) * scale;
                var rul = new Vector2f(ccos.x - r, ccos.y - r);
                var rect = new Rectangle((int)rul.x, (int)rul.y, (int)(2 * r), (int)(2 * r));
                using (var g = Graphics.FromImage(bitmap))
                {
                    
                    
                    g.SmoothingMode = SmoothingMode.None;
                    g.InterpolationMode = InterpolationMode.Low;
                    g.PixelOffsetMode = PixelOffsetMode.None;

                    var pen = new Pen(m_ColorCircles);
                    g.DrawEllipse(pen, (ccos.x - r), (ccos.y - r), r * 2, r * 2);
                    //g.DrawRectangle(pen, rect);
                    //g.DrawLine(pen, center.x, center.y, rect.Left, rect.Top);
                }
            }
            */

            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".png";
            bitmap.Save(path);

            //Log Tris
            //LogTri(_fileName + ".txt", triList.ToArray());
        }

        public static void Visualize(Delaunay _d, Voronoi _v = null, string _fileName = "debugMesh")
        {
            if (!Enabled) return;
            //var bitmap = new Bitmap((int) (_d.BoundsRect.width * 1.2f), (int) (_d.BoundsRect.height * 1.2f));
            var bitmap = new Bitmap(2000, 2000);

            var originOffset = _d.BoundsRect.min;
            originOffset.x += 0 - bitmap.Width / 2f;
            originOffset.y += 0 - bitmap.Height / 2f;
            if (!float.IsInfinity(OriginOffsetOverride.x))
            {
                originOffset = _d.BoundsRect.min;
                originOffset.x += OriginOffsetOverride.x - bitmap.Width / 2f;
                originOffset.y += OriginOffsetOverride.y - bitmap.Height / 2f;
            }

            //Draw Mesh
            var tris = _d.Triangles();
            foreach (var tri in tris)
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
            /*
            var cs = (CircleSweep) _d.triangulator;
            var fPt = cs.frontier.LastAddedFPt;
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
            */


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
            /*
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
            */

            //Voronoi on top
            if (_v != null)
            {
                var sites = _v.SitesByDIdx.Values;
                foreach (var site in sites)
                {
                    var edges = site.Edges;
                    for (var idx = 0; idx < edges.Count; ++idx)
                    {
                        if (idx == edges.Count - 1 && !site.Closed) break;
                        var edge = edges[idx];
                        var x1 = edge.Origin.x - originOffset.x;
                        var y1 = edge.Origin.y - originOffset.y;
                        var x2 = edge.NextEdge.Origin.x - originOffset.x;
                        var y2 = edge.NextEdge.Origin.y - originOffset.y;

                        using (var g = Graphics.FromImage(bitmap))
                        {
                            g.SmoothingMode = SmoothingMode.None;
                            g.InterpolationMode = InterpolationMode.Low;
                            g.PixelOffsetMode = PixelOffsetMode.None;
                            var pen = new Pen(m_ColorVor);
                            g.DrawLine(pen, x1, y1, x2, y2);

                        }
                    }
                }
                    
            }

            //Draw Voronoi Bounds Rect
            if (_v != null)
            {
                var dBnds = new Rect(_v.Bounds);
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
            /*
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
            */

            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".png";
            bitmap.Save(path);
        }

        public static float ToDeg(float _radians)
        {
            return _radians * 180f / (float) Math.PI;
        }
        
        public static void LogToFile(string _fileName, string _text)
        {
            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".txt";
            using (var fs = File.Create(path))
            {
                // writing data in string
                var info = new UTF8Encoding(true).GetBytes(_text);
                fs.Write(info, 0, info.Length);

                // writing data in bytes already
                //byte[] data = new byte[] { 0x0 };
                //fs.Write(data, 0, data.Length);
            }
        }
        
        public static void LogTri(string _fileName, Delaunay.Triangle[] _tris, string _logAdder = "")
        {
            if (!Enabled) return;
            var sb = new StringBuilder();
            for (var tIdx = 0; tIdx < _tris.Length; ++tIdx)
            {
                var tri = _tris[tIdx];
                sb.AppendLine("Tri " + tIdx);
                sb.AppendLine(" CC = ( " + tri.CCX + " , " + tri.CCY + " )");
                sb.AppendLine(" Rad = " + (float) Math.Sqrt(tri.CCRSq) + " : " + tri.CCRSq);

                var edges = new[] {tri.Edge0, tri.Edge1, tri.Edge2};
                for (var eIdx = 0; eIdx < 3; ++eIdx)
                {
                    var edge = edges[eIdx];
                    sb.AppendLine(" Edge " + eIdx + "  " + edge.OriginIdx + "  " + edge.OriginPos);
                    var v0 = edge.OriginPos;
                    var v1 = edge.NextEdge.OriginPos;
                    var v2 = edge.NextEdge.NextEdge.OriginPos;
                    var vecRt = v1 - v0;
                    var vecLt = v2 - v0;
                    var angle = vecRt.AngleCW(vecLt);
                    sb.AppendLine(" Angle: " + angle + " : " + ToDeg(angle));
                    sb.AppendLine();
                }
            }

            sb.AppendLine("Adder: " + _logAdder);
            sb.AppendLine();

            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".txt";
            using (var fs = File.Create(path))
            {
                // writing data in string
                var info = new UTF8Encoding(true).GetBytes(sb.ToString());
                fs.Write(info, 0, info.Length);

                // writing data in bytes already
                //byte[] data = new byte[] { 0x0 };
                //fs.Write(data, 0, data.Length);
            }
        }

        public static Color RandColor(Random _r)
        {
            var r = (int) ((0.2f + _r.NextDouble() * 0.8f) * 255);
            var g = (int) ((0.2f + _r.NextDouble() * 0.8f) * 255);
            var b = (int) ((0.2f + _r.NextDouble() * 0.8f) * 255);
            return Color.FromArgb(r, g, b);
        }

        public static Bitmap CropImage(Bitmap source, Rectangle section)
        {
            // An empty bitmap which will hold the cropped image
            var bmp = new Bitmap(section.Width, section.Height);

            var g = Graphics.FromImage(bmp);

            // Draw the given area (section) of the source image
            // at location 0,0 on the empty bitmap (bmp)
            g.DrawImage(source, 0, 0, section, GraphicsUnit.Pixel);

            return bmp;
        }
    }
}