using System;
using System.Collections.Generic;
using System.Drawing.Drawing2D;

namespace ioDelaunay
{
    using System.Drawing;
    public static class DebugVisualizer
    {
        private static Color m_ColorMesh = Color.White;
        private static Color m_ColorFront = Color.Red;
        private static Color m_ColorFont = Color.BurlyWood;
        public static void Visualize(CircleSweep _cs, string _fileName = "debugMesh")
        {
            var bitmap = new Bitmap((int)_cs.Bounds.width, (int)_cs.Bounds.height);
            var originOffset = _cs.Bounds.min;
            var vectTextDrawn = new HashSet<int>();

            
            //Draw Mesh
            foreach (var tri in _cs.Triangles)
            {
                for (int idx = 0; idx < 3; ++idx)
                {
                    var edge = tri.HalfEdge(idx);
                    var x1 = edge.Origin.Pos.x - originOffset.x;
                    var y1 = edge.Origin.Pos.y - originOffset.y;
                    var x2 = edge.NextEdge.Origin.Pos.x - originOffset.x;
                    var y2 = edge.NextEdge.Origin.Pos.y - originOffset.y;

                    using (var g = Graphics.FromImage(bitmap))
                    {
                        var pen = new Pen(m_ColorMesh);
                        g.DrawLine(pen, x1, y1, x2, y2);
                        
                        var textRect = new RectangleF(new PointF(x1, y1), new SizeF(30,20) );
                        
                        g.SmoothingMode = SmoothingMode.AntiAlias;
                        g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                        g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                        g.DrawString(edge.OriginIdx.ToString(), new Font("Tahoma",8), Brushes.BurlyWood, textRect);
                    }
                }
            }
            
            //Draw Frontier
            foreach (var fpt in _cs.DebugFrontier.FrontierPts)
            {
                var x1 = fpt.Vert.Pos.x - originOffset.x;
                var y1 = fpt.Vert.Pos.y - originOffset.y;
                var x2 = fpt.Right.Vert.Pos.x - originOffset.x;
                var y2 = fpt.Right.Vert.Pos.y - originOffset.y;

                using (var g = Graphics.FromImage(bitmap))
                {
                    g.SmoothingMode = SmoothingMode.AntiAlias;
                    g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                    g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                    var pen = new Pen(m_ColorFront);
                    g.DrawLine(pen, x1, y1, x2, y2);
                }
            }
            
            //Draw Origin
            using (var g = Graphics.FromImage(bitmap))
            {    
                g.SmoothingMode = SmoothingMode.AntiAlias;
                g.InterpolationMode = InterpolationMode.HighQualityBicubic;
                g.PixelOffsetMode = PixelOffsetMode.HighQuality;
                var pen = new Pen(m_ColorFront);
                var x1 = _cs.Origin.x - originOffset.x;
                var y1 = _cs.Origin.y - originOffset.y;
                g.FillRectangle(Brushes.Cyan, x1, y1, 2, 2);
                
            }

            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".bmp";
            bitmap.Save(path);
        }
    }
}