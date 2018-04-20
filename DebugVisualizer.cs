using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;

namespace ioDelaunay
{
    public static class DebugVisualizer
    {
        public static int fontSize = 6;
        public static bool showVertIdxs = false;
        private static readonly Color m_ColorMesh = Color.White;
        private static readonly Color m_ColorFront = Color.Red;
        private static Color m_ColorFont = Color.BurlyWood;

        public static void Visualize(Delaunay _d, string _fileName = "debugMesh")
        {
            var bitmap = new Bitmap((int) (_d.BoundsRect.width * 1.2f), (int) (_d.BoundsRect.height * 1.2f));
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
            var fPt = cs.DebugFrontier.Project(0)[0];
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

            var path = AppDomain.CurrentDomain.BaseDirectory + _fileName + ".bmp";
            bitmap.Save(path);
        }
    }
}