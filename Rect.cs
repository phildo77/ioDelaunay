namespace ioDelaunay
{
    using System;

  public struct Rect
    {
        #region Fields

        private float m_Height;
        private float m_Width;
        private float m_XMin;
        private float m_YMin;

        #endregion Fields

        #region Constructors

      public Rect(float x, float y, float width, float height)
        {
            m_XMin = x;
            m_YMin = y;
            m_Width = width;
            m_Height = height;
        }

      public Rect(Vector2 position, Vector2 size)
        {
            m_XMin = position.x;
            m_YMin = position.y;
            m_Width = size.x;
            m_Height = size.y;
        }

      public Rect(Rect source)
        {
            m_XMin = source.m_XMin;
            m_YMin = source.m_YMin;
            m_Width = source.m_Width;
            m_Height = source.m_Height;
        }

        #endregion Constructors

        #region Properties

      public static Rect zero
        {
            get { return new Rect(0.0f, 0.0f, 0.0f, 0.0f); }
        }

      public Vector2 center
        {
            get { return new Vector2(x + m_Width / 2f, y + m_Height / 2f); }
            set
            {
                m_XMin = value.x - m_Width / 2f;
                m_YMin = value.y - m_Height / 2f;
            }
        }

      public float height
        {
            get { return m_Height; }
            set { m_Height = value; }
        }

      public Vector2 max
        {
            get { return new Vector2(xMax, yMax); }
            set
            {
                xMax = value.x;
                yMax = value.y;
            }
        }

      public Vector2 min
        {
            get { return new Vector2(xMin, yMin); }
            set
            {
                xMin = value.x;
                yMin = value.y;
            }
        }

      public Vector2 position
        {
            get { return new Vector2(m_XMin, m_YMin); }
            set
            {
                m_XMin = value.x;
                m_YMin = value.y;
            }
        }

      public Vector2 size
        {
            get { return new Vector2(m_Width, m_Height); }
            set
            {
                m_Width = value.x;
                m_Height = value.y;
            }
        }

      public float width
        {
            get { return m_Width; }
            set { m_Width = value; }
        }

      public float x
        {
            get { return m_XMin; }
            set { m_XMin = value; }
        }

      public float xMax
        {
            get { return m_Width + m_XMin; }
            set { m_Width = value - m_XMin; }
        }

      public float xMin
        {
            get { return m_XMin; }
            set
            {
                var xMax = this.xMax;
                m_XMin = value;
                m_Width = xMax - m_XMin;
            }
        }

      public float y
        {
            get { return m_YMin; }
            set { m_YMin = value; }
        }

      public float yMax
        {
            get { return m_Height + m_YMin; }
            set { m_Height = value - m_YMin; }
        }

      public float yMin
        {
            get { return m_YMin; }
            set
            {
                var yMax = this.yMax;
                m_YMin = value;
                m_Height = yMax - m_YMin;
            }
        }

        #endregion Properties

        #region Methods

        public static bool operator !=(Rect lhs, Rect rhs)
        {
            return !(lhs == rhs);
        }

        public static bool operator ==(Rect lhs, Rect rhs)
        {
            return lhs.x == (double) rhs.x && lhs.y == (double) rhs.y && lhs.width == (double) rhs.width &&
                   lhs.height == (double) rhs.height;
        }

      public bool Contains(Vector2 point)
        {
            return point.x >= (double) xMin && point.x < (double) xMax && point.y >= (double) yMin &&
                   point.y < (double) yMax;
        }


        public void Encapsulate(Vector2 _point)
        {
            if (_point.x > xMax)
                xMax = _point.x;
            if (_point.x < xMin)
                xMin = _point.x;
            if (_point.y > yMax)
                yMax = _point.y;
            if (_point.y < yMin)
                yMin = _point.y;
        }

        public override bool Equals(object other)
        {
            if (!(other is Rect))
                return false;
            var rect = (Rect) other;
            return x.Equals(rect.x) && y.Equals(rect.y) && width.Equals(rect.width) && height.Equals(rect.height);
        }

        public override int GetHashCode()
        {
            return x.GetHashCode() ^ (width.GetHashCode() << 2) ^ (y.GetHashCode() >> 2) ^ (height.GetHashCode() >> 1);
        }


      /// <summary>
      ///     <para>Returns a nicely formatted string for this Rectf.</para>
      /// </summary>
      /// <param name="format"></param>
      public override string ToString()
        {
            return string.Format("(x:{0:F2}, y:{1:F2}, width:{2:F2}, height:{3:F2})", (object) x, (object) y,
                (object) width, (object) height);
        }

      /// <summary>
      ///     <para>Returns a nicely formatted string for this Rectf.</para>
      /// </summary>
      /// <param name="format"></param>
      public string ToString(string format)
        {
            return string.Format("(x:{0}, y:{1}, width:{2}, height:{3})", (object) x.ToString(format),
                (object) y.ToString(format), (object) width.ToString(format), (object) height.ToString(format));
        }


        #endregion Methods

    }

}