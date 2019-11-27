namespace ioDelaunay
{
    using System;


  public struct Vector2
    {
        private static readonly Vector2 zeroVector = new Vector2(0.0f, 0.0f);
        private static readonly Vector2 oneVector = new Vector2(1f, 1f);
        private static readonly Vector2 upVector = new Vector2(0.0f, 1f);
        private static readonly Vector2 downVector = new Vector2(0.0f, -1f);
        private static readonly Vector2 leftVector = new Vector2(-1f, 0.0f);
        private static readonly Vector2 rightVector = new Vector2(1f, 0.0f);

        private static readonly Vector2 positiveInfinityVector =
            new Vector2(float.PositiveInfinity, float.PositiveInfinity);

        private static readonly Vector2 negativeInfinityVector =
            new Vector2(float.NegativeInfinity, float.NegativeInfinity);

      
      public float x;

      public float y;

      public Vector2(float x, float y)
        {
            this.x = x;
            this.y = y;
        }

      public void Set(float newX, float newY)
        {
            x = newX;
            y = newY;
        }

      public void Normalize()
        {
            var magnitude = this.magnitude;
            if (magnitude > 9.99999974737875E-06)
                this = this / magnitude;
            else
                this = zero;
        }

      public Vector2 normalized
        {
            get
            {
                var vector2f = new Vector2(x, y);
                vector2f.Normalize();
                return vector2f;
            }
        }

      public override string ToString()
        {
            return string.Format("({0:F1}, {1:F1})", x, y);
        }

      public string ToString(string format)
        {
            return string.Format("({0}, {1})", x.ToString(format), y.ToString(format));
        }

        public override int GetHashCode()
        {
            return x.GetHashCode() ^ (y.GetHashCode() << 2);
        }

      public override bool Equals(object other)
        {
            if (!(other is Vector2))
                return false;
            var vector2f = (Vector2) other;
            return x.Equals(vector2f.x) && y.Equals(vector2f.y);
        }


      public float magnitude
        {
            get { return (float) Math.Sqrt((float) (x * (double) x + y * (double) y)); }
        }
      public float sqrMagnitude
        {
            get { return (float) (x * (double) x + y * (double) y); }
        }

        public static Vector2 operator +(Vector2 a, Vector2 b)
        {
            return new Vector2(a.x + b.x, a.y + b.y);
        }

        public static Vector2 operator -(Vector2 a, Vector2 b)
        {
            return new Vector2(a.x - b.x, a.y - b.y);
        }

        public static Vector2 operator -(Vector2 a)
        {
            return new Vector2(-a.x, -a.y);
        }

        public static Vector2 operator *(Vector2 a, float d)
        {
            return new Vector2(a.x * d, a.y * d);
        }

        public static Vector2 operator *(float d, Vector2 a)
        {
            return new Vector2(a.x * d, a.y * d);
        }

        public static Vector2 operator /(Vector2 a, float d)
        {
            return new Vector2(a.x / d, a.y / d);
        }

        public static bool operator ==(Vector2 lhs, Vector2 rhs)
        {
            return (lhs - rhs).sqrMagnitude < 9.99999943962493E-11;
        }

        public static bool operator !=(Vector2 lhs, Vector2 rhs)
        {
            return !(lhs == rhs);
        }


      /// <summary>
      ///     <para>Shorthand for writing Vector2f(0, 0).</para>
      /// </summary>
      public static Vector2 zero
        {
            get { return zeroVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(1, 1).</para>
      /// </summary>
      public static Vector2 one
        {
            get { return oneVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(0, 1).</para>
      /// </summary>
      public static Vector2 up
        {
            get { return upVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(0, -1).</para>
      /// </summary>
      public static Vector2 down
        {
            get { return downVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(-1, 0).</para>
      /// </summary>
      public static Vector2 left
        {
            get { return leftVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(1, 0).</para>
      /// </summary>
      public static Vector2 right
        {
            get { return rightVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(float.PositiveInfinity, float.PositiveInfinity).</para>
      /// </summary>
      public static Vector2 positiveInfinity
        {
            get { return positiveInfinityVector; }
        }

      /// <summary>
      ///     <para>Shorthand for writing Vector2f(float.NegativeInfinity, float.NegativeInfinity).</para>
      /// </summary>
      public static Vector2 negativeInfinity
        {
            get { return negativeInfinityVector; }
        }

    }

}