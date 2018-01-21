using System;
using System.Collections.Generic;
using System.Drawing;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace PrintOptimizer
{
    class Line
    {
        public static Vector2 vUndefined = new Vector2(999999, 999999);
        //public static float printSpeed = 40;
        //public static float transitionSpeed = printSpeed * 2;

        private Vector2 start;
        private Vector2 end;
        private bool transition;
        private float speed;

        public Vector2 Start
        {
            get
            {
                return this.start;
            }
        }
        public Vector2 End
        {
            get
            {
                return this.end;
            }
        }
        public bool IsTransition
        {
            get
            {
                return this.transition;
            }
        }
        public float Speed
        {
            get
            {
                return this.speed;
            }
        }

        public float Length
        {
            get
            {
                return (this.start - this.end).Length();
            }
        }

        public float Duration
        {
            get
            {
                //return this.Length / ((IsTransition) ? transitionSpeed : printSpeed);
                return this.Length / this.speed;
            }
        }

        public Line()
        {
            this.start = vUndefined;
            this.end = vUndefined;
            this.transition = false;
        }
        public Line(Vector2 start, Vector2 end, float speed) : this()
        {
            this.start = start;
            this.end = end;
            this.speed = speed;
        }
        public Line(Vector2 start, Vector2 end, float speed, bool transition) : this(start, end, speed)
        {
            this.transition = transition;
        }

        public void Reverse()
        {
            Vector2 tmp = this.start;
            this.start = this.end;
            this.end = tmp;
        }
    }
}
