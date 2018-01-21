using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Numerics;

namespace PrintOptimizer
{
    /// <summary>
    /// A Segment is a group of continuous lines of the same type (isTransition).
    /// </summary>
    class Segment
    {
        private List<Line> lines;
        private bool transition;

        public List<Line> Lines
        {
            get
            {
                return this.lines;
            }
        }

        public bool IsTransition
        {
            get
            {
                return this.transition;
            }
        }
        
        public float Length
        {
            get
            {
                return this.Lines.Sum(s => s.Length);
            }
        }
        
        public float Duration
        {
            get
            {
                return this.Lines.Sum(s => s.Duration);
            }
        }

        public Vector2 Start
        {
            get
            {
                return this.Lines.First().Start;
            }
        }

        public Vector2 End
        {
            get
            {
                return this.Lines.Last().End;
            }
        }

        public float AverageSpeed
        {
            get
            {
                return this.lines.Average(l => l.Speed);
            }
        }

        public float MaxSpeed
        {
            get
            {
                return this.lines.Max (l => l.Speed);
            }
        }

        public Segment()
        {
            this.lines = new List<Line>();
        }
        public Segment(List<Line> lines, bool transition) : this()
        {
            this.lines = lines;
            this.transition = transition;
        }

        public void Reverse()
        {
            foreach(Line line in this.lines)
            {
                line.Reverse();
            }
            lines.Reverse();
        }
    }
}
