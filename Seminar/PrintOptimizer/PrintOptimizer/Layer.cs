using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;

namespace PrintOptimizer
{
    class Layer
    {
        private List<Segment> segments;
        private Layer optimizedLayer;

        public Layer OptimizedLayer
        {
            get
            {
                if (optimizedLayer == null)
                {
                    throw new Exception("optimized Layer wasn't calculated");
                }
                return optimizedLayer;
            }
        }
        public Vector2 Start
        {
            get
            {
                return segments.First().Start;
            }
        }
        public Vector2 End
        {
            get
            {
                return segments.Last().End;
            }
        }
        public List<Segment> Segments
        {
            get
            {
                return this.segments;
            }
        }
        public List<Segment> PrintSegments
        {
            get
            {
                return this.segments.Where(s => !s.IsTransition).ToList();
            }
        }
        public List<Segment> TransitionSegments
        {
            get
            {
                return this.segments.Where(s => s.IsTransition).ToList();
            }
        }

        public float PrintLength
        {
            get
            {
                return this.PrintSegments.Sum(s => s.Length);
            }
        }
        public float TransitionLength
        {
            get
            {
                return this.TransitionSegments.Sum(s => s.Length);
            }
        }
        public float TotalLength
        {
            get
            {
                return this.Segments.Sum(s => s.Length);
            }
        }

        public float PrintDuration
        {
            get
            {
                return this.PrintSegments.Sum(s => s.Duration);
            }
        }
        public float TransitionDuration
        {
            get
            {
                return this.TransitionSegments.Sum(s => s.Duration);
            }
        }
        public float TotalDuration
        {
            get
            {
                return this.Segments.Sum(s => s.Duration);
            }
        }

        public float PrintDurationRelative
        {
            get
            {
                return this.PrintDuration / this.TotalDuration;
            }
        }
        public float TransitionDurationRelative
        {
            get
            {
                return this.TransitionDuration / this.TotalDuration;
            }
        }

        public Layer()
        {
            this.segments = new List<Segment>();
        }
        public Layer(List<Segment> segments) : this()
        {
            this.segments = segments;
        }

        public Layer Optimize(Vector2 entryPoint)
        {
            List<Segment> pool = this.segments.Where(s => !s.IsTransition).ToList();
            float transitionSpeed = this.TransitionSegments.Max(s => s.MaxSpeed);

            Vector2 pos = entryPoint;

            List<Segment> newPrint = new List<Segment>();
            // do optimization logic until all Segments are covered
            do
            {
                Segment nextSegment = findNearest(pos, pool);
                pool.Remove(nextSegment);
                if ((nextSegment.Start - pos).Length() > (nextSegment.End - pos).Length())
                {
                    nextSegment.Reverse();
                }
                Segment transition = getTransitionSegment(pos, nextSegment, transitionSpeed);

                newPrint.Add(transition);
                newPrint.Add(nextSegment);

                pos = nextSegment.End;
            } while (pool.Count > 0);

            this.optimizedLayer = new Layer(newPrint);
            return this.optimizedLayer;
        }

        private Segment findNearest(Vector2 target, List<Segment> source)
        {
            List<KeyValuePair<float, Segment>> distances = source
                .Select<Segment, KeyValuePair<float, Segment>>
                    (s => new KeyValuePair<float, Segment>(
                        Math.Min(
                            (s.Start - target).Length(), 
                            (s.End - target).Length())
                        , s))
                .OrderBy(p => p.Key).ToList();

            KeyValuePair<float, Segment> nearest = distances.First();
            return nearest.Value;
        }

        private Segment getTransitionSegment(Vector2 position, Segment target, float speed)
        {
            List<Line> lines = new List<Line>();
            lines.Add(new Line(position, target.Start, speed, true));
            return new Segment(lines, true);
        }

        public Layer getFasterLayer()
        {
            if (this.TotalDuration < this.optimizedLayer.TotalDuration)
            {
                return this;
            }
            return this.optimizedLayer;
        }
    }
}
