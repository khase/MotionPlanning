using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Threading.Tasks;

namespace PrintOptimizer
{
    class Gcode
    {
        private string file;
        private List<Layer> layers;
        private int linesRead;
        private int commandsRead;

        public string Name
        {
            get
            {
                return this.file;
            }
        }
        public int LinesRead
        {
            get
            {
                return linesRead;
            }
        }
        public int CommandsRead
        {
            get
            {
                return commandsRead;
            }
        }
        public int LayerCount
        {
            get
            {
                return layers.Count;
            }
        }
        public List<Layer> Layers
        {
            get
            {
                return this.layers;
            }
        }

        public Gcode()
        {
            this.linesRead = 0;
            this.commandsRead = 0;
            this.layers = new List<Layer>();
        }
        public Gcode(string Path): this()
        {
            this.layers = load(Path);
        }

        private List<Layer> load(string path)
        {
            Line lastLine = null;
            List<Layer> planes = new List<Layer>();
            Vector2 lastOptimizedEnd = new Vector2(0, 0);
            this.file = new FileInfo(path).Name;
            using (StreamReader sr = new StreamReader(path))
            {
                List<Segment> segments = new List<Segment>();
                List<Line> lines = new List<Line>();
                bool lastType = false;
                while (!sr.EndOfStream)
                {
                    String commandLine = sr.ReadLine();
                    linesRead++;
                    if (commandLine.Trim().StartsWith(";"))
                    {
                        // This is a comment
                        continue;
                    }
                    if (commandLine.Trim().Length == 0)
                    {
                        // This line is empty
                        continue;
                    }

                    // This line has some sort of command
                    commandsRead++;

                    int its = commandLine.IndexOf(";");
                    if (its != -1)
                    {
                        // This line has an inline comment
                        // Omit comment
                        commandLine = commandLine.Substring(0, its);
                    }

                    if (commandLine.StartsWith("G1"))
                    {
                        if (commandLine.StartsWith("G1 Z") && segments.Count > 0)
                        {
                            // This is the indicator for a new layer
                            Layer l = new Layer(segments);
                            lastOptimizedEnd = l.Optimize(lastOptimizedEnd).End;
                            planes.Add(l);
                            segments = new List<Segment>();
                        }
                        Line line = GetSegment(lastLine, commandLine);
                        if (line.Start != Line.vUndefined && line.End != Line.vUndefined)
                        {
                            if (lines.Count() > 0 && line.IsTransition != lastType)
                            {
                                segments.Add(new Segment(lines, lastType));
                                lines = new List<Line>();
                            }
                            lines.Add(line);
                            lastType = line.IsTransition;
                        }
                        lastLine = line;
                    }
                }
                if (segments.Count > 0)
                {
                    Layer layer = new Layer(segments);
                    lastOptimizedEnd = layer.Optimize(lastOptimizedEnd).End;
                    planes.Add(layer);
                }
            }
            return planes;
        }

        private Line GetSegment(Line lastLine, string command)
        {
            string[] split = command.Trim().Split(' ');
            Vector2 end = Line.vUndefined;
            bool isTransition = true;
            float speed = (lastLine != null) ? lastLine.Speed : 40;

            // skip the first one since this is "G1"
            for (int i = 1; i < split.Length; i++)
            {
                string arg = split[i];
                switch (arg.ElementAt(0))
                {
                    case 'X':
                        end.X = float.Parse(arg.Substring(1)) / 1000;
                        break;
                    case 'Y':
                        end.Y = float.Parse(arg.Substring(1)) / 1000;
                        break;
                    case 'Z':
                        break;
                    case 'E':
                        isTransition = false;
                        break;
                    case 'F':
                        // gcode uses mm/minute
                        speed = float.Parse(arg.Substring(1)) / 60;
                        break;
                    default:
                        Console.WriteLine("Unknown argument: " + arg);
                        break;
                }
            }

            
            return new Line((lastLine != null) ? lastLine.End : Line.vUndefined, end, speed, isTransition);
        }
    }
}
