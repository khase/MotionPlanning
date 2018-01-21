using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PrintOptimizer
{
    class Graph
    {
        private Segment[] nodes;
        private int[,] arcs;

        private Graph(int size)
        {
            this.nodes = new Segment[size];
            this.arcs = new int[size, size];
        }
        public Graph(List<Segment> nodes): this(nodes.Count)
        {
            this.nodes = nodes.ToArray();
        }
        
        public void findConnections()
        {

        }
        
    }
}
