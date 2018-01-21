using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Numerics;

namespace PrintOptimizer
{
    public partial class Form1 : Form
    {
        Gcode code;
        Layer currentLayer;

        public Form1()
        {
            InitializeComponent();
        }

        private void Form1_DragEnter(object sender, DragEventArgs e)
        {
            if (e.Data.GetDataPresent(DataFormats.FileDrop))
            {
                e.Effect = DragDropEffects.Copy;
            }
        }

        private void Form1_DragDrop(object sender, DragEventArgs e)
        {
            string[] files = (string[])e.Data.GetData(DataFormats.FileDrop);
            if (files.Length > 1)
            {
                MessageBox.Show(
                    "Maximal eine Datei erlaubt", 
                    "Oh nooooo", 
                    MessageBoxButtons.OK, 
                    MessageBoxIcon.Error, 
                    MessageBoxDefaultButton.Button1);
                return;
            }
            code = loadFile(files[0]);
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            code = null;
            richTextBox1.Text = "Please Drop a G-Code File here...";
        }

        private Gcode loadFile(String path)
        {
            FileInfo fi = new FileInfo(path);
            if (!fi.Exists)
            {
                MessageBox.Show(
                    "Es scheint als würde die ausgewählte Datei nicht existieren ...",
                    "Oh nooooo",
                    MessageBoxButtons.OK,
                    MessageBoxIcon.Error,
                    MessageBoxDefaultButton.Button1);
                return null;
            }

            richTextBox1.Text = "Datei: " + fi.Name + "\n";
            richTextBox1.Text += "Größe: " + fi.Length + "bytes\n";
            richTextBox1.Text += "\n";

            Gcode code = new Gcode(path);
            richTextBox1.Text += "Zeilen: " + code.LinesRead + "\n";
            richTextBox1.Text += "Befehle: " + code.CommandsRead + "\n";
            richTextBox1.Text += "Ebenen: " + code.LayerCount + "\n";
            richTextBox1.Text += "\n";

            float total = code.Layers.Sum(l => l.TotalDuration);
            float duration = code.Layers.Sum(l => l.TransitionDuration);
            float optimized = code.Layers.Sum(l => l.OptimizedLayer.TransitionDuration);
            richTextBox1.Text += "Totale Dauer: " + total.ToString("N2") + "s\n";
            richTextBox1.Text += "Übergänge: " + duration.ToString("N2") + "s (" + (duration / total * 100).ToString("N2") + "%)\n";
            richTextBox1.Text += "Optimiert: " + optimized.ToString("N2") + "s\n";
            richTextBox1.Text += "Relativ: " + ((optimized / duration * 100) - 100).ToString("N2") + "%\n";
            richTextBox1.Text += "\n";

            vScrollBar1.Maximum = code.LayerCount + (vScrollBar1.LargeChange - 2);
            vScrollBar1.Value = code.LayerCount;

            return code;
        }

        private void vScrollBar1_Scroll(object sender, ScrollEventArgs e)
        {
            if (code == null)
            {
                return;
            }
            int layerIndex = (vScrollBar1.Maximum - (vScrollBar1.LargeChange - 2) - vScrollBar1.Value);
            this.Text = "Print-Optimizer";
            
            if (layerIndex >= this.code.Layers.Count)
            {
                return;
            }

            this.currentLayer = this.code.Layers[layerIndex];
            writeStats();
        }

        private void writeStats()
        {
            resetToLine(12);

            richTextBox1.Text += "Layer: " + this.code.Layers.IndexOf(this.currentLayer) + "\n";

            richTextBox1.Text += "Segmente: " + this.currentLayer.Segments.Count + "\n";
            richTextBox1.Text += "Übergänge: " + this.currentLayer.TransitionSegments.Count + "\n";
            richTextBox1.Text += "Totale Länge: " + this.currentLayer.TotalLength.ToString("N2") + "mm\n";
            richTextBox1.Text += "Dauer: " + this.currentLayer.TotalDuration.ToString("N2") + "s\n";
            richTextBox1.Text += "  Übergänge: " + this.currentLayer.TransitionDuration.ToString("N2") + "s (" + (this.currentLayer.TransitionDurationRelative * 100).ToString("N2") + "%)\n";

            richTextBox1.Text += "\n";
            richTextBox1.Text += "Optimiert (" + ((this.currentLayer.OptimizedLayer.TotalDuration / this.currentLayer.TotalDuration * 100) - 100).ToString("N2") + "):\n";
            richTextBox1.Text += "  Totale Länge: " + this.currentLayer.OptimizedLayer.TotalLength.ToString("N2") + "mm\n";
            richTextBox1.Text += "  Dauer: " + this.currentLayer.OptimizedLayer.TotalDuration.ToString("N2") + "s\n";
            richTextBox1.Text += "    Übergänge: " + this.currentLayer.OptimizedLayer.TransitionDuration.ToString("N2") + "s (" + (this.currentLayer.OptimizedLayer.TransitionDurationRelative * 100).ToString("N2") + "%)\n";
            draw();
        }

        private void draw()
        {
            if (this.currentLayer == null || this.currentLayer.Segments.Count == 0)
            {
                return;
            }

            Layer layer = this.currentLayer;
            if (cb_autoOptimize.Checked)
            {
                cb_optimize.Checked = this.currentLayer.getFasterLayer() == this.currentLayer.OptimizedLayer;
            }
            if (cb_optimize.Checked)
            {
                layer = layer.OptimizedLayer;
            }

            Bitmap bmp = new Bitmap(2200, 2200);
            using (Graphics g = Graphics.FromImage(bmp))
            {
                g.Clear(Color.White);
                Pen mypen1 = new Pen(Color.Black);
                Pen mypen2 = new Pen(Color.Black);
                Pen mypen3 = new Pen(Color.Red);
                Pen mypen4 = new Pen(Color.Red);
                int i = 0;
                foreach (Segment seg in layer.PrintSegments)
                {
                    if (!rb_full.Checked)
                    {
                        g.DrawLine((i++ % 2 == 0) ? mypen1 : mypen2, vector2point(seg.Start, 10), vector2point(seg.End, 10));
                    } else
                    {
                        foreach (Line line in seg.Lines) {
                            g.DrawLine((i++ % 2 == 0) ? mypen1 : mypen2, vector2point(line.Start, 10), vector2point(line.End, 10));
                        }
                    }
                }
                if (cb_transitions.Checked)
                {
                    foreach (Segment seg in layer.TransitionSegments)
                    {
                        if (!rb_full.Checked)
                        {
                            g.DrawLine((i++ % 2 == 0) ? mypen3 : mypen4, vector2point(seg.Start, 10), vector2point(seg.End, 10));
                        } else
                        {
                            foreach (Line line in seg.Lines)
                            {
                                g.DrawLine((i++ % 2 == 0) ? mypen3 : mypen4, vector2point(line.Start, 10), vector2point(line.End, 10));
                            }
                        }
                    }
                }
            }
            pictureBox1.Image = bmp;
        }

        private PointF vector2point(Vector2 vector, float scale)
        {
            return new PointF(vector.X * scale, vector.Y * scale);
        }

        private void resetToLine(int line)
        {
            string text = richTextBox1.Text;

            int lineCount = text.Split('\n').Length;
            if (lineCount <= line)
            {
                return;
            }

            Dictionary<int,int> lineStartIndices = new Dictionary<int, int>();
            for (int l = 1; l < lineCount; l++)
            {
                int searchStart = (lineStartIndices.Count > 0) ? lineStartIndices[l - 1] + 1 : 0;
                int lineStart = text.IndexOf('\n', searchStart);

                lineStartIndices.Add(l, lineStart);
            }

            richTextBox1.Text = text.Substring(0, lineStartIndices[line] + 1);
        }

        private void rb_full_CheckedChanged(object sender, EventArgs e)
        {
            draw();
        }

        private void cb_transitions_CheckedChanged(object sender, EventArgs e)
        {
            draw();
        }

        private void bt_optimize_Click(object sender, EventArgs e)
        {
            draw();
        }

        private void cb_optimize_CheckedChanged(object sender, EventArgs e)
        {
            draw();
        }

        private void cb_autoOptimize_CheckedChanged(object sender, EventArgs e)
        {
            cb_optimize.Enabled = !cb_autoOptimize.Checked;
        }

        private void richTextBox1_MouseDoubleClick(object sender, MouseEventArgs e)
        {
            List<KeyValuePair<float, Layer>> gain = this.code.Layers
                .Select<Layer, KeyValuePair<float, Layer>>
                    (l => new KeyValuePair<float, Layer>(
                        l.OptimizedLayer.TotalDuration / l.TotalDuration
                        , l))
                .OrderBy(p => p.Key).ToList();

           this.currentLayer = gain.First().Value;
            writeStats();
            draw();
        }

        private void Form1_KeyDown(object sender, KeyEventArgs e)
        {
            if (e.Control && e.KeyCode == Keys.S)
            {
                string name = this.code.Name;
                int layer = this.code.Layers.IndexOf(this.currentLayer);

                string style = (rb_full.Checked) ? "Full" : "Abstract";
                string render = ((cb_optimize.Checked) ? "-Optimized" : "")
                    + ((cb_transitions.Checked) ? "-Transitions" : "-NoTransitions");

                Crop(new Bitmap(pictureBox1.Image)).Save(name + "-" + layer + "-" + style + render + ".png", System.Drawing.Imaging.ImageFormat.Png);
            }
        }
        public static Bitmap Crop(Bitmap bmp)
        {
            int w = bmp.Width;
            int h = bmp.Height;

            Func<int, bool> allWhiteRow = row =>
            {
                for (int i = 0; i < w; ++i)
                    if (bmp.GetPixel(i, row).R != 255)
                        return false;
                return true;
            };

            Func<int, bool> allWhiteColumn = col =>
            {
                for (int i = 0; i < h; ++i)
                    if (bmp.GetPixel(col, i).R != 255)
                        return false;
                return true;
            };

            int topmost = 0;
            for (int row = 0; row < h; ++row)
            {
                if (allWhiteRow(row))
                    topmost = row;
                else break;
            }

            int bottommost = 0;
            for (int row = h - 1; row >= 0; --row)
            {
                if (allWhiteRow(row))
                    bottommost = row;
                else break;
            }

            int leftmost = 0, rightmost = 0;
            for (int col = 0; col < w; ++col)
            {
                if (allWhiteColumn(col))
                    leftmost = col;
                else
                    break;
            }

            for (int col = w - 1; col >= 0; --col)
            {
                if (allWhiteColumn(col))
                    rightmost = col;
                else
                    break;
            }

            if (rightmost == 0) rightmost = w; // As reached left
            if (bottommost == 0) bottommost = h; // As reached top.

            int croppedWidth = rightmost - leftmost;
            int croppedHeight = bottommost - topmost;

            if (croppedWidth == 0) // No border on left or right
            {
                leftmost = 0;
                croppedWidth = w;
            }

            if (croppedHeight == 0) // No border on top or bottom
            {
                topmost = 0;
                croppedHeight = h;
            }

            try
            {
                var target = new Bitmap(croppedWidth, croppedHeight);
                using (Graphics g = Graphics.FromImage(target))
                {
                    g.DrawImage(bmp,
                      new RectangleF(0, 0, croppedWidth, croppedHeight),
                      new RectangleF(leftmost, topmost, croppedWidth, croppedHeight),
                      GraphicsUnit.Pixel);
                }
                return target;
            }
            catch (Exception ex)
            {
                throw new Exception(
                  string.Format("Values are topmost={0} btm={1} left={2} right={3} croppedWidth={4} croppedHeight={5}", topmost, bottommost, leftmost, rightmost, croppedWidth, croppedHeight),
                  ex);
            }
        }

    }
}
