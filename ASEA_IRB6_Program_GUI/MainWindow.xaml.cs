using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media.Imaging;
using System.Windows.Threading;
using Microsoft.Win32;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Util;
using System.Drawing;
using System.Drawing.Imaging;

namespace RobotGui
{
    public partial class MainWindow : Window
    {
       /* ---------- scaling constants ---------- */

        private readonly double _xMin = 0.75, _xMax = 0.91,
                                _yMin = -0.08, _yMax = 0.08;

        /* ---------- serial ---------- */
        private readonly SerialPort _serialPort;

       /* ---------- contours ---------- */
        private List<Contour> currentContours;
        private int _pointStep;
        private const int MaxCoords = 2390;

        /* ---------- bitmaps ---------- */
        private Bitmap _staticBmp;   // tło (pełny kontur)
        private Bitmap _simBmp;      // bieżąca klatka animacji

        /* ---------- animation ---------- */
        private readonly DispatcherTimer _simTimer;
        private List<Point3D> _simPoints;
        private int _simIndex;
        private const int SimFps = 60;

        /* -- Z constants ----- */
        private const double PenUpZ = 0.95;
        private const double ZTolerance = 1e-4;

        private double _z = 0.9343;
        private double _doubleEdgeThreshold = 0.95;
        private double Z => _z;

        /* ---------- last loaded image ---------- */
        private string _currentImagePath;
        private bool _currentRotate;

        // ====== trajectory in angles ======
        private List<double[]> _anglesSequence = new();

        // ====== robot geometry ======
        private const double D1 = 0.7;

        private const double L2 = 0.45;
        private const double L3 = 0.67;
        private const double D4 = 0.095;

        private static readonly double[] Theta2Table = { -45, -20, 0, 15, 32, 45 };
        private static readonly double[] Theta3MinTable = { -5, -34, -55, -65, -60, -70 };
        private static readonly double[] Theta3MaxTable = { 25, 30, 35, 20, 20, 3 };

        private static double DegToRad(double deg) => deg * Math.PI / 180.0;

        private static double RadToDeg(double rad) => rad * 180.0 / Math.PI;

        private static double Clamp(double v, double lo, double hi) =>
            v < lo ? lo : (v > hi ? hi : v);

        private static double GetTheta3Min(double theta2)
        {
            for (int i = 0; i < Theta2Table.Length - 1; i++)
                if (theta2 <= Theta2Table[i])
                    return Theta3MinTable[i];
            return Theta3MinTable[^1];
        }

        private static double GetTheta3Max(double theta2)
        {
            for (int i = 0; i < Theta2Table.Length - 1; i++)
                if (theta2 <= Theta2Table[i])
                    return Theta3MaxTable[i];
            return Theta3MaxTable[^1];
        }

        /* ---------- init ---------- */

        public MainWindow()
        {
            InitializeComponent();

            _serialPort = new SerialPort();
            _serialPort.DataReceived += SerialPort_DataReceived;
            _serialPort.NewLine = "\n";
            _serialPort.WriteTimeout = 5000;

            _simTimer = new DispatcherTimer
            {
                Interval = TimeSpan.FromMilliseconds(1000.0 / SimFps)
            };
            _simTimer.Tick += SimTimer_Tick;

            if (TxtZ != null)
                TxtZ.Text = _z.ToString("F4", CultureInfo.InvariantCulture);

            if (SldEdgeThreshold != null)
                SldEdgeThreshold.Value = _doubleEdgeThreshold;

            if (TxtEdgeValue != null)
                TxtEdgeValue.Text = _doubleEdgeThreshold.ToString("F2", CultureInfo.InvariantCulture);

            ApplyDensity(70);
        }

        /// <summary>
        /// Computes [theta1, theta2, theta3, theta4] (degrees)
        /// for a given point (X,Y,Z).
        ///
        /// Arduino-compatible version:
        /// acos clamp + FK check + theta3 sign flip.
        /// </summary>
        private double[] ComputeAnglesForPoint(Point3D p)
        {
            double px = p.X;
            double py = p.Y;
            double pz = p.Z;

            double Alpha = 1.57;  
            double Gamma = 1.579; 

            double theta1Rad = Math.Atan2(py, px);
            double r = Math.Sqrt(px * px + py * py);

            double a = D4 * Math.Sin(Alpha);
            double b = D4 * Math.Cos(Alpha);

            double pzModified = pz - D1;
            double c = pzModified - b;
            double d = r - a;
            double e = Math.Sqrt(c * c + d * d);

            if (e < 1e-12)
                throw new InvalidOperationException("Singular point / out of range (e ~ 0).");

            double betaRad = Math.Acos(Clamp(d / e, -1.0, 1.0));
            double lambdaRad = Math.Acos(Clamp((L2 * L2 + e * e - L3 * L3) / (2 * L2 * e), -1.0, 1.0));

            double theta2Deg;
            if (pz < 0.74)
                theta2Deg = 90.0 + RadToDeg(betaRad) - RadToDeg(lambdaRad);
            else
                theta2Deg = 90.0 - RadToDeg(betaRad) - RadToDeg(lambdaRad);

            double f = L2 * Math.Sin(DegToRad(theta2Deg));
            double g = d - f;

            double theta3Deg = RadToDeg(Math.Acos(Clamp(g / L3, -1.0, 1.0)));

            double theta1Deg = RadToDeg(theta1Rad);
            double theta4Deg = RadToDeg(Alpha);
            double theta5Deg = RadToDeg(Gamma);

            const double tol = 0.08;

            var thetas5 = new[] { theta1Deg, theta2Deg, theta3Deg, theta4Deg, theta5Deg };
            var (pxC, pyC, pzC) = ForwardKinematics_ArduinoStyle(thetas5);

            if (Math.Abs(px - pxC) > tol || Math.Abs(py - pyC) > tol || Math.Abs(pz - pzC) > tol)
            {
                theta3Deg = -theta3Deg;
                thetas5[2] = theta3Deg;
                (pxC, pyC, pzC) = ForwardKinematics_ArduinoStyle(thetas5);
            }

            double min3 = GetTheta3Min(theta2Deg);
            double max3 = GetTheta3Max(theta2Deg);

            if (theta3Deg < min3 || theta3Deg > max3)
                throw new InvalidOperationException($"theta3 poza zakresem: {theta3Deg:F3} not in [{min3:F3}, {max3:F3}] dla theta2={theta2Deg:F3}");

            return new[] { theta1Deg, theta2Deg, theta3Deg, theta4Deg };
        }

        private List<double[]> BuildAnglesSequence()
        {
            if (currentContours == null || currentContours.Count == 0)
                throw new InvalidOperationException("Brak wczytanych konturów.");

            var allPoints = currentContours.SelectMany(c => c.Points).ToList();
            var result = new List<double[]>(allPoints.Count);

            foreach (var p in allPoints)
            {
                var angles = ComputeAnglesForPoint(p);
                result.Add(angles);
            }

            return result;
        }

        /* ---------- helpers ---------- */
        private string SelectedPort => (CmbPort.SelectedItem as ComboBoxItem)?.Content.ToString() ?? "COM6";
        private int SelectedBaud => int.TryParse((CmbBaud.SelectedItem as ComboBoxItem)?.Content.ToString(), out var b) ? b : 115200;

        private static int PercentToStep(int percent) => 2 + ((100 - percent) / 10) * 2;

        private void ApplyDensity(int percent)
        {
            _pointStep = PercentToStep(percent);
            TxtDensityUsed.Text = $"Zagęszczenie: {percent} %";
        }

        /* =================================================================== */
        /* =======================  W G R Y W A N I E  ======================= */
        /* =================================================================== */

        private void BtnWgraj_Click(object sender, RoutedEventArgs e)
        {
            var dlg = new OpenFileDialog
            {
                Title = "Wybierz plik do podglądu",
                Filter = "Obrazy (*.png;*.jpg;*.jpeg)|*.png;*.jpg;*.jpeg|Wszystkie pliki|*.*"
            };
            if (dlg.ShowDialog() != true) return;
            string path = dlg.FileName;

            bool rotateImage = ChkRotate.IsChecked == true;

            _currentImagePath = path;
            _currentRotate = rotateImage;

            int density = 70;
            int step = PercentToStep(density);
            currentContours = GetAllContours(path, step, rotateImage);
            int coordCount = currentContours.Sum(c => c.Points.Count);

            if (coordCount > MaxCoords)
            {
                while (density > 10 && coordCount > MaxCoords)
                {
                    density -= 10;
                    step = PercentToStep(density);
                    currentContours = GetAllContours(path, step, rotateImage);
                    coordCount = currentContours.Sum(c => c.Points.Count);
                }
            }
            else
            {
                while (density < 100)
                {
                    int testDensity = density + 10;
                    int testStep = PercentToStep(testDensity);
                    var testContours = GetAllContours(path, testStep, rotateImage);
                    int testCount = testContours.Sum(c => c.Points.Count);
                    if (testCount > MaxCoords) break;
                    density = testDensity;
                    step = testStep;
                    currentContours = testContours;
                    coordCount = testCount;
                }
            }

            ApplyDensity(density);

            _staticBmp?.Dispose();
            _staticBmp = GenerateContourBitmap(currentContours, 400, 300);
            _simBmp?.Dispose();
            _simBmp = null;

            ImgPreview.Source = BitmapToSource(_staticBmp);

            TxtStatus.Visibility = Visibility.Visible;
            TxtStatus.Text =
                $"Obraz wczytany poprawnie\n" +
                $"Liczba współrzędnych: {coordCount}\n" +
                $"Użyte zagęszczenie: {density} %\n" +
                $"Obrót 90°: {(rotateImage ? "TAK" : "NIE")}\n" +
                $"Aktualne Z rysowania: {Z.ToString("F4", CultureInfo.InvariantCulture)}\n" +
                $"Próg podwójnego obrysu: {_doubleEdgeThreshold.ToString("F2", CultureInfo.InvariantCulture)}";

            bool overLimit = coordCount > MaxCoords;
            BtnStartSignal.IsEnabled =
            BtnEndSignal.IsEnabled =
            BtnSimulate.IsEnabled = !overLimit;

            if (overLimit)
                TxtStatus.Text += $"\nPrzekroczono limit {MaxCoords} współrzędnych – przyciski zostały zablokowane.";
        }

        /* =================================================================== */
        /* ======================  S E R I A L  ============================== */
        /* =================================================================== */

        private void OpenPortIfNeeded()
        {
            if (_serialPort.IsOpen &&
               (_serialPort.PortName != SelectedPort || _serialPort.BaudRate != SelectedBaud))
                _serialPort.Close();

            if (!_serialPort.IsOpen)
            {
                _serialPort.PortName = SelectedPort;
                _serialPort.BaudRate = SelectedBaud;
                _serialPort.Open();
            }
        }

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string data = _serialPort.ReadExisting();
                Dispatcher.BeginInvoke((Action)(() =>
                {
                    TxtStatus.Text += $"\nOdpowiedź Arduino: {data}";
                    TxtStatus.Visibility = Visibility.Visible;
                }));
            }
            catch { }
        }

        private void BtnHome_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                OpenPortIfNeeded();
                _serialPort.WriteLine("1");
                TxtStatus.Text = $"Wysłano HOME (1) → {SelectedPort} @ {SelectedBaud}";
                TxtStatus.Visibility = Visibility.Visible;
            }
            catch (Exception ex) { MessageBox.Show("Błąd komunikacji: " + ex.Message); }
        }

        // ======= NOWE: wysyłka w chunkach (stabilna) =======
        private void SendAnglesInChunks(List<double[]> angles, int blocksPerLine = 80, int delayMs = 5)
        {
            // Arduino: wejście w tryb odbioru
            _serialPort.WriteLine("5");

            int total = angles.Count;
            int i = 0;

            var sb = new StringBuilder(8192);

            while (i < total)
            {
                sb.Clear();
                int end = Math.Min(i + blocksPerLine, total);

                for (; i < end; i++)
                {
                    var a = angles[i];
                    sb.Append("{");
                    sb.Append(a[0].ToString("F6", CultureInfo.InvariantCulture));
                    sb.Append(", ");
                    sb.Append(a[1].ToString("F6", CultureInfo.InvariantCulture));
                    sb.Append(", ");
                    sb.Append(a[2].ToString("F6", CultureInfo.InvariantCulture));
                    sb.Append(", ");
                    sb.Append(a[3].ToString("F6", CultureInfo.InvariantCulture));
                    sb.Append("},");
                }

                _serialPort.WriteLine(sb.ToString());

                if (delayMs > 0)
                    Thread.Sleep(delayMs);
            }

            // wyjście z trybu 5 do menu głównego
            _serialPort.WriteLine("back");
        }

        private void BtnStartSignal_Click(object sender, RoutedEventArgs e)
        {
            if (currentContours == null || currentContours.Count == 0)
            {
                MessageBox.Show("Brak wczytanego pliku lub danych (konturów).");
                return;
            }

            try
            {
                OpenPortIfNeeded();

                _anglesSequence = BuildAnglesSequence();

                // wysyłka chunkami zamiast jednej gigantycznej linii
                SendAnglesInChunks(_anglesSequence, blocksPerLine: 80, delayMs: 5);

                TxtStatus.Text =
                    $"Przeliczono i wysłano kąty trajektorii: {_anglesSequence.Count} punktów " +
                    $"→ {SelectedPort} @ {SelectedBaud}";
                TxtStatus.Visibility = Visibility.Visible;
            }
            catch (Exception ex)
            {
                MessageBox.Show("Błąd komunikacji lub obliczeń: " + ex.Message);
            }
        }

        private void BtnEndSignal_Click(object sender, RoutedEventArgs e)
        {
            try
            {
                OpenPortIfNeeded();
                _serialPort.WriteLine("4");

                TxtStatus.Text =
                    $"Wysłano polecenie wykonania trajektorii (4) → {SelectedPort} @ {SelectedBaud}";
                TxtStatus.Visibility = Visibility.Visible;
            }
            catch (Exception ex)
            {
                MessageBox.Show("Błąd komunikacji: " + ex.Message);
            }
        }

        private void BtnSendCoords_Click(object sender, RoutedEventArgs e)
        {
            MessageBox.Show(
                "Wysyłanie XYZ zostało zastąpione wysyłaniem kątów (opcja B).\n\n" +
                "Użyj przycisku „Gotowość do startu” (5), a potem „Wykonaj trajektorię” (4).");
        }

        /* =================================================================== */
        /* ======================  S Y M U L A C J A  ======================== */
        /* =================================================================== */

        private void BtnSimulate_Click(object sender, RoutedEventArgs e)
        {
            if (currentContours == null || currentContours.Count == 0)
            {
                MessageBox.Show("Najpierw wgraj plik.");
                return;
            }

            _simPoints = currentContours.SelectMany(c => c.Points).ToList();
            _simIndex = 0;

            _simBmp?.Dispose();
            _simBmp = new Bitmap(_staticBmp);

            _simTimer.Start();
        }

        private void SimTimer_Tick(object sender, EventArgs e)
        {
            if (_simPoints == null || _simIndex >= _simPoints.Count - 1)
            {
                _simTimer.Stop();
                return;
            }

            var pObj1 = _simPoints[_simIndex];
            var pObj2 = _simPoints[_simIndex + 1];

            bool penUp = Math.Abs(pObj1.Z - PenUpZ) < ZTolerance ||
                         Math.Abs(pObj2.Z - PenUpZ) < ZTolerance;

            if (!penUp)
            {
                double w = _simBmp.Width;
                double h = _simBmp.Height;

                var p1 = ToScreen(pObj1, w, h);
                var p2 = ToScreen(pObj2, w, h);

                using (var g = Graphics.FromImage(_simBmp))
                using (var pen = new System.Drawing.Pen(System.Drawing.Color.DeepSkyBlue, 2))
                {
                    g.DrawLine(pen, p1, p2);
                }

                ImgPreview.Source = BitmapToSource(_simBmp);
            }

            _simIndex++;
        }

        /* =================================================================== */
        /* ======================  S T E R O W A N I E  Z  =================== */
        /* =================================================================== */

        private void TxtZ_LostFocus(object sender, RoutedEventArgs e)
        {
            var text = TxtZ.Text?.Trim() ?? string.Empty;
            text = text.Replace(',', '.');

            if (double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture, out var val))
            {
                _z = val;
                TxtStatus.Text =
                    $"Ustawiono Z rysowania na {val.ToString("F4", CultureInfo.InvariantCulture)}.\n" +
                    $"Aby zastosować nowe Z w konturach, kliknij 'Przegeneruj'.";
                TxtStatus.Visibility = Visibility.Visible;
            }
            else
            {
                MessageBox.Show("Nieprawidłowa wartość Z. Podaj liczbę, np. 0.9825");
                TxtZ.Text = _z.ToString("F4", CultureInfo.InvariantCulture);
            }
        }

        private void BtnRegenZ_Click(object sender, RoutedEventArgs e)
        {
            if (string.IsNullOrEmpty(_currentImagePath))
            {
                MessageBox.Show("Najpierw wgraj plik.");
                return;
            }

            try
            {
                currentContours = GetAllContours(_currentImagePath, _pointStep, _currentRotate);

                _staticBmp?.Dispose();
                _staticBmp = GenerateContourBitmap(currentContours, 400, 300);

                _simBmp?.Dispose();
                _simBmp = null;

                ImgPreview.Source = BitmapToSource(_staticBmp);

                if (TxtStatus != null)
                {
                    int coordCount = currentContours.Sum(c => c.Points.Count);
                    TxtStatus.Text =
                        $"Przeliczono kontury z nowym Z = {Z.ToString("F4", CultureInfo.InvariantCulture)}\n" +
                        $"Liczba współrzędnych: {coordCount}";
                    TxtStatus.Visibility = Visibility.Visible;
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show("Błąd podczas przeliczania konturów: " + ex.Message);
            }
        }

        private void SldEdgeThreshold_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            _doubleEdgeThreshold = e.NewValue;

            if (TxtEdgeValue != null)
                TxtEdgeValue.Text = _doubleEdgeThreshold.ToString("F2", CultureInfo.InvariantCulture);

            if (string.IsNullOrEmpty(_currentImagePath))
            {
                if (TxtStatus != null)
                {
                    TxtStatus.Text = $"Ustawiono próg podwójnego obrysu na {_doubleEdgeThreshold.ToString("F2", CultureInfo.InvariantCulture)}";
                    TxtStatus.Visibility = Visibility.Visible;
                }
                return;
            }

            try
            {
                currentContours = GetAllContours(_currentImagePath, _pointStep, _currentRotate);

                _staticBmp?.Dispose();
                _staticBmp = GenerateContourBitmap(currentContours, 400, 300);

                _simBmp?.Dispose();
                _simBmp = null;

                ImgPreview.Source = BitmapToSource(_staticBmp);

                if (TxtStatus != null)
                {
                    int coordCount = currentContours.Sum(c => c.Points.Count);
                    TxtStatus.Text =
                        $"Zmieniono próg podwójnego obrysu na {_doubleEdgeThreshold.ToString("F2", CultureInfo.InvariantCulture)}\n" +
                        $"Liczba współrzędnych: {coordCount}";
                    TxtStatus.Visibility = Visibility.Visible;
                }
            }
            catch (Exception ex)
            {
                if (TxtStatus != null)
                {
                    TxtStatus.Text = "Błąd podczas przeliczania konturów: " + ex.Message;
                    TxtStatus.Visibility = Visibility.Visible;
                }
            }
        }

        private void ChkRotate_CheckedChanged(object sender, RoutedEventArgs e)
        {
            bool rotate = ChkRotate.IsChecked == true;
            _currentRotate = rotate;

            if (string.IsNullOrEmpty(_currentImagePath))
            {
                if (TxtStatus != null)
                {
                    TxtStatus.Text = $"Ustawiono obrót 90°: {(rotate ? "TAK" : "NIE")}";
                    TxtStatus.Visibility = Visibility.Visible;
                }
                return;
            }

            try
            {
                currentContours = GetAllContours(_currentImagePath, _pointStep, _currentRotate);

                _staticBmp?.Dispose();
                _staticBmp = GenerateContourBitmap(currentContours, 400, 300);

                _simBmp?.Dispose();
                _simBmp = null;

                ImgPreview.Source = BitmapToSource(_staticBmp);

                if (TxtStatus != null)
                {
                    int coordCount = currentContours.Sum(c => c.Points.Count);
                    TxtStatus.Text =
                        $"Zmieniono obrót 90° na: {(rotate ? "TAK" : "NIE")}\n" +
                        $"Liczba współrzędnych: {coordCount}";
                    TxtStatus.Visibility = Visibility.Visible;
                }
            }
            catch (Exception ex)
            {
                if (TxtStatus != null)
                {
                    TxtStatus.Text = "Błąd podczas przeliczania konturów (obrót): " + ex.Message;
                    TxtStatus.Visibility = Visibility.Visible;
                }
            }
        }

        /* =================================================================== */
        /* ======================  K O N T U R Y  ============================ */
        /* =================================================================== */

        private List<Contour> GetAllContours(string imagePath, int pointStep, bool rotate)
        {
            if (pointStep <= 0)
                throw new ArgumentOutOfRangeException(nameof(pointStep), "pointStep must be > 0");

            using var image = CvInvoke.Imread(imagePath, ImreadModes.Grayscale);
            if (image.IsEmpty)
                throw new FileNotFoundException("Cannot read image", imagePath);

            if (rotate)
                CvInvoke.Rotate(image, image, RotateFlags.Rotate90CounterClockwise);

            using var binaryImage = new Mat();
            CvInvoke.Threshold(image, binaryImage, 128, 255, ThresholdType.BinaryInv);

            using var contoursVec = new VectorOfVectorOfPoint();
            using var hierarchy = new Mat();

            CvInvoke.FindContours(binaryImage, contoursVec, hierarchy, RetrType.Tree, ChainApproxMethod.ChainApproxNone);

            var allContours = new List<Contour>();

            if (contoursVec.Size == 0 ||
                hierarchy == null || hierarchy.Rows == 0 || hierarchy.Cols == 0)
                return allContours;

            var h = (int[,,])hierarchy.GetData();
            int n = contoursVec.Size;

            var parentIndex = new int[n];
            var area = new double[n];

            for (int i = 0; i < n; i++)
            {
                parentIndex[i] = h[0, i, 3];
                area[i] = Math.Abs(CvInvoke.ContourArea(contoursVec[i]));
            }

            double doubleEdgeRatioThreshold = _doubleEdgeThreshold;

            for (int i = 0; i < n; i++)
            {
                int parent = parentIndex[i];

                if (parent >= 0)
                {
                    double aParent = area[parent];
                    double aChild = area[i];

                    if (aParent > 0 && aChild > 0)
                    {
                        double ratio = aChild / aParent;
                        if (ratio > doubleEdgeRatioThreshold)
                            continue;
                    }
                }

                var contourPoints = contoursVec[i].ToArray();
                if (contourPoints.Length == 0)
                    continue;

                var scaledPoints = new List<Point3D>(contourPoints.Length / pointStep + 2);

                scaledPoints.Add(ScalePoint(contourPoints[0], PenUpZ, image));

                for (int j = 0; j < contourPoints.Length; j += pointStep)
                    scaledPoints.Add(ScalePoint(contourPoints[j], Z, image));

                scaledPoints.Add(ScalePoint(contourPoints[^1], PenUpZ, image));

                allContours.Add(new Contour
                {
                    ContourNumber = i + 1,
                    Points = scaledPoints
                });
            }

            return allContours;
        }

        private Point3D ScalePoint(System.Drawing.Point p, double z, Mat img)
        {
            double sx = _xMin + (_xMax - _xMin) * (p.X / (double)img.Width);
            double sy = _yMin + (_yMax - _yMin) * (1 - p.Y / (double)img.Height);
            return new Point3D { X = sx, Y = sy, Z = z };
        }

        /* ---------- bitmap helpers ---------- */

        private Bitmap GenerateContourBitmap(List<Contour> contours, int w, int h)
        {
            var bmp = new Bitmap(w, h);
            using var g = Graphics.FromImage(bmp);
            g.Clear(System.Drawing.Color.White);

            using var pen = new System.Drawing.Pen(System.Drawing.Color.Black, 1);
            foreach (var c in contours)
            {
                var pts = c.Points.Select(p => ToScreen(p, w, h)).ToArray();
                if (pts.Length > 1) g.DrawLines(pen, pts);
            }
            return bmp;
        }

        private BitmapSource BitmapToSource(Bitmap bmp)
        {
            using var ms = new MemoryStream();
            bmp.Save(ms, ImageFormat.Png);
            ms.Position = 0;

            var img = new BitmapImage();
            img.BeginInit();
            img.CacheOption = BitmapCacheOption.OnLoad;
            img.StreamSource = ms;
            img.EndInit();
            img.Freeze();
            return img;
        }

        private System.Drawing.PointF ToScreen(Point3D p, double w, double h)
        {
            return new System.Drawing.PointF(
                (float)((p.X - _xMin) / (_xMax - _xMin) * w),
                (float)((1 - (p.Y - _yMin) / (_yMax - _yMin)) * h)
            );
        }

        /* =================================================================== */
        /* ===================  FK jak Arduino calculate_T_matrices  ========== */
        /* =================================================================== */
        private const int SIZE = 4;

        private static double[,] Mul(double[,] A, double[,] B)
        {
            var R = new double[SIZE, SIZE];
            for (int i = 0; i < SIZE; i++)
                for (int j = 0; j < SIZE; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < SIZE; k++)
                        sum += A[i, k] * B[k, j];
                    R[i, j] = sum;
                }
            return R;
        }

        private static double[,] RotZ(double thetaRad)
        {
            double c = Math.Cos(thetaRad), s = Math.Sin(thetaRad);
            return new double[,]
            {
                { c, -s, 0, 0 },
                { s,  c, 0, 0 },
                { 0,  0, 1, 0 },
                { 0,  0, 0, 1 }
            };
        }

        private static double[,] TransZ(double d)
        {
            return new double[,]
            {
                { 1,0,0,0 },
                { 0,1,0,0 },
                { 0,0,1,d },
                { 0,0,0,1 }
            };
        }

        private static double[,] TransX(double a)
        {
            return new double[,]
            {
                { 1,0,0,a },
                { 0,1,0,0 },
                { 0,0,1,0 },
                { 0,0,0,1 }
            };
        }

        private static double[,] RotX(double alphaRad)
        {
            double c = Math.Cos(alphaRad), s = Math.Sin(alphaRad);
            return new double[,]
            {
                { 1,0, 0,0 },
                { 0,c,-s,0 },
                { 0,s, c,0 },
                { 0,0, 0,1 }
            };
        }

        private static double[,] DH(double thetaDeg, double d, double a, double alphaDeg)
        {
            double th = DegToRad(thetaDeg);
            double al = DegToRad(alphaDeg);
            return Mul(Mul(Mul(RotZ(th), TransZ(d)), TransX(a)), RotX(al));
        }

        // 1:1 styl Arduino calculate_T_matrices + mask
        private (double px, double py, double pz) ForwardKinematics_ArduinoStyle(double[] thetas5)
        {
            var T00 = DH(thetas5[0], 0, 0, 90);
            var T03 = DH(90, 0, 0, 0);
            var T04 = DH(-thetas5[1], 0, L2, 0);
            var T05 = DH(-90, 0, 0, -90);

            var T10 = DH(thetas5[0], 0, 0, 90);
            var T16 = DH(thetas5[2], 0, L3, -90);

            var T21 = DH(thetas5[0], D1, 0, 90);
            var T23 = DH(-thetas5[3] + 90, 0, D4, 0);
            var T29 = DH(-90, 0, 0, -90);
            var T210 = DH(0, 0, 0, 0);

            var T0 = Mul(Mul(Mul(T00, T03), T04), T05);
            var T1 = Mul(T10, T16);
            var T2 = Mul(Mul(Mul(T21, T23), T29), T210);

            // mask: tylko translacje (0..2,3) bierzemy jako (T0+T1)+T2, reszta = T2
            var T = new double[SIZE, SIZE];
            for (int i = 0; i < SIZE; i++)
                for (int j = 0; j < SIZE; j++)
                {
                    bool isTrans = (j == 3 && i <= 2);
                    T[i, j] = isTrans ? (T0[i, j] + T1[i, j] + T2[i, j]) : T2[i, j];
                }

            return (T[0, 3], T[1, 3], T[2, 3]);
        }
    }

    /* ---------- POCO ---------- */

    public class Point3D
    {
        public double X, Y, Z;
    }

    public class Contour
    {
        public int ContourNumber;
        public List<Point3D> Points = new();
        public bool IsOuter { get; set; }
        public int? ParentContourNumber { get; set; }
    }
}
