using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using Microsoft.Kinect;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Linq;
using System.Threading.Tasks;
using System.Windows.Forms;
using static System.Windows.Forms.VisualStyles.VisualStyleElement.Rebar;

namespace KinectHandTrack
{
    public class FormMao : Form
    {
        public PictureBox pictureBox;

        public FormMao()
        {
            this.Text = "Deteccao de Mao - Aberta ou Fechada";
            this.Width = 640;
            this.Height = 480;

            pictureBox = new PictureBox();
            pictureBox.Dock = DockStyle.Fill;
            pictureBox.SizeMode = PictureBoxSizeMode.StretchImage;

            this.Controls.Add(pictureBox);
        }

        private void InitializeComponent()
        {
            this.SuspendLayout();
            // 
            // FormMao
            // 
            this.ClientSize = new System.Drawing.Size(284, 261);
            this.Name = "FormMao";
            this.ResumeLayout(false);

        }
    }
    public partial class FormControle : Form
    {
        private KinectSensor kinect; //Declare Kinect object
        private FormMao cameraWindow;
        private RCAPINet.Spel m_spel; //Declare robot Spel object
        RCAPINet.SpelPoint pt_ref; //Declare robot Reference Point variable

        //Declare list of points to be saved
        private List<(string Nome, RCAPINet.SpelPoint Ponto)> pontosSalvos = new List<(string, RCAPINet.SpelPoint)>();
        private int contadorPontos = 1;

        //Declare throttle variable
        private DateTime lastUpdateTime = DateTime.MinValue;

        //Calibration variables
        private float handxmin = float.MaxValue;
        private float handxmax = float.MinValue;
        private float handymin = float.MaxValue;
        private float handymax = float.MinValue;
        private float handzmin = float.MaxValue;
        private float handzmax = float.MinValue;
        //Robot range values
        const float robotXmin = -350;
        const float robotXmax = 350;
        const float robotYmin = 200;
        const float robotYmax = 500;
        const float robotZmin = 250;
        const float robotZmax = 750;

        private bool isCalibrating = true; // Flag que indica se ainda está calibrando
        private bool modoRastreamento = true; // Flag que indica se o robô está no modo rastreio

        //Método para interpolação linear
        private float MapRange(float value, float inMin, float inMax, float outMin, float outMax)
        {
            if (inMax - inMin == 0) return outMin; // evitar divisão por zero
            return (value - inMin) / (inMax - inMin) * (outMax - outMin) + outMin;
        }
        public FormControle()
        {
            InitializeComponent();
            this.Load += FormControle_Load;
        }
        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            if (kinect != null)
            {
                kinect.AllFramesReady -= Kinect_AllFramesReady;
            }

         //   m_spel.Dispose(); // Libera recurso do robô Epson
            base.OnFormClosing(e);
        }

        private void FormControle_Load(object sender, EventArgs e)
        {
            Console.WriteLine("1. Abrindo cameraWindow");
            // Abrir nova janela
            cameraWindow = new FormMao();
            cameraWindow.Show();

            Console.WriteLine("2. Inicializando robô");
            m_spel = new RCAPINet.Spel();
            m_spel.Initialize();

            //Botão para finalizar calibração
            Button btnSetCalibrationDone = new Button();
            btnSetCalibrationDone.Text = "Finalizar Calibração";
            btnSetCalibrationDone.Top = 10;
            btnSetCalibrationDone.Left = 10;
            btnSetCalibrationDone.Click += BtnSetCalibrationDone_Click;
            this.Controls.Add(btnSetCalibrationDone);

            Label lblCalibStatus = new Label();
            lblCalibStatus.Text = "Calibrando... Mova sua mão para todos os extremos.";
            lblCalibStatus.Top = 40;
            lblCalibStatus.Left = 10;
            lblCalibStatus.Width = 400;
            this.Controls.Add(lblCalibStatus);

            //Botão para salvar ponto
            Button btnSalvarPonto = new Button();
            btnSalvarPonto.Text = "Salvar Novo Ponto";
            btnSalvarPonto.Top = 70;
            btnSalvarPonto.Left = 10;
            btnSalvarPonto.Click += BtnSalvarPonto_Click;
            this.Controls.Add(btnSalvarPonto);

            //Lista de pontos salvos
            ListBox listBoxPontos = new ListBox();
            listBoxPontos.Name = "listBoxPontos";
            listBoxPontos.Top = 100;
            listBoxPontos.Left = 10;
            listBoxPontos.Width = 200;
            listBoxPontos.Height = 150;
            this.Controls.Add(listBoxPontos);

            // Botão para mover até o ponto selecionado
            Button btnMoverParaPonto = new Button();
            btnMoverParaPonto.Text = "Mover para Ponto";
            btnMoverParaPonto.Top = 260;
            btnMoverParaPonto.Left = 10;
            btnMoverParaPonto.Click += BtnMoverParaPonto_Click;
            this.Controls.Add(btnMoverParaPonto);

            //Checkbox para definir se robô deve rastrear a mão ou não
            CheckBox chkModoRastreamento = new CheckBox();
            chkModoRastreamento.Text = "Rastrear Mão";
            chkModoRastreamento.Checked = true;
            chkModoRastreamento.Top = 290; // ou qualquer posição livre
            chkModoRastreamento.Left = 10;
            chkModoRastreamento.CheckedChanged += ChkModoRastreamento_CheckedChanged;
            this.Controls.Add(chkModoRastreamento);

            m_spel.EventReceived += new RCAPINet.Spel.EventReceivedEventHandler(m_spel_EventReceived);
            m_spel.Project = "c:\\EpsonRC70\\projects\\API_Demos\\Demo1\\demo1.sprj";

            m_spel.MotorsOn = true; //Turn On robot motors
            m_spel.Speed(100); //Define robot speed
            pt_ref = m_spel.GetPoint("PT1"); //Get Default Point1
            m_spel.ShowWindow(RCAPINet.SpelWindows.Simulator); //Show Simulator window
        
            Console.WriteLine("3. Adicionando evento Kinect");
            try
            {
                // Kinect initialization
                kinect = KinectManager.Instance.Sensor;

                Console.WriteLine("Sensor inicializado: " + (kinect != null));
                Console.WriteLine("Sensor rodando? " + kinect.IsRunning);
                Console.WriteLine("ColorStream habilitado? " + kinect.ColorStream.IsEnabled);
                Console.WriteLine("SkeletonStream habilitado? " + kinect.SkeletonStream.IsEnabled);

                kinect.AllFramesReady += Kinect_AllFramesReady;
                Console.WriteLine("Chegou um frame");
            }
            catch (Exception ex)
            {
                MessageBox.Show("Erro ao inicializar Kinect: " + ex.Message);
            }
        }
        private void BtnSetCalibrationDone_Click(object sender, EventArgs e)
        {
            isCalibrating = false;
            MessageBox.Show($"Calibração finalizada:\n" +
                $"Xmin: {handxmin:F2}, Xmax: {handxmax:F2}\n" +
                $"Ymin: {handymin:F2}, Ymax: {handymax:F2}\n" +
                $"Zmin: {handzmin:F2}, Zmax: {handzmax:F2}");
        }

        private void BtnSalvarPonto_Click(object sender, EventArgs e)
        {
            string nome = $"PT{contadorPontos}";
            var novoPonto = new RCAPINet.SpelPoint
            {
                X = pt_ref.X,
                Y = pt_ref.Y,
                Z = pt_ref.Z
            };

            pontosSalvos.Add((nome, novoPonto));
            contadorPontos++;

            // Atualizar lista de pontos (veja passo 4)
            AtualizarListaPontos();

            MessageBox.Show($"Ponto {nome} salvo: X={novoPonto.X}, Y={novoPonto.Y}, Z={novoPonto.Z}");
        }

        private void AtualizarListaPontos()
        {
            var listBox = this.Controls.Find("listBoxPontos", true).FirstOrDefault() as ListBox;
            if (listBox != null)
            {
                listBox.Items.Clear();
                foreach (var ponto in pontosSalvos)
                {
                    listBox.Items.Add(ponto.Nome);
                }
            }
        }
        private void BtnMoverParaPonto_Click(object sender, EventArgs e)
        {
            var listBox = this.Controls.Find("listBoxPontos", true).FirstOrDefault() as ListBox;
            if (listBox == null || listBox.SelectedIndex < 0)
            {
                MessageBox.Show("Selecione um ponto para mover.");
                return;
            }

            var nomeSelecionado = listBox.SelectedItem.ToString();
            var ponto = pontosSalvos.FirstOrDefault(p => p.Nome == nomeSelecionado).Ponto;

            try
            {
                m_spel.WaitCommandComplete();
                m_spel.Go(ponto);
            }
            catch (Exception ex)
            {
                MessageBox.Show("Erro ao mover para o ponto: " + ex.Message);
            }
        }
        private void ChkModoRastreamento_CheckedChanged(object sender, EventArgs e)
        {
            CheckBox chk = sender as CheckBox;
            modoRastreamento = chk.Checked;
            this.Text = modoRastreamento ? "Modo: Rastreando Mão" : "Modo: Manual (Lista de Pontos)";
        }

        public void m_spel_EventReceived(object sender, RCAPINet.SpelEventArgs e)
        {

        }
        private bool TryGetSafeHandRoi(Joint hand, ColorImageFormat format, out Rectangle roi, out ColorImagePoint point)
        {
            roi = Rectangle.Empty;
            point = new ColorImagePoint();

            if (hand.TrackingState != JointTrackingState.Tracked)
                return false;

            point = kinect.CoordinateMapper.MapSkeletonPointToColorPoint(hand.Position, format);

            //int boxSize = 90;
            int boxSize = 175;
            int x = Math.Max(0, point.X - boxSize / 2);
            int y = Math.Max(0, point.Y - boxSize / 2);
            //int width = Math.Min(boxSize, 640 - x);
            //int height = Math.Min(boxSize, 480 - y);
            int width = Math.Min(boxSize, 1280 - x);
            int height = Math.Min(boxSize, 960 - y);
            roi = new Rectangle(x, y, width, height);

            return roi.Width > 0 && roi.Height > 0;
        }

        private Bitmap GetBitmapFromColorFrame(ColorImageFrame colorFrame)
        {
            byte[] pixels = new byte[colorFrame.PixelDataLength];
            colorFrame.CopyPixelDataTo(pixels);

            Bitmap bitmap = new Bitmap(colorFrame.Width, colorFrame.Height, PixelFormat.Format32bppRgb);
            Rectangle rect = new Rectangle(0, 0, bitmap.Width, bitmap.Height);
            BitmapData bmpData = bitmap.LockBits(rect, ImageLockMode.WriteOnly, bitmap.PixelFormat);
            System.Runtime.InteropServices.Marshal.Copy(pixels, 0, bmpData.Scan0, pixels.Length);
            bitmap.UnlockBits(bmpData);
            return bitmap;
        }
        public Image<Gray, byte> SegmentSkinYCgCr(Image<Bgr, byte> inputImage)
        {
            int width = inputImage.Width;
            int height = inputImage.Height;

            Image<Gray, byte> skinMask = new Image<Gray, byte>(width, height);

            byte[,,] bgrData = inputImage.Data;
            byte[,,] maskData = skinMask.Data;

            // Pseudoimagens para exibição colorida
            Image<Bgr, byte> pseudoYCgCr = new Image<Bgr, byte>(width, height);
            Image<Bgr, byte> pseudoYCgCrRot = new Image<Bgr, byte>(width, height);

            // Acesso aos dados de imagem
            byte[,,] pseudoData = pseudoYCgCr.Data;
            byte[,,] pseudoRotData = pseudoYCgCrRot.Data;

            double cos30 = Math.Cos(Math.PI / 6); // ~0.866
            double sin30 = Math.Sin(Math.PI / 6); // ~0.5

            for (int y = 0; y < height; y++)
            {
                for (int x = 0; x < width; x++)
                {
                    byte B = bgrData[y, x, 0];
                    byte G = bgrData[y, x, 1];
                    byte R = bgrData[y, x, 2];

                    // 1. Conversão RGB para YCgCr
                    double Y = 0.257 * R + 0.504 * G + 0.098 * B + 16;
                    double Cg = -0.317 * R + 0.438 * G - 0.121 * B + 128;
                    double Cr = 0.439 * R - 0.368 * G - 0.071 * B + 128;

                    // 2. Rotação para YCg′Cr′
                    double Cg_ = Cg * cos30 + Cr * sin30 - 48;
                    double Cr_ = -Cg * sin30 + Cr * cos30 + 80;

                    // Clamping (garantir valores válidos entre 0 e 255)
                    byte Yb = (byte)Math.Max(0, Math.Min(255, Y));
                    byte Cgb = (byte)Math.Max(0, Math.Min(255, Cg));
                    byte Crb = (byte)Math.Max(0, Math.Min(255, Cr));
                    byte Cg_b = (byte)Math.Max(0, Math.Min(255, Cg_));
                    byte Cr_b = (byte)Math.Max(0, Math.Min(255, Cr_));

                    // 3. Condições para região de pele
                    //if (Cg_ >= 125 && Cg_ <= 140 && Cr_ >= 136 && Cr_ <= 217) // Artigo
                    if (Cg_ >= 110 && Cg_ <= 150 && Cr_ >= 130 && Cr_ <= 230)
                    {
                        maskData[y, x, 0] = 255;
                    }
                    else
                    {
                        maskData[y, x, 0] = 0;
                    }

                    // Pseudoimagem YCgCr: R=Y, G=Cg, B=Cr
                    pseudoData[y, x, 2] = Yb;    // R
                    pseudoData[y, x, 1] = Cgb;   // G
                    pseudoData[y, x, 0] = Crb;   // B

                    // Pseudoimagem YCg′Cr′: R=Y, G=Cg′, B=Cr′
                    pseudoRotData[y, x, 2] = Yb;     // R
                    pseudoRotData[y, x, 1] = Cg_b;   // G
                    pseudoRotData[y, x, 0] = Cr_b;   // B
                }
            }
            // Exibir pseudoimagens - tratamento

            //CvInvoke.Imshow("Imagem Original", inputImage);
            //CvInvoke.Imshow("Pseudoimagem YCgCr (R=Y, G=Cg, B=Cr)", pseudoYCgCr);
            //CvInvoke.Imshow("Pseudoimagem YCg′Cr′ (R=Y, G=Cg′, B=Cr′)", pseudoYCgCrRot);
            //CvInvoke.Imshow("Máscara a partir da segmentação YCg'Cr'", skinMask);

            return skinMask;
        }

        private string DetectarMaoAbertaOuFechada(Mat handMat)
        {
            if (handMat == null || handMat.IsEmpty)
                return "handMat.IsEmpty";

            // 1. Imagem recortada com a posição da mão
            Image<Bgr, byte> handImage = handMat.ToImage<Bgr, byte>();
            CvInvoke.Imshow("1 - Hand RGB", handMat);

            // 2. Suavização
            CvInvoke.GaussianBlur(handMat, handMat, new Size(5, 5), 0);
            //CvInvoke.Imshow("2 - Blur", handMat);

            // 3. Segmentação por cor de pele no espaço YCgCr
            Image<Gray, byte> skinMask = SegmentSkinYCgCr(handImage);
            Mat thresh = skinMask.Mat;
            //CvInvoke.Imshow("3 - SkinMask YCgCr", thresh);

            // 4. Operações morfológicas
            Mat morph = new Mat();
            Mat kernel = CvInvoke.GetStructuringElement(ElementShape.Rectangle, new Size(3, 3), new Point(-1, -1));
            CvInvoke.MorphologyEx(thresh, morph, MorphOp.Open, kernel, new Point(-1, -1), 1, Emgu.CV.CvEnum.BorderType.Default, new MCvScalar());
            CvInvoke.MorphologyEx(morph, morph, MorphOp.Close, kernel, new Point(-1, -1), 1, Emgu.CV.CvEnum.BorderType.Default, new MCvScalar());
            // CvInvoke.Imshow("4 - Morfologia (Open + Close)", morph);


            // 5. Operação Extra -  Remoção dos blobs muito pequenos da máscara pós-morfologia
            Mat filteredMask = Mat.Zeros(morph.Size.Height, morph.Size.Width, DepthType.Cv8U, 1);
            VectorOfVectorOfPoint validContours = new VectorOfVectorOfPoint();
            Mat hierarchy = new Mat();
            CvInvoke.FindContours(morph, validContours, hierarchy, RetrType.External, ChainApproxMethod.ChainApproxSimple);

            for (int i = 0; i < validContours.Size; i++)
            {
                double area = CvInvoke.ContourArea(validContours[i]);
                if (area > 250) // valor ajustável, pode ser 200 ou 500 conforme os testes
                {
                    CvInvoke.DrawContours(filteredMask, validContours, i, new MCvScalar(255), -1);
                }
            }
            CvInvoke.Imshow("5 - Máscara sem pequenos blobs", filteredMask);


            // 5. Encontrar contornos
            VectorOfVectorOfPoint contours = new VectorOfVectorOfPoint();
            CvInvoke.FindContours(filteredMask, contours, null, RetrType.External, ChainApproxMethod.ChainApproxSimple);
            if (contours.Size == 0)
            {
                Console.WriteLine("Nenhum contorno encontrado.");
                return "Nenhum Contorno";
            }

            // 6. Encontrar o maior contorno (presumivelmente a mão)
            int maxIndex = 0;
            double maxArea = 0;
            for (int i = 0; i < contours.Size; i++)
            {
                double area = CvInvoke.ContourArea(contours[i]);
                if (area > maxArea)
                {
                    maxArea = area;
                    maxIndex = i;
                }
            }
            VectorOfPoint handContour = contours[maxIndex];

            // 7. Visualizar contorno
            Mat debugMat = handMat.Clone();
            CvInvoke.DrawContours(debugMat, contours, maxIndex, new MCvScalar(0, 0, 255), 2); // vermelho
            //CvInvoke.Imshow("7 - Contorno Principal", debugMat);

            // 8. Convex Hull
            VectorOfInt hull = new VectorOfInt();
            CvInvoke.ConvexHull(handContour, hull, false);

            Point[] contourPoints = handContour.ToArray();
            Point[] hullPoints = hull.ToArray().Select(i => contourPoints[i]).ToArray();
            VectorOfPoint hullContour = new VectorOfPoint(hullPoints);
            CvInvoke.Polylines(debugMat, hullContour, true, new MCvScalar(0, 255, 0), 2); // verde
            //CvInvoke.Imshow("8 - Hull + Contorno", debugMat);

            // 9. Defeitos convexos
            using (Mat defects = new Mat())
            {
                if (handContour.Size < 3 || hull.Size < 3)
                    return "Defeitos Convexos";

                try
                {
                    CvInvoke.ConvexityDefects(handContour, hull, defects);
                    if (defects.Rows == 0) return "Fechada";

                    int defectCount = defects.Rows;
                    int contagemDefeitosProfundos = 0;

                    using (Matrix<int> defectData = new Matrix<int>(defects.Rows, defects.Cols, defects.NumberOfChannels))
                    {
                        defects.CopyTo(defectData);

                        for (int i = 0; i < defectCount; i++)
                        {
                            int startIdx = defectData.Data[i, 0];
                            int endIdx = defectData.Data[i, 1];
                            int farIdx = defectData.Data[i, 2];
                            float depth = defectData.Data[i, 3] / 256.0f;

                            if (depth > 10)
                            {
                                contagemDefeitosProfundos++;
                                CvInvoke.Circle(debugMat, contourPoints[farIdx], 5, new MCvScalar(255, 0, 0), -1); // azul
                            }
                        }
                    }

                    CvInvoke.Imshow("9 - Defeitos Convexos", debugMat);

                    return contagemDefeitosProfundos >= 4 ? "Aberta" : "Fechada";
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Erro ao calcular ConvexityDefects: " + ex.Message);
                    return "Erro ao calcular";
                }
            }
        }
        private async void Kinect_AllFramesReady(object sender, AllFramesReadyEventArgs e)
        {
            using (ColorImageFrame colorFrame = e.OpenColorImageFrame())
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame == null || colorFrame == null) return;

                // Converte o frame de cor para imagem
                Bitmap bitmap = GetBitmapFromColorFrame(colorFrame);
                Image<Bgr, byte> image = bitmap.ToImage<Bgr, byte>();
                Mat frameMat = image.Mat;

                // Lê os dados do esqueleto
                Skeleton[] skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                skeletonFrame.CopySkeletonDataTo(skeletons);
                Skeleton skeleton = skeletons.FirstOrDefault(s => s.TrackingState == SkeletonTrackingState.Tracked);

                if (skeleton == null)
                    return;

                Joint handRight = skeleton.Joints[JointType.HandRight];
                Joint handLeft = skeleton.Joints[JointType.HandLeft];

                //Mão esquerda
                if (TryGetSafeHandRoi(handLeft, colorFrame.Format, out Rectangle roiLeft, out ColorImagePoint pointLeft))
                {
                    Point textPosLeft = new Point(roiLeft.X, roiLeft.Y - 10);
                    MCvScalar colorLeft = new MCvScalar(0, 255, 0); // Verde
                    string labelLeft = "Esquerda";

                    CvInvoke.Rectangle(frameMat, roiLeft, colorLeft, 2);
                    CvInvoke.PutText(frameMat, labelLeft, textPosLeft, FontFace.HersheySimplex, 1.2, colorLeft, 1);
                }
                //Mão Direita
                if (TryGetSafeHandRoi(handRight, colorFrame.Format, out Rectangle roiRight, out ColorImagePoint pointRight))
                {
                    Point textPosRight = new Point(roiRight.X, roiRight.Y - 10);
                    MCvScalar colorRight = new MCvScalar(255, 0, 0); // Azul
                    string labelRight = "Direita";

                    CvInvoke.Rectangle(frameMat, roiRight, colorRight, 2);
                    CvInvoke.PutText(frameMat, labelRight, textPosRight, FontFace.HersheySimplex, 1.2, colorRight, 1);
                }
                // Atualizar imagem de forma segura na UI
                if (cameraWindow != null && cameraWindow.pictureBox != null)
                {
                    cameraWindow.pictureBox.Invoke((Action)(() =>
                    {
                        cameraWindow.pictureBox.Image?.Dispose();
                        cameraWindow.pictureBox.Image = frameMat.ToBitmap();
                    }));
                }


                foreach (Joint hand in new[] { handLeft, handRight })
                {
                    // --- MÃO ESQUERDA (detecção aberta/fechada)
                    if (hand.JointType == JointType.HandLeft)
                    {
                            Mat handRegion = new Mat(frameMat, roiLeft);
                            string StatusMaoEsquerda = DetectarMaoAbertaOuFechada(handRegion);
                            Console.WriteLine(StatusMaoEsquerda);
                    }
                    // --- MÃO DIREITA (controle do robo)
                    if (hand.JointType == JointType.HandRight)
                    {
                        // Throttle to 10 updates per second
                        if ((DateTime.Now - lastUpdateTime).TotalMilliseconds < 100)
                            return;
                        lastUpdateTime = DateTime.Now;

                        //Update coordinates based on new hand position
                        float x = hand.Position.X;
                        float y = hand.Position.Y;
                        float z = hand.Position.Z;

                        // Update the window title with hand position
                        this.BeginInvoke((Action)(() =>
                        {
                            this.Text = $"Right Hand - X: {x:F2}, Y: {y:F2}, Z: {z:F2}"; //Display Hand Position
                        }));

                        if (isCalibrating)
                        {
                            // Atualiza limites
                            handxmin = Math.Min(handxmin, x);
                            handxmax = Math.Max(handxmax, x);
                            handymin = Math.Min(handymin, y);
                            handymax = Math.Max(handymax, y);
                            handzmin = Math.Min(handzmin, z);
                            handzmax = Math.Max(handzmax, z);
                            return; // Não controla o robô durante calibração
                        }

                        if (!modoRastreamento) return;

                        pt_ref.X = MapRange(x, handxmax, handxmin, robotXmin, robotXmax); //Define new X position
                        pt_ref.Y = MapRange(z, handzmax, handzmin, robotYmin, robotYmax); //Define new Y position
                        pt_ref.Z = MapRange(y, handymin, handymax, robotZmin, robotZmax); //Define new Z position

                        if (pt_ref.X > robotXmax || pt_ref.X < robotXmin) return;
                        if (pt_ref.Y > robotYmax || pt_ref.Y < robotYmin) return;
                        if (pt_ref.Z > robotZmax || pt_ref.Z < robotZmin) return;

                        try
                        {
                            await Task.Run(() =>
                            {
                                m_spel.WaitCommandComplete(); //Wait robot to conclude task
                                
                                
                                
                                
                                
                                
                                
                                
                                
                                m_spel.Go(pt_ref); // Go to the new coordinate
                            });
                        }
                        catch (Exception ex)
                        {
                            // Handle any exceptions that may occur during the command
                            this.BeginInvoke((Action)(() =>
                            {
                                MessageBox.Show("Error moving robot: " + ex.Message);
                            }));
                        }
                    }
                }
            }
        }

    }
}
