using System;

using System.Collections.Generic;

using System.ComponentModel;

using System.Data;

using System.Drawing;

using System.IO;

using System.IO.Ports;

using System.Linq;

using System.Text;

using System.Threading;

using System.Windows.Forms;

using System.Windows.Forms.DataVisualization.Charting;



namespace IERMS

{

    public partial class Form1 : Form

    {



        // 시리얼 포트 객체 생성

        private SerialPort serialPort = new SerialPort();



        private const int MAX_DATA_POINTS = 100;



        // for Progress

        private int connectionProgress = 0;



        private System.Windows.Forms.Timer timerUsage = new System.Windows.Forms.Timer();

        private DateTime connectStartTime;



        private List<int[]> rawDataList = new List<int[]>();



        public Form1()

        {

            InitializeComponent();

            InitializeCharts();

            



            this.CenterToScreen();



            // Initailize UI config

            progressBar1.Visible = false;

            progressBar1.Maximum = 100;

            chkRepeat.Enabled = false;



            timerUsage.Interval = 1000;

            timerUsage.Tick += TimerUsage_Tick;



        }





        private void TimerUsage_Tick(object sender, EventArgs e)

        {

            TimeSpan elapsed = DateTime.Now - connectStartTime;

            lblTimeState.Text = elapsed.ToString(@"hh\:mm\:ss");

        }



        private void AddToLog(string msg)

        {

            // 다른 스레드에서 호출될 경우를 대비한 Invoke 처리

            if (lbConsole.InvokeRequired)

            {

                lbConsole.Invoke(new Action(() => AddToLog(msg)));

            }

            else

            {

                string timestamp = DateTime.Now.ToString("HH:mm:ss.fff");

                lbConsole.Items.Add($"[{timestamp}] {msg}");



                // 자동 스크롤 (맨 아래로)

                lbConsole.TopIndex = lbConsole.Items.Count - 1;

            }

        }



        // 1. 차트 초기화 설정 (속도 최적화 및 스타일)

        private void InitializeCharts()

        {

            SetupDualChart(chart1, "axis-x Output", "axis-x Input");

            SetupDualChart(chart2, "axis-y Output", "axis-y Input");

            SetupDualChart(chart3, "axis-z Output", "axis-z Input");

            SetupDualChart(chart4, "axis-c Output", "axis-c Input");

        }



        private void SetupDualChart(Chart chart, string nameActual, string nameTarget)

        {

            chart.Series.Clear();



            // Series 1: Actual (실선, 진한색)

            var s1 = chart.Series.Add(nameActual);

            s1.ChartType = SeriesChartType.FastLine;

            s1.Color = Color.Blue; // 실제값은 파란색

            s1.BorderWidth = 2;



            // Series 2: Target (점선 or 연한색, 붉은색)

            var s2 = chart.Series.Add(nameTarget);

            s2.ChartType = SeriesChartType.FastLine;

            s2.Color = Color.Red;  // 목표값은 빨간색

            s2.BorderWidth = 1;

            // s2.BorderDashStyle = ChartDashStyle.Dash; // 점선으로 하고 싶으면 주석 해제





            // 스타일 설정

            chart.ChartAreas[0].AxisX.LabelStyle.Enabled = false;

            chart.ChartAreas[0].AxisX.MajorGrid.LineColor = Color.LightGray;

            chart.ChartAreas[0].AxisY.MajorGrid.LineColor = Color.LightGray;

        }









        private void cboPort_Click(object sender, EventArgs e)

        {

            cboPort.Items.Clear();



            string[] ports = SerialPort.GetPortNames().Distinct().OrderBy(s => s).ToArray();



            foreach (var item in ports)

            {

                cboPort.Items.Add(item);

            }

        }



        // 3. 연결 버튼 클릭

        private void btnConnect_Click(object sender, EventArgs e)

        {

            if (cboPort.Text == "")

            {

                MessageBox.Show("Please select the port", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);

                return;

            }



            try

            {

                if (serialPort.IsOpen)

                {



                    serialPort.Close();

                    timerUsage.Stop();



                    //progressBar1.Value = 0;



                    btnConnect.BackColor = Color.Green;

                    btnConnect.ForeColor = Color.White;

                    btnConnect.Text = "Connect";



                    lblPortState.Text = "-";

                    lblBaudRateState.Text = "-";



                    lblStatusState.ForeColor = Color.Red;

                    lblStatusState.Text = "Off";



                    lblTimeState.Text = "Start";



                    cboPort.Enabled = true;

                    cboBaudRate.Enabled = true;

                    chkRepeat.Checked = false;

                    chkRepeat.Enabled = false;

                    btnReset.Enabled = false;



                    lbConsole.Items.Add("[System] Port Closed.");



                }



                else

                {

                    serialPort.PortName = cboPort.Text;

                    serialPort.BaudRate = Convert.ToInt32(cboBaudRate.Text); //115200

                    

                    serialPort.DataReceived += SerialPort_DataReceived;



                    serialPort.DtrEnable = true;



                    serialPort.Open();



                    lbConsole.Items.Add($"[System] Open {cboPort.Text} @ {serialPort.BaudRate}");





                    // loading UI

                    progressBar1.Maximum = 100;

                    progressBar1.Value = 0;

                    progressBar1.Visible = true;

                    connectionProgress = 0;

                    timerConnect.Start(); // 타이머가 UI를 업데이트함





                    btnConnect.BackColor = Color.IndianRed;

                    btnConnect.ForeColor = Color.White;

                    btnConnect.Text = "Disconnect";



                    lblPortState.Text = cboPort.Text;

                    lblBaudRateState.Text = cboBaudRate.Text;



                    lblStatusState.ForeColor = Color.Green;

                    lblStatusState.Text = "On";



                    cboPort.Enabled = false;

                    cboBaudRate.Enabled = false;

                    btnReset.Enabled = true;

                }

            }

            catch (Exception)

            {

                MessageBox.Show("Connection error", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);

            }

        }



        private void timerConnect_Tick(object sender, EventArgs e)

        {

            // 아두이노 부팅 시간(약 2~3초)을 확보하기 위해 진행 속도를 늦춤

            connectionProgress += 5; // 10에서 2로 변경 (약 5초 대기)



            if (connectionProgress > 100) connectionProgress = 100;



            if (connectionProgress > progressBar1.Maximum) connectionProgress = progressBar1.Maximum;



            progressBar1.Value = connectionProgress;



            if (connectionProgress >= 100)

            {

                timerConnect.Stop();

                progressBar1.Visible = false;



                // [중요] 부팅 완료 후 버퍼 비우기 (부팅 중 발생한 쓰레기 데이터 제거)

                if (serialPort.IsOpen)

                {

                    serialPort.DiscardInBuffer();

                    serialPort.DiscardOutBuffer();

                }



                chkRepeat.Enabled = true;

                MessageBox.Show("Arduino connection and initialization complete!");



                connectStartTime = DateTime.Now;

                timerUsage.Start();



                lbConsole.Items.Add("[System] Ready.");

            }

        }







        // 5. 데이터 수신 및 파싱 (핵심 로직)

        private void SerialPort_DataReceived(object sender, SerialDataReceivedEventArgs e)

        {

            try

            {

                // 아두이노에서 한 줄 읽기

                string rawData = serialPort.ReadLine();



                // UI 스레드에서 차트 업데이트 실행

                this.BeginInvoke(new Action(() =>

                {

                    ParseAndPlot(rawData);

                }));

            }

            catch { /* 통신 중 끊김 등의 예외 무시 */ }

        }



        private void ParseAndPlot(string data)

        {

            if (!data.StartsWith("E")) return;



            string[] parts = data.Split(',');





            // E, Time, X_act, X_tar, Y_act, Y_tar, Z_act, Z_tar, C_act, C_tar (총 10개)

            if (parts.Length >= 10)

            {

                double x_act, x_tar, y_act, y_tar, z_act, z_tar, c_act, c_tar;



                if (double.TryParse(parts[2], out x_act) && double.TryParse(parts[3], out x_tar) &&

                    double.TryParse(parts[4], out y_act) && double.TryParse(parts[5], out y_tar) &&

                    double.TryParse(parts[6], out z_act) && double.TryParse(parts[7], out z_tar) &&

                    double.TryParse(parts[8], out c_act) && double.TryParse(parts[9], out c_tar))

                {

                    UpdateDualChart(chart1, x_act, x_tar);

                    UpdateDualChart(chart2, y_act, y_tar);

                    UpdateDualChart(chart3, z_act, z_tar);

                    UpdateDualChart(chart4, c_act, c_tar);

                }

            }

        }



        private void UpdateDualChart(Chart chart, double actual, double target)

        {

            // Series[0]: Actual

            chart.Series[0].Points.AddY(actual);

            if (chart.Series[0].Points.Count > MAX_DATA_POINTS) chart.Series[0].Points.RemoveAt(0);



            // Series[1]: Target

            chart.Series[1].Points.AddY(target);

            if (chart.Series[1].Points.Count > MAX_DATA_POINTS) chart.Series[1].Points.RemoveAt(0);

        }







        private void Form1_Load(object sender, EventArgs e)

        {



        }



        private void exitToolStripMenuItem_Click(object sender, EventArgs e)

        {

            Application.Exit();

        }





        private void btnStop_Click(object sender, EventArgs e)

        {

            if (serialPort.IsOpen)

            {

                serialPort.Write("S");

            }

            else

            {

                MessageBox.Show("Serial port was Closed", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);

            }

        }



        private void btnCurrentPatient_Click(object sender, EventArgs e)

        {

            if (serialPort.IsOpen)

            {

                serialPort.Write("P");

            }

            else

            {

                MessageBox.Show("Serial port was Closed", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);

            }

        }



        private void chkRepeat_CheckedChanged(object sender, EventArgs e)

        {

            if (serialPort.IsOpen)

            {

                serialPort.Write("R");

            }

        }







        private void btnMotorx_Click(object sender, EventArgs e)

        {

            SendCommand(1, txtMotorx.Text);

        }



        private void btnMotory_Click(object sender, EventArgs e)

        {

            SendCommand(2, txtMotory.Text);

        }



        private void btnMotorz_Click(object sender, EventArgs e)

        {

            SendCommand(3, txtMotorz.Text);

        }



        private void btnMotorc_Click(object sender, EventArgs e)

        {

            SendCommand(4, txtMotorc.Text);

        }





        private void SendCommand(int motorIndex, string stepText)

        {

            if (serialPort.IsOpen && int.TryParse(stepText, out int steps))

            {

                // protocol: "M" + motor No. + " " + step count (예: "M1 100")

                string command = $"M{motorIndex} {steps}";

                serialPort.WriteLine(command);

            }

            else

            {

                MessageBox.Show("유효한 숫자를 입력하거나 연결을 확인하세요.");

            }

        }





        private void loadToolStripMenuItem_Click(object sender, EventArgs e)

        {

            //if (!serialPort.IsOpen)

            //{

            //    MessageBox.Show("Serial port was Closed", "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);

            //    return;

            //}



            //openFileDialog1.Filter = "CSV Files (*.csv)|*.csv|All Files (*.*)|*.*";

            //if (openFileDialog1.ShowDialog() == DialogResult.OK)

            //{

            //    string filePath = openFileDialog1.FileName;

            //    string fileName = openFileDialog1.SafeFileName;



            //    lblPatientName.Text = fileName;

            //    LoadAndUploadCsv(filePath);

            //}



            openFileDialog1.Filter = "CSV Files (*.csv)|*.csv|All Files (*.*)|*.*";

            if (openFileDialog1.ShowDialog() == DialogResult.OK)

            {

                LoadCsvToMemory(openFileDialog1.FileName);

            }



        }





        // ================================================================

        // file load for csv

        // ================================================================



        private void LoadCsvToMemory(string path)

        {

            try

            {

                string[] lines = File.ReadAllLines(path);

                rawDataList.Clear(); // 기존 데이터 비우기



                foreach (string line in lines)

                {

                    string[] parts = line.Split(',');

                    if (parts.Length >= 4)

                    {

                        int v1, v2, v3, v4;

                        // 정수로 변환하여 리스트에 원본 저장

                        if (int.TryParse(parts[0], out v1) && int.TryParse(parts[1], out v2) &&

                            int.TryParse(parts[2], out v3) && int.TryParse(parts[3], out v4))

                        {

                            rawDataList.Add(new int[] { v1, v2, v3, v4 });

                        }

                    }

                }



                if (rawDataList.Count > 0)

                {

                    lblPatientName.Text = Path.GetFileName(path); // 파일명 표시

                    MessageBox.Show($"File Loaded! ({rawDataList.Count} points)\nNow set the scale and click 'Upload'.");



                    // 로드 직후 자동으로 업로드까지 하고 싶다면 아래 주석 해제

                    // btnUpload_Click(null, null); 

                }

                else

                {

                    MessageBox.Show("No valid data found in CSV.");

                }

            }

            catch (Exception ex)

            {

                MessageBox.Show("File Load Error: " + ex.Message);

            }

        }



        // =============================================================

        // [2] 데이터 업로드 (계수 적용 및 전송) - btnUpload_Click 이벤트에 연결

        // =============================================================

        private void btnUpload_Click(object sender, EventArgs e)

        {

            // 1. 데이터 확인

            if (rawDataList.Count == 0)

            {

                MessageBox.Show("Please load a CSV file first.");

                return;

            }



            if (!serialPort.IsOpen)

            {

                MessageBox.Show("Please connect to Arduino first.");

                return;

            }



            // 2. 보정 계수 가져오기 (소수점 인식 강화)

            // CultureInfo.InvariantCulture를 사용하여 무조건 점(.)을 소수점으로 인식하게 함

            double sX = ParseDouble(txtScaleX.Text);

            double sY = ParseDouble(txtScaleY.Text);

            double sZ = ParseDouble(txtScaleZ.Text);

            double sC = ParseDouble(txtScaleC.Text);



            // 3. 전송 데이터 생성 (Raw * Scale) -> 정수(Step)로 변환

            List<string> packetsToSend = new List<string>();

            foreach (var raw in rawDataList)

            {

                // 소수점 곱셈 후 반올림하여 정수로 변환 (정확도 향상)

                int val1 = (int)Math.Round(raw[0] * sX);

                int val2 = (int)Math.Round(raw[1] * sY);

                int val3 = (int)Math.Round(raw[2] * sZ);

                int val4 = (int)Math.Round(raw[3] * sC);



                packetsToSend.Add($"D,{val1},{val2},{val3},{val4}");

            }



            // 4. 업로드 프로세스 시작

            // 로그에 적용된 계수를 찍어서 확인시켜줌

            lbConsole.Items.Add($"[Scale Applied] X:{sX} Y:{sY} Z:{sZ} C:{sC}");

            UploadPackets(packetsToSend, sX, sY, sZ, sC);

        }



        // [보조 함수] 안전한 소수점 변환기

        private double ParseDouble(string text)

        {

            double result = 1.0;

            // 공백 제거 및 소수점 처리

            if (double.TryParse(text, System.Globalization.NumberStyles.Any, System.Globalization.CultureInfo.InvariantCulture, out result))

            {

                return result;

            }

            return 1.0; // 실패 시 기본값 1.0

        }



        // 실제 전송 로직 (기존 LoadAndUploadCsv의 뒷부분을 분리)

        private void UploadPackets(List<string> dataPackets, double sX, double sY, double sZ, double sC)

        {

            progressBar1.Visible = true;

            progressBar1.Maximum = dataPackets.Count;

            progressBar1.Value = 0;

            if (lblLoadMotionCount != null) lblLoadMotionCount.Text = "Uploading...";



            serialPort.DataReceived -= SerialPort_DataReceived; // 이벤트 잠시 해제

            int originalTimeout = serialPort.ReadTimeout;

            serialPort.ReadTimeout = 3000;



            try

            {

                lbConsole.Items.Add($"--- Upload Start (Scale: {sX}, {sY}, {sZ}, {sC}) ---");

                serialPort.DiscardInBuffer();

                Thread.Sleep(50);



                // L 명령 (재시도 로직)

                bool ready = false;

                for (int i = 0; i < 3; i++)

                {

                    serialPort.Write("L");

                    string res = ReadLineToConsole();

                    if (res.Contains("SYSTEM")) res = ReadLineToConsole(); // 부팅 메시지 스킵



                    if (res.Contains("READY")) { ready = true; break; }

                    Thread.Sleep(100);

                    serialPort.DiscardInBuffer();

                }



                if (!ready) throw new Exception("No READY response from Arduino.");



                // 데이터 전송

                foreach (string packet in dataPackets)

                {

                    serialPort.WriteLine(packet);

                    string ack = ReadLineToConsole();



                    if (!ack.Contains("K"))

                    {

                        ack = ReadLineToConsole(); // 재시도

                        if (!ack.Contains("K")) throw new Exception($"Ack Failed. Msg: {ack}");

                    }

                    progressBar1.Value++;

                    if (progressBar1.Value % 10 == 0) Application.DoEvents();

                }



                // 종료

                serialPort.Write("E");

                string doneMsg = ReadLineToConsole();



                // 결과 표시

                int count = dataPackets.Count;

                if (doneMsg.Contains("DONE:")) int.TryParse(doneMsg.Replace("DONE:", ""), out count);



                double timeSec = count * 0.1;



                this.Invoke(new Action(() => {

                    if (lblLoadMotionCount != null) lblLoadMotionCount.Text = $"Points: {count} | Time: {timeSec:F1}s";

                }));



            }

            catch (Exception ex)

            {

                lbConsole.Items.Add("Err: " + ex.Message);

                MessageBox.Show("Upload Failed: " + ex.Message);

            }

            finally

            {

                serialPort.ReadTimeout = originalTimeout;

                serialPort.DataReceived += SerialPort_DataReceived;

                progressBar1.Visible = false;

                lbConsole.Items.Add($"[Applied Axis Coefficients] X:{sX} Y:{sY} Z:{sZ} C:{sC}");

                lbConsole.Items.Add("--- Upload End ---");

                lbConsole.TopIndex = lbConsole.Items.Count - 1;

            }

        }



        private string ReadLineToConsole()

        {

            try

            {

                string line = serialPort.ReadLine().Trim();

                lbConsole.Items.Add("[RX_LOAD] " + line);

                lbConsole.TopIndex = lbConsole.Items.Count - 1;

                return line;

            }

            catch

            {

                lbConsole.Items.Add("[RX_ERR] Timeout");

                return "TIMEOUT";

            }

        }



        //private void LoadAndUploadCsv(string path)

        //{

        //    try

        //    {

        //        string[] lines = File.ReadAllLines(path);

        //        List<string> uploadData = new List<string>();



        //        foreach (string line in lines)

        //        {

        //            string[] parts = line.Split(',');

        //            if (parts.Length >= 4)

        //                uploadData.Add($"D,{parts[0]},{parts[1]},{parts[2]},{parts[3]}");

        //        }



        //        if (uploadData.Count == 0) { AddToLog("No data in CSV"); return; }



        //        progressBar1.Visible = true;

        //        progressBar1.Maximum = uploadData.Count;

        //        progressBar1.Value = 0;



        //        lblLoadMotionCount.Text = "Uploading...";



        //        // 타임아웃 오류 방지를 위해 잠시 이벤트 핸들러 해제 (이 함수에서 직접 읽기 위해)

        //        serialPort.DataReceived -= SerialPort_DataReceived;

        //        int originalTimeout = serialPort.ReadTimeout;

        //        serialPort.ReadTimeout = 1000;



        //        try

        //        {

        //            AddToLog("=== Upload Start ===");



        //            // 버퍼 비우기

        //            serialPort.DiscardInBuffer();

        //            Thread.Sleep(50);



        //            // 1. L 명령 전송

        //            serialPort.Write("L");

        //            AddToLog("[TX] L");



        //            // 2. READY 대기 (직접 읽어서 로그에 찍음)

        //            string response = ReadWithLog();

        //            // 혹시 엔코더 데이터나 SYSTEM_READY가 껴있을 수 있으므로 "READY" 찾을 때까지 몇 번 읽음

        //            for (int i = 0; i < 10; i++)

        //            {

        //                if (response.Contains("READY")) break;

        //                response = ReadWithLog();

        //            }



        //            if (!response.Contains("READY"))

        //            {

        //                throw new Exception($"Failed to receive READY. Last received: {response}");

        //            }



        //            // 3. 데이터 전송

        //            foreach (string packet in uploadData)

        //            {

        //                serialPort.WriteLine(packet);

        //                // AddToLog("[TX] " + packet); // 너무 많으면 주석 처리



        //                string ack = ReadWithLog(); // K 대기

        //                if (!ack.Contains("K"))

        //                {

        //                    // 한번 더 기회 줌 (엔코더 데이터 간섭 가능성)

        //                    ack = ReadWithLog();

        //                    if (!ack.Contains("K")) throw new Exception($"Ack Failed. RX: {ack}");

        //                }



        //                progressBar1.Value++;

        //                if (progressBar1.Value % 10 == 0) Application.DoEvents();

        //            }



        //            // 4. 종료

        //            serialPort.Write("E");

        //            AddToLog("[TX] E");

        //            string doneMsg = ReadWithLog();



        //            int count = 0;



        //            // "DONE:숫자" 파싱

        //            if (doneMsg.Contains("DONE:"))

        //            {

        //                string countStr = doneMsg.Replace("DONE:", "").Trim();

        //                int.TryParse(countStr, out count);

        //            }

        //            else

        //            {

        //                count = uploadData.Count; // 응답 없으면 보낸 개수로 대체

        //            }



        //            // 시간 계산 (개수 * 100ms)

        //            double totalSeconds = count * 0.1;



        //            // 라벨에 표시 (예: "Points: 200 | Time: 20.0s")

        //            this.Invoke(new Action(() =>

        //            {

        //                lblLoadMotionCount.Text = $"Count: {count} | Time: {totalSeconds:F1}s";

        //            }));



        //            string resultMsg = $"Upload Complete!\ncount: {count}\nTime: {totalSeconds:F1} sec";

        //            lbConsole.Items.Add("[INFO] " + resultMsg.Replace("\n", " "));

        //            //MessageBox.Show(resultMsg);

        //        }

        //        catch (Exception ex)

        //        {

        //            AddToLog("Upload Error: " + ex.Message);

        //            MessageBox.Show("Error: " + ex.Message);

        //        }

        //        finally

        //        {

        //            // 복구

        //            serialPort.ReadTimeout = originalTimeout;

        //            serialPort.DataReceived += SerialPort_DataReceived; // 이벤트 핸들러 복구

        //            progressBar1.Visible = false;

        //            AddToLog("=== Upload Finish ===");

        //        }

        //    }

        //    catch (Exception ex) { MessageBox.Show(ex.Message); }

        //}







        private void btnReset_Click(object sender, EventArgs e)

        {

            if (!serialPort.IsOpen) return;



            try

            {

                lbConsole.Items.Add("[System] Resetting Arduino...");



                // DTR 토글로 하드웨어 리셋 유도 (Uno/Nano/Mega 기준)

                serialPort.DtrEnable = false;

                Thread.Sleep(100);

                serialPort.DtrEnable = true;



                // 리셋되었으므로 다시 부팅 대기 모드로 진입

                timerUsage.Stop(); // 시간 측정 일시 정지

                lblTimeState.Text = "Resetting...";



                progressBar1.Value = 0;

                progressBar1.Visible = true;

                chkRepeat.Checked = false;



                connectionProgress = 0;

                timerConnect.Start(); // 부팅 대기 타이머 재가동

            }

            catch (Exception ex)

            {

                MessageBox.Show("Reset Error: " + ex.Message);

            }

        }



    }

}
