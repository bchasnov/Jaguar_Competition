﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.IO;
using System.Net.Sockets;
using System.Net;

namespace DrRobot.JaguarControl
{
    public class Navigation
    {
        private DateTime globalLoopTime;

        #region Navigation Variables
        public long[] LaserData = new long[DrRobot.JaguarControl.JaguarCtrl.DISDATALEN];
        public double initialX = 0, initialY = 0, initialT = -Math.PI/2;
        public double x, y, t;
        public double x_est, y_est, t_est;
        public double x_des, y_des, t_des;
        public double x_gps, y_gps, t_gps;
        public double desiredX, desiredY, desiredT;

        public double currentEncoderPulseL, currentEncoderPulseR;
        public double lastEncoderPulseL, lastEncoderPulseR;
        public double wheelDistanceR, wheelDistanceL;
        public double tiltAngle, zoom;
        public double currentAccel_x, currentAccel_y, currentAccel_z;
        public double lastAccel_x, lastAccel_y, lastAccel_z;
        public double currentGyro_x, currentGyro_y, currentGyro_z;
        public double last_v_x, last_v_y;
        public double filteredAcc_x, filteredAcc_y;

        public int robotType, controllerType;
        enum ROBOT_TYPE { SIMULATED, REAL };
        enum CONTROLLERTYPE { MANUALCONTROL, POINTTRACKER, EXPERIMENT };
        public bool motionPlanRequired, displayParticles, displayNodes, displaySimRobot;
        private JaguarCtrl jaguarControl;
        private AxDRROBOTSentinelCONTROLLib.AxDDrRobotSentinel realJaguar;
        private AxDDrRobotSentinel_Simulator simulatedJaguar;
        private Thread controlThread;
        private short motorSignalL, motorSignalR;
        private short desiredRotRateR, desiredRotRateL;
        public bool runThread = true;
        public bool loggingOn;
        StreamWriter logFile;
        public int deltaT = 10;
        private static int encoderMax = 32767;
        public int pulsesPerRotation = 190;
        public double wheelRadius = 0.089*0.89;
        public double robotRadius = 0.242*1.4;//0.232
        private double angleTravelled, distanceTravelled;
        private double diffEncoderPulseL, diffEncoderPulseR;
        private double maxVelocity = 0.25;
        public double Kpho = 1;
        public double Kalpha = 0.51;//8
        public double Kbeta = -0.5;//-0.5//-1.0;
        const double alphaTrackingAccuracy = 0.10;
        const double betaTrackingAccuracy = 0.1;
        const double phoTrackingAccuracy = 0.10;
        double time = 0;
        DateTime startTime;

        public double K_P = 15;//15;
        public double K_I = 0;//0;
        public double K_D = 3;//3;
        public short frictionComp = 8750;//8750;
        public double e_sum_R, e_sum_L;
        public double u_R = 0;
        public double u_L = 0;
        public double e_R = 0;
        public double e_L = 0;

        public double rotRateL, rotRateR;
        public double K_p, K_i, K_d, maxErr;

        public double accCalib_x = 18;
        public double accCalib_y = 4;



        public Map map;
        public Particle[] particles;
        //public Particle[] propagatedParticles;
        public Particle[] tempParticles;
        public int tempParticlesCount = 0;
        public int numParticles = 10000;
        public double K_wheelRandomness = 0.15;//0.25
        public Random random = new Random();
        public bool newLaserData = false;
        public double laserMaxRange = 4.0;
        public double laserMinRange = 0.2;
        public double[] laserAngles;
        private int laserCounter;
        private int laserStepSize = 3;
        private int newLaserCounter = 0;
        int redistributeFactor = 4;

        public Boolean correctionOverrideEnabled = false;
        public Boolean correctionOverride = false;

        public class Particle
        {
            public double x, y, t, w;

            public Particle()
            {
            }
        }

        // Motion Planner Variables
        const int numXCells = 20;
        const int numYCells = 20;
        const int maxNumNodes = 5000;
        const float minWorkspaceX = -10.0f;
        const float maxWorkspaceX = 10.0f;
        const float minWorkspaceY = -10.0f;
        const float maxWorkspaceY = 10.0f;

        // Motion Planner Variables 
        public double samplingCellSizeX, samplingCellSizeY;
        public int numOccupiedCells;
        public int[] occupiedCellsList;
        public int[] numNodesInCell;
        public Node[,] NodesInCells;
        public Node[] trajList, nodeList;
        public int trajSize, trajCurrentNode, numNodes;

        IPhoneSensor iPhoneSensor = new IPhoneSensor(5555);
        Gps myGps;

        public class Node
        {
            public double x, y;
            public int lastNode;
            public int nodeIndex;

            public Node()
            {
                x = 0;
                y = 0;
                lastNode = 0;
                nodeIndex = 0;
            }

            public Node(double _x, double _y, int _nodeIndex, int _lastNode)
            {
                x = _x;
                y = _y;
                nodeIndex = _nodeIndex;
                lastNode = _lastNode;
            }
        }


        #endregion

        DateTime prevTime = DateTime.Now;
        void ttime(string str)
        {
            DateTime tmpTime = prevTime;
            prevTime = DateTime.Now;
            Console.WriteLine(">>" + str);
            Console.WriteLine(">>t=" + (prevTime - tmpTime).Milliseconds);
        }



        #region Navigation Setup
        
        // Constructor for the Navigation class
        public Navigation(JaguarCtrl jc)
        {
            // Initialize vars
            jaguarControl = jc;
            myGps = new Gps(jc);
            realJaguar = jc.realJaguar;
            simulatedJaguar = jc.simulatedJaguar;
            map = new Map();
            particles = new Particle[numParticles];

            tempParticles = new Particle[numParticles * redistributeFactor];
            // Create particles


            for (int i = 0; i < numParticles; i++)
            {
                particles[i] = new Particle();
                //propagatedParticles[i] = new Particle();
            }

            for (int i = 0; i < numParticles * redistributeFactor; i++)
            {
                tempParticles[i] = new Particle();
            }

            this.Initialize();


            // Start Control Thread
            controlThread = new Thread(new ThreadStart(runControlLoop));
            controlThread.Start();
        }

        // All class variables are initialized here
        // This is called every time the reset button is pressed
        public void Initialize()
        {
            // Initialize state estimates
            if (!map.trajectory.empty())
            {
                JagPoint start = map.trajectory.getTargetPoint();
                
                initialX = start.x;
                initialY = start.y;
                initialT = start.theta;

                desiredX = initialX;
                desiredY = initialY;
                desiredT = initialT;

                x_des = initialX;
                y_des = initialY;
                t_des = initialT;
            }
            //if (iPhoneSensor.newCompass) t = iPhoneSensor.Theta();

            x = initialX;
            y = initialY;
            t = initialT;

            // Initialize state estimates
            x_est = x;
            y_est = y;
            t_est = t;

            // Set desired state
            desiredX = 0;// initialX;
            desiredY = 0;// initialY;
            desiredT = 0;// initialT;

            // Reset Localization Variables
            wheelDistanceR = 0;
            wheelDistanceL = 0;

            // Zero actuator signals
            motorSignalL = 0;
            motorSignalR = 0;
            loggingOn = false;

            // Set random start for particles
            InitializeParticles();

            // Set default to no motionPlanRequired
            motionPlanRequired = false;

            // Set visual display
            tiltAngle = 25.0;
            displayParticles = true;
            displayNodes = true;
            displaySimRobot = true;

            laserAngles = new double[LaserData.Length];
            for (int i = 0; i < LaserData.Length; i++)                
                laserAngles[i] = DrRobot.JaguarControl.JaguarCtrl.startAng + DrRobot.JaguarControl.JaguarCtrl.stepAng * i;

            // MP variable setup
            occupiedCellsList = new int[numXCells * numYCells];
            numNodesInCell = new int[numXCells * numYCells];
            NodesInCells = new Node[numXCells * numYCells, 500];
            trajList = new Node[maxNumNodes];
            nodeList = new Node[maxNumNodes];
            numNodes = 0;
            trajList[0] = new Node(0, 0, 0, 0);
            trajSize = 0;


        }

        // This function is called from the dialogue window "Reset Button"
        // click function. It resets all variables.
        public void Reset()
        {
            simulatedJaguar.Reset();
            GetFirstEncoderMeasurements();
            //CalibrateIMU();
            Initialize();
        }
        #endregion


        #region Main Loop

        /************************ MAIN CONTROL LOOP ***********************/
        // This is the main control function called from the control loop
        // in the RoboticsLabDlg application. This is called at every time
        // step.
        // Students should choose what type of localization and control 
        // method to use. 
        public void runControlLoop()
        {
            // Wait
            Thread.Sleep(100);

            // Don't run until we have gotten our first encoder measurements to difference with
            GetFirstEncoderMeasurements();


            // Run infinite Control Loop
            while (runThread)
            {
                //Console.WriteLine(x_gps +" " + y_gps);
                //t = iPhoneSensor.Theta();
                globalLoopTime = DateTime.Now;

                // ****************** Additional Student Code: Start ************

                // Students can select what type of localization and control
                // functions to call here. For lab 1, we just call the function
                // WallPositioning to have the robot maintain a constant distance
                // to the wall (see lab manual).
                
                // Update Sensor Readings
                UpdateSensorMeasurements();

                // Determine the change of robot position, orientation (lab 2)	
                Boolean moved = MotionPrediction();

                if (moved || correctionOverride)
                {
                    Console.WriteLine("Tick");
                    // Update the global state of the robot - x,y,t (lab 2)
                    LocalizeRealWithOdometry();
                    // Update the global state of the robot - x,y,t (lab 2)
                    //LocalizeRealWithIMU();


                    //Console.WriteLine(">"+x+","+y+","+t+","+map.GetClosestWallDistance(x, y, t));



                    // Estimate the global state of the robot -x_est, y_est, t_est (lab 4)
                    LocalizeEstWithParticleFilter();
                }


                // If using the point tracker, call the function
                if (jaguarControl.controlMode == jaguarControl.AUTONOMOUS)
                {
                    if (jaguarControl.autoMode == jaguarControl.AUTO_TRACKTRAJ)
                    {
                        TrackTrajectoryManual();
                    }
                    // Check if we need to create a new trajectory
                    if (false)//motionPlanRequired)
                    {
                        // Construct a new trajectory (lab 5)
                        PRMMotionPlanner();
                        motionPlanRequired = false;
                    }

                    // Drive the robot to a desired Point (lab 3)
                    t_des = desiredT;
                    x_des = desiredX;
                    y_des = desiredY;

                    FlyToSetPoint();

                    // Follow the trajectory instead of a desired point (lab 3)
                    TrackTrajectory();

                    // Actuate motors based actuateMotorL and actuateMotorR
                    if (jaguarControl.Simulating())
                    {
                        CalcSimulatedMotorSignals();
                        ActuateMotorsWithVelControl();
                    }
                    else 
                    {
                        // Determine the desired PWM signals for desired wheel speeds
                        CalcMotorSignals_ben();
                        ActuateMotorsWithPWMControl();
                    }

                }
                else
                {
                    e_sum_L = 0;
                    e_sum_R = 0;
                }
                
                // ****************** Additional Student Code: End   ************

                // Log data
                LogData();

                // Sleep to approximate 20 Hz update rate
                while ((DateTime.Now - globalLoopTime).Milliseconds < deltaT)
                {
                    Thread.Sleep(1);
                }
            }
        }


        public void CalibrateIMU()
        {

            accCalib_x = 0;
            accCalib_y = 0;
            int numMeasurements = 100;
            for (int i = 0; i < numMeasurements; i++)
            {
                accCalib_x += currentAccel_x;
                accCalib_y += currentAccel_y;

                Thread.Sleep(deltaT);
            }
            accCalib_x = accCalib_x / numMeasurements;
            accCalib_y = accCalib_y /numMeasurements;


        }


        // Before starting the control loop, the code checks to see if 
        // the robot needs to get the first encoder measurements
        public void GetFirstEncoderMeasurements()
        {
            if (!jaguarControl.Simulating())
            {
                // Get last encoder measurements
                bool gotFirstEncoder = false;
                int counter = 0;
                while (!gotFirstEncoder && counter < 10)
                {
                    try
                    {
                        currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                        currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();
                        lastEncoderPulseL = currentEncoderPulseL;
                        lastEncoderPulseR = currentEncoderPulseR;
                        gotFirstEncoder = true;

                        currentAccel_x = jaguarControl.getAccel_x();
                        currentAccel_y = jaguarControl.getAccel_y();
                        currentAccel_z = jaguarControl.getAccel_z();
                        lastAccel_x = currentAccel_x;
                        lastAccel_y = currentAccel_y;
                        lastAccel_z = currentAccel_z;
                        last_v_x = 0;
                        last_v_y = 0;

                    }
                    catch (Exception e) { }
                    counter++;
                    Thread.Sleep(100);
                }
            }
            else
            {
                currentEncoderPulseL = 0;
                currentEncoderPulseR = 0;
                lastEncoderPulseL = 0;
                lastEncoderPulseR = 0;
                lastAccel_x = 0;
                lastAccel_y = 0;
                lastAccel_z = 0;
                last_v_x = 0;
                last_v_y = 0;

            }
        }





        // At every iteration of the control loop, this function will make 
        // sure all the sensor measurements are up to date before
        // makeing control decisions.
        public void UpdateSensorMeasurements()
        {
            iPhoneSensor.updateReadings();
            if (iPhoneSensor.newCompass)
            {
                t_gps = iPhoneSensor.Theta();
            }
            if (false)//iPhoneSensor.newGps)
            {
                x_gps = iPhoneSensor.GPSx();
                y_gps = iPhoneSensor.GPSy();
            }

            myGps.updateGps();
            x_gps = myGps.getX();
            y_gps = myGps.getY();

            Console.WriteLine(iPhoneSensor.latitude + " " + iPhoneSensor.longitude + " " + x_gps + " " + y_gps);

            // For simulations, update the simulated measurements
            if (jaguarControl.Simulating())
            {
                jaguarControl.simulatedJaguar.UpdateSensors(deltaT);

                // Get most recenct encoder measurements
                currentEncoderPulseL = simulatedJaguar.GetEncoderPulse4();
                currentEncoderPulseR = simulatedJaguar.GetEncoderPulse5();

                // Get most recent laser scanner measurements
                laserCounter = laserCounter + deltaT;
                if (laserCounter >= 2000)
                {
                    for (int i = 0; i < LaserData.Length; i=i+laserStepSize)
                    {
                        LaserData[i] = (long)(1000 * map.GetClosestWallDistance(x, y, t -1.57 + laserAngles[i]));
                    }
                    laserCounter = 0;
                    newLaserData = true;
                }
            }
            else
            {
                // Get most recenct encoder measurements
                try
                {

                    // Update IMU Measurements
                    currentAccel_x = jaguarControl.getAccel_x();
                    currentAccel_y = jaguarControl.getAccel_y();
                    currentAccel_z = jaguarControl.getAccel_z();
                   
                    // Update Encoder Measurements
                    currentEncoderPulseL = jaguarControl.realJaguar.GetEncoderPulse4();
                    currentEncoderPulseR = jaguarControl.realJaguar.GetEncoderPulse5();

                }
                catch (Exception e)
                {
                }
            }
        }

        // At every iteration of the control loop, this function calculates
        // the PWM signal for corresponding desired wheel speeds
        public void CalcSimulatedMotorSignals()
        {

            motorSignalL = (short)(desiredRotRateL);
            motorSignalR = (short)(desiredRotRateR);

        }
        public void CalcMotorSignals_josh()
        {
            Console.WriteLine("calcmotorsignals");
            short zeroOutput = 16383;
            short maxPosOutput = 32767;

            // We will use the desiredRotRateRs to set our PWM signals
            int cur_e_R = (int)((double)desiredRotRateR - (diffEncoderPulseR / deltaT));   // [pulses/second]
            int cur_e_L = (int)((double)desiredRotRateL - (diffEncoderPulseL / deltaT));
            int e_dir_R = (int)((cur_e_R - e_R) / deltaT);
            int e_dir_L = (int)((cur_e_L - e_L) / deltaT);
            e_R = cur_e_R;
            e_L = cur_e_L;

            int maxErr = (int)(3000 / deltaT);

            //double K_p = 0.1;//1
            //double K_i = 12; // deltaT;//20
            //double K_d = 100.1;

            //Krho = 1.0;             // Krho > 0 (Conditions for stability)
            //Kalpha = 3;//4          // Kalpha - Krho > 0
            //Kbeta = -2;//-1.0;    // Kbeta < 0

            u_R = K_P * e_R + K_I * e_sum_R + K_D * e_dir_R;
            u_L = K_P * e_L + K_I * e_sum_L + K_D * e_dir_L;

            motorSignalL = (short)(zeroOutput - u_L); //* 300);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R); //* 100);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            e_sum_R = Math.Max(-maxErr, Math.Min(0.90 * e_sum_R + e_R * deltaT, maxErr));  // clever.
            e_sum_L = Math.Max(-maxErr, Math.Min(0.90 * e_sum_L + e_L * deltaT, maxErr));

        }


        public void CalcMotorSignals_ben()
        {

            Console.WriteLine("{0} {1}", desiredRotRateL, desiredRotRateR);

            short zeroOutput = 16383;
            short maxPosOutput = 32767;
            // desiredRotRateR = 10;
            // desiredRotRateL = 10;
            double deltaTinS = (double)deltaT / 1000;
            // We will use the desiredRotRateRs to set our PWM signals
            //desiredRotRateR = 600;
            //desiredRotRateL = 600;

            double currentRotRateR = -jaguarControl.rightFrontWheelMotor.encodeSpeed * jaguarControl.rightFrontWheelMotor.encoderDir;//(diffEncoderPulseR / (count * deltaTinS));
            double currentRotRateL = jaguarControl.leftFrontWheelMotor.encodeSpeed * jaguarControl.leftFrontWheelMotor.encoderDir;//(diffEncoderPulseL / (count * deltaTinS));

            double cur_e_R = desiredRotRateR - currentRotRateR; //in units of rad per second
            double cur_e_L = desiredRotRateL - currentRotRateL;

            int e_dir_R = (int)(cur_e_R - e_R);
            int e_dir_L = (int)(cur_e_L - e_L);
            e_R = cur_e_R;
            e_L = cur_e_L;

            int maxErr = (int)(3000 / deltaTinS);


            K_p = K_P;// 0.1;//1
            K_i = K_I; // 0;//12 / deltaT;//20
            K_d = K_D;// 0;// 100.1;

            //Kpho = 1.5;
            //Kalpha = 8;//4
            //Kbeta = -0.8;//-1.0;


            u_R = K_p * desiredRotRateR;//e_R + K_i * e_sum_R + K_d * e_dir_R;
            u_L = K_p * desiredRotRateL;//e_L + K_i * e_sum_L + K_d * e_dir_L;



            if (desiredRotRateL == 0)
            {
                u_L = 0;
            }
            if (desiredRotRateR == 0)
            {
                u_R = 0;
            }


            motorSignalL = (short)(zeroOutput + u_L);//(zeroOutput + desiredRotRateL * 300);// (zeroOutput + u_L);
            motorSignalR = (short)(zeroOutput - u_R);// (zeroOutput - desiredRotRateR * 100);//(zeroOutput - u_R);

            motorSignalL = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalL));
            motorSignalR = (short)Math.Min(maxPosOutput, Math.Max(0, (int)motorSignalR));

            e_sum_R = Math.Max(-maxErr, Math.Min(0.90 * e_sum_R + e_R * deltaTinS, maxErr));
            e_sum_L = Math.Max(-maxErr, Math.Min(0.90 * e_sum_L + e_L * deltaTinS, maxErr));

            //Console.WriteLine(K_p + " " + cur_e_R + " " + currentRotRateR + "desired: " + desiredRotRateR.ToString() + " diffEncoderPulseR:" + diffEncoderPulseR + " diff/s:" + diffEncoderPulseR / (count * deltaTinS) + " cur_e_R: " + cur_e_R + " u_R:" + u_R + " motorSignalR:" + motorSignalR);

            /*aa = u_R;
            bb = diffEncoderPulseR;
            cc = diffEncoderPulseR / deltaTinS;
            dd = desiredRotRateR;
            ee = motorSignalR;*/

            //count = 1;
        }


        // At every iteration of the control loop, this function sends
        // the width of a pulse for PWM control to the robot motors
        public void ActuateMotorsWithPWMControl()
        { 
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            else
            {
                jaguarControl.realJaguar.DcMotorPwmNonTimeCtrAll(0, 0, 0, motorSignalL, motorSignalR, 0);
            }
        }

        // At every iteration of the control loop, this function sends
        // desired wheel velocities (in pulses / second) to the robot motors
        public void ActuateMotorsWithVelControl()
        {
            if (jaguarControl.Simulating())
                simulatedJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
            else
                jaguarControl.realJaguar.DcMotorVelocityNonTimeCtrAll(0, 0, 0, motorSignalL, (short)(-motorSignalR), 0);
        }
        #endregion


        #region Logging Functions

        // This function is called from a dialogue window "Record" button
        // It creates a new file and sets the logging On flag to true
        public void TurnLoggingOn()
        {
            //int fileCnt= 0;
            String date = DateTime.Now.Year.ToString() + "-" + DateTime.Now.Month.ToString() + "-" + DateTime.Now.Day.ToString() + "-" + DateTime.Now.Minute.ToString();
            ToString();
            logFile = File.CreateText("JaguarData_" + date + ".txt");
            startTime = DateTime.Now;
            loggingOn = true;
        }

        // This function is called from a dialogue window "Record" button
        // It closes the log file and sets the logging On flag to false
        public void TurnLoggingOff()
        {
            if (logFile != null)
                logFile.Close();
            loggingOn = false;
        }

        // This function is called at every iteration of the control loop
        // IF the loggingOn flag is set to true, the function checks how long the 
        // logging has been running and records this time
        private void LogData()
        {
            if (loggingOn)
            {
                TimeSpan ts = DateTime.Now - startTime;
                time = ts.TotalSeconds;
                String newData = time.ToString() + " " + x_gps + " " + y_gps + " " + iPhoneSensor.latitude + " " + iPhoneSensor.longitude;//x.ToString() + " " + y.ToString() + " " + t.ToString() ;

               

                logFile.WriteLine(newData);
            }
        }
        #endregion


        # region Control Functions

        // This function is called at every iteration of the control loop
        // It will drive the robot forward or backward to position the robot 
        // 1 meter from the wall.
        private void WallPositioning()
        {

            // Here is the distance measurement for the central laser beam 
            double centralLaserRange = LaserData[113];

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated motorSignalR and 
            // motorSignalL. Make sure the robot does not exceed 
            // maxVelocity!!!!!!!!!!!!

            // Send Control signals, put negative on left wheel control

 

            // ****************** Additional Student Code: End   ************                
        }


        // This function is called at every iteration of the control loop
        // if used, this function can drive the robot to any desired
        // robot state. It does not check for collisions
        private void FlyToSetPoint()
        {

            // ****************** Additional Student Code: Start ************
            // ****************** Additional Student Code: Start ************
            int dir = 1;

            double dx = x_des - x;
            double dy = y_des - y;
            

            double a = -1.0 * t + Math.Atan2(dy, dx);
            if (a < -Math.PI / 2 || a > Math.PI / 2)
            {
                dir = -1;
                a = -1.0 * t + Math.Atan2(-dy, -dx);
            }

            double p = Math.Sqrt(Math.Pow(dx, 2) + Math.Pow(dy, 2)); //distance away from setpoint
            double b = -1.0 * t - a + t_des;

            double v = Kpho * p; //set velocity to v (m/s)
            v = Math.Min(maxVelocity, v);
            v = dir * v;
            double w = Kalpha * a + Kbeta * b; //set rotation to w




            double w2 = 0.5 * (w + v / robotRadius); //Left rotation
            double w1 = 0.5 * (w - v / robotRadius); //Right rotation

            double phi1 = -1.0 * robotRadius * w1 / wheelRadius;
            double phi2 = robotRadius * w2 / wheelRadius;

            double pulsePerMeter = pulsesPerRotation / (wheelRadius * 2 * Math.PI);


            desiredRotRateL = (short)(phi1 * pulsePerMeter);
            desiredRotRateR = (short)(phi2 * pulsePerMeter);



            // ****************** Additional Student Code: End   ************
        }

        public Boolean initFlyToSetPoint;
        public int dir;

        public void setSetPoint(double xx, double yy, double tt)
        {
            x_des = xx;
            y_des = yy;
            t_des = tt;
            resetFlyToSetPoint();
        }

        public void resetFlyToSetPoint()
        {
            initFlyToSetPoint = true;
        }

        public Boolean hasStartedTrackingTrajectory;
        public double trajThresh = 0.5;
        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectoryManual()
        {
            if (map.trajectory.empty())
            {
                if (map.trajectory.isWithinTheshhold(trajThresh, new JagPoint(x, y)))
                {
                    jaguarControl.controlMode = jaguarControl.MANUAL;
                    return;
                }
            }

            if (!hasStartedTrackingTrajectory)
            {
                JagPoint target = map.trajectory.getTargetPoint();
                setSetPoint(target.x, target.y, map.trajectory.tangent());
                hasStartedTrackingTrajectory = true;
            }
            if (!map.trajectory.isEnd() && map.trajectory.isWithinTheshhold(trajThresh, new JagPoint(x, y)))
            {
                map.trajectory.nextPoint();

                JagPoint target = map.trajectory.getTargetPoint();
                desiredX = target.x;
                desiredY = target.y;
                desiredT = target.hasTheta() ? target.theta : 0;

                resetFlyToSetPoint();
            }

        }


        // THis function is called to follow a trajectory constructed by PRMMotionPlanner()
        private void TrackTrajectory()
        {
            double distToCurrentNode = Math.Sqrt(Math.Pow(x - trajList[trajCurrentNode].x, 2) + Math.Pow(y - trajList[trajCurrentNode].y, 2));
            if (distToCurrentNode < 0.1 && trajCurrentNode + 1 < trajSize)
            {
                trajCurrentNode++;
                x_des = trajList[trajCurrentNode].x;
                y_des = trajList[trajCurrentNode].y;

                if (trajCurrentNode != trajSize)
                {
                    double slope = Math.Atan2(  trajList[trajCurrentNode + 1].y - trajList[trajCurrentNode].y ,
                                                trajList[trajCurrentNode + 1].x - trajList[trajCurrentNode].x );
                    t_des = slope;
                }
                else{
                    t_des = desiredT;
                }

                t_des = 0;
            }

            FlyToSetPoint();
        }

        // This function houses the core motion planner. This function
        // will generate a new trajectory when called. Students must 
        // add their code here.

        private void PRMMotionPlanner()
        {
            // Initialize sampling grid cell variables for weighted
            // random selection of nodes to expand.
            samplingCellSizeX = (maxWorkspaceX - minWorkspaceX) / numXCells;
            samplingCellSizeY = (maxWorkspaceY - minWorkspaceY) / numYCells;
            numOccupiedCells = 0;
            for (int i = 0; i < numXCells * numYCells; i++)
                numNodesInCell[i] = 0;
            numNodes = 0;


            // ****************** Additional Student Code: Start ************

            // Put code here to expand the PRM until the goal node is reached,
            // or until a max number of iterations is reached.


            // Create and add the start Node
            Node startNode = new Node(x_est, y_est, 0, 0);
            AddNode(startNode);


            // Create the goal node
            Node goalNode = new Node(desiredX, desiredY, 0, 0);


            // Loop until path created
            bool pathFound = false;
            int maxIterations = maxNumNodes;
            int iterations = 0;
            Random randGenerator = new Random();

            const double distance = 1;

            int randCellNumber = 0;
            int randNodeNumber = 0;

            Node randExpansionNode;

            //check for direct line from start to goal
            if (!map.CollisionFound(startNode, goalNode, 0.05))
            {
                goalNode.nodeIndex = numNodes;
                goalNode.lastNode = startNode.nodeIndex;
                pathFound = true;
            }



            while (iterations < maxIterations && !pathFound)
            {
                randCellNumber = (int)(randGenerator.NextDouble() * numOccupiedCells);
                randNodeNumber = (int)(randGenerator.NextDouble() * numNodesInCell[occupiedCellsList[randCellNumber]]);

                randExpansionNode = NodesInCells[occupiedCellsList[randCellNumber], randNodeNumber];

                double theta = randGenerator.NextDouble()*2*3.1415;
                double dx = Math.Cos(theta) * distance;
                double dy = Math.Sin(theta) * distance;

                double newX = randExpansionNode.x + dx;
                double newY = randExpansionNode.y + dy;

                Node newNode = new Node(newX, newY, numNodes, randExpansionNode.nodeIndex);

                if (!map.CollisionFound(randExpansionNode, newNode, 0.05))
                {
                    AddNode(newNode);
                }

                if (!map.CollisionFound(newNode, goalNode, 0.05))
                {
                    goalNode.nodeIndex = numNodes;
                    goalNode.lastNode = newNode.nodeIndex;
                    pathFound = true;
                }


                // Increment number of iterations
                iterations++;
            }


            // Create the trajectory to follow
            BuildTraj(goalNode);
                

            
            // ****************** Additional Student Code: End   ************




        }




        // This function is used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // The work environment is divided into a grid of cells.
        // This function returns the cell number.
        int GetCellNumber(double x, double y)
        {
            int cell = (int)Math.Floor((x - minWorkspaceX) / samplingCellSizeX) + (int)(Math.Floor((y - minWorkspaceY) / samplingCellSizeY) * numXCells);
            return cell;
        }

        // This function is also used to implement weighted sampling in 
        // when randomly selecting nodes to expand from in the PRM.
        // When new nodes for the PRM are generated, they must be added
        // to a variety of memory locations.
        // First, the node is stored in a list of nodes specific to a grid
        // cell. If this is the first node in that grid cell, the list of 
        // occupied cells is updated. Then, the node is stored in a general
        // list that keeps track of all nodes for building the final
        // trajectory.

        void AddNode(Node n)
        {
            int cellNumber = GetCellNumber(n.x, n.y);
            if (numNodesInCell[cellNumber] < 1)
            {
                occupiedCellsList[numOccupiedCells] = cellNumber;
                numOccupiedCells++;
            }

            if (numNodesInCell[cellNumber] < 400)
            {
                NodesInCells[cellNumber, numNodesInCell[cellNumber]] = n;
                numNodesInCell[cellNumber]++;

                // Add to nodelist
                nodeList[numNodes] = n;
                numNodes++;
            }
            return;
        }


        // Given the goal node, this function will recursively add the
        // parent node to a trajectory until the start node is reached.
        // The result is a list of nodes that connect the start node to
        // the goal node with collision free edges.

        void BuildTraj(Node goalNode)
        {
            Node[] tempList = new Node[maxNumNodes];
            for (int j = 0; j < maxNumNodes; j++)
                trajList[j] = new Node(0, 0, 0, 0);

            tempList[0] = goalNode;
            int i = 1;

            // Make backwards traj by looking at parent of every child node
            while (tempList[i - 1].nodeIndex != 0)
            {
                tempList[i] = nodeList[tempList[i - 1].lastNode];
                i++;
            }

            // Reverse trajectory order
            for (int j = 0; j < i; j++)
            {
                trajList[j] = tempList[i - j - 1];
            }

            // Set size of trajectory and initialize node counter
            trajSize = i;
            trajCurrentNode = 0;

            return;
        }


 


        #endregion


        #region Localization Functions
        /************************ LOCALIZATION ***********************/

        // This function will grab the most recent encoder measurements
        // from either the simulator or the robot (whichever is activated)
        // and use those measurements to predict the RELATIVE forward 
        // motion and rotation of the robot. These are referred to as
        // distanceTravelled and angleTravelled respectively.
        public Boolean MotionPrediction()
        {

            // ****************** Additional Student Code: Start ************

            // Put code here to calculated distanceTravelled and angleTravelled.
            // You can set and use variables like diffEncoder1, currentEncoderPulse1,
            // wheelDistanceL, wheelRadius, encoderResolution etc. These are defined
            // in the Robot.h file.


            diffEncoderPulseR = -(currentEncoderPulseR - lastEncoderPulseR);
            diffEncoderPulseL = currentEncoderPulseL - lastEncoderPulseL;

            // If rollover occurs, this will measure the correct encoder difference
            if (Math.Abs(diffEncoderPulseR) > 0.5 * encoderMax)
            {
                if (diffEncoderPulseR < 0)
                {
                    diffEncoderPulseR = -encoderMax - 1 - diffEncoderPulseR;
                }
                if (diffEncoderPulseR > 0)
                {
                    diffEncoderPulseR = encoderMax + 1 - diffEncoderPulseR;
                }
            }
            if (Math.Abs(diffEncoderPulseL) > 0.5 * encoderMax)
            {
                if (diffEncoderPulseL < 0)
                {
                    diffEncoderPulseL = -encoderMax - 1 - diffEncoderPulseL;
                }
                if (diffEncoderPulseL > 0)
                {
                    diffEncoderPulseL = encoderMax + 1 - diffEncoderPulseL;
                }
            }


            // distance = r*theta where r=wheelRadius and theta=2*pi*encoderMeasurement/pulsesPerRevolution
            wheelDistanceR = wheelRadius * 2 * Math.PI * diffEncoderPulseR / pulsesPerRotation;
            wheelDistanceL = wheelRadius * 2 * Math.PI * diffEncoderPulseL / pulsesPerRotation;

            // Distance travelled is the average of the left and right wheel distances
            distanceTravelled = (wheelDistanceL + wheelDistanceR) / 2;
            angleTravelled = (wheelDistanceR - wheelDistanceL) / (2 * robotRadius);

            //Save encoder value for next loop iteration
            lastEncoderPulseL = currentEncoderPulseL;
            lastEncoderPulseR = currentEncoderPulseR;

            if (diffEncoderPulseL == 0 && diffEncoderPulseR == 0)
                return false;



            return true;



            // ****************** Additional Student Code: End   ************
        }

        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithOdometry()
        {
            // ****************** Additional Student Code: Start ************

            x = x + distanceTravelled * Math.Cos(t + angleTravelled / 2);
            y = y + distanceTravelled * Math.Sin(t + angleTravelled / 2);
            t = t + angleTravelled;
            if (iPhoneSensor.newCompass)
            {
                //t = iPhoneSensor.Theta();
            }
            //Console.WriteLine(t);

            if (t > Math.PI)
            {
                t = t - 2 * Math.PI;
            }
            if (t < -1 * Math.PI)
            {
                t = t + 2 * Math.PI;
            }







                
            // ****************** Additional Student Code: End   ************
        }
        // This function will Localize the robot, i.e. set the robot position
        // defined by x,y,t using the last position with angleTravelled and
        // distance travelled.
        public void LocalizeRealWithIMU()
        {
            // ****************** Additional Student Code: Start ************

            // Put code here to calculate x,y,t based on odemetry 
            // (i.e. using last x, y, t as well as angleTravelled and distanceTravelled).
            // Make sure t stays between pi and -pi


            // ****************** Additional Student Code: End   ************
        }

        double avgWeight;

        public void LocalizeEstWithParticleFilter()
        {
            // To start, just set the estimated to be the actual for simulations
            // This will not be necessary when running the PF lab
            

            // ****************** Additional Student Code: Start ************
            double p_wheelDistanceR = 0;
            double p_wheelDistanceL = 0;
            double p_distanceTravelled = 0;
            double p_angleTravelled = 0;

            //Propagate particles with randomness
            Console.WriteLine("prediction " + time);
            ttime("start");
            for (int i = 0; i < numParticles; i++)
            {
                // distance = r*theta where r=wheelRadius and theta=2*pi*encoderMeasurement/pulsesPerRevolution
                p_wheelDistanceL = wheelDistanceL + wheelDistanceL * RandomGaussian() * K_wheelRandomness;
                p_wheelDistanceR = wheelDistanceR + wheelDistanceR * RandomGaussian() * K_wheelRandomness;

                // Distance travelled is the average of the left and right wheel distances
                p_distanceTravelled = (p_wheelDistanceL + p_wheelDistanceR) / 2;
                p_angleTravelled = (p_wheelDistanceR - p_wheelDistanceL) / (2 * robotRadius);

                particles[i].x = particles[i].x + p_distanceTravelled * Math.Cos(particles[i].t + p_angleTravelled / 2);
                particles[i].y = particles[i].y + p_distanceTravelled * Math.Sin(particles[i].t + p_angleTravelled / 2);
                particles[i].t = particles[i].t + p_angleTravelled;

                particles[i].t = boundAngle(particles[i].t, 1);

            }
            ttime("prediction");

            Console.WriteLine(newLaserData + "<><>");
            if (newLaserData)
            {
                newLaserCounter++;
                //newLaserData = false;
                Console.WriteLine(newLaserCounter + "<<<<<");
            }

            if (newLaserCounter > 2 && ((!correctionOverrideEnabled) || (correctionOverrideEnabled && correctionOverride)))
            {
                newLaserCounter = 0;
                Console.WriteLine("Correction Step");
                correctionOverride = false;
                //calculate unnormalized weight
                for (int i = 0; i < numParticles; i++)
                {
                    CalculateWeight(i);
                }
                ttime("calculate weight");
                //calculate normalized weight
                double maxWeight = 0.01;

                avgWeight = 0;
                for (int i = 0; i < numParticles; i++)
                {
                    maxWeight = Math.Max(particles[i].w, maxWeight);
                    avgWeight += particles[i].w;
                }
                avgWeight = avgWeight / numParticles;
                Console.WriteLine(avgWeight);
                for (int i = 0; i < numParticles; i++)
                {
                    particles[i].w = particles[i].w / maxWeight;
                    //Console.Write(propagatedParticles[i].w+" ");
                }
                //LogData();
                Console.WriteLine();

                ttime("normalize");

                //correction step
                //udacityResample();
                approximateRedistribute();
                //exactRedistribute();

                ttime("resample");

            }
            else { Console.WriteLine("skipped correction"); }


            //average to find the current robot state
            for (int i = 0; i < numParticles; i++)
            {
                x_est += particles[i].x;
                y_est += particles[i].y;
                t_est += particles[i].t;
            }
            x_est = x_est / numParticles;
            y_est = y_est / numParticles;
            t_est = t_est / numParticles;




            // ****************** Additional Student Code: End   ************

        }

        private void approximateRedistribute()
        {
            tempParticlesCount = 0;
            double[] c = { 0.000001, 0.2, 0.4, 0.7, 1 };
            for (int i = 0; i < numParticles; i++)
            {
                /*if (particles[i].w < c[0])
                {
                    particles[i].x = map.minX + myRandom.NextDouble() * (map.maxX - map.minX);
                    particles[i].y = map.minY + myRandom.NextDouble() * (map.maxY - map.minY);
                    particles[i].t = myRandom.NextDouble() * Math.PI * 2;

                    copyParticleIntoTemp(i, 1);
                    continue;
                }*/
                if (particles[i].w < c[1])
                {
                    copyParticleIntoTemp(i, 1);
                    continue;
                }
                if (particles[i].w < c[2])
                {
                    copyParticleIntoTemp(i, 2);
                    continue;
                }
                if (particles[i].w < c[3])
                {
                    copyParticleIntoTemp(i, 3);
                    continue;
                }
                if (particles[i].w <= c[4])
                {
                    copyParticleIntoTemp(i, 4);
                }
            }
            //fillTempWithRandomness();

            for (int i = 0; i < numParticles; i++)
            {
                int r = (int)(tempParticlesCount * myRandom.NextDouble());
                particles[i].x = tempParticles[r].x;
                particles[i].y = tempParticles[r].y;
                particles[i].t = tempParticles[r].t;
            }
            tempParticlesCount = 0;
        }

        private void copyParticleIntoTemp(int p, int n)
        {
            for (int i = 0; i < n; i++)
            {
                tempParticles[tempParticlesCount].x = particles[p].x;
                tempParticles[tempParticlesCount].y = particles[p].y;
                tempParticles[tempParticlesCount].t = particles[p].t;
                tempParticlesCount += 1;
            }

        }

        void CalculateWeight_improved(int p)
        {
            long expectedMeasurement = 0;
            long weight = 0;
            int n = 0;
            int sigma = 10000000;

            // ****************** Additional Student Code: Start ************
            for (int i = 0; i < LaserData.Length; i += 18)
            {
                expectedMeasurement = (long)(1000 * map.GetClosestWallDistance(particles[p].x, particles[p].y, particles[p].t - 1.570796327
 + laserAngles[i]));

                //n++;
                //weight *= (1.0 / Math.Sqrt(2 * Math.PI * sigma)) * Math.Exp(-Math.Pow(expectedMeasurement - LaserData[i], 2) / (Math.Pow(sigma, 2)*2));//Math.Pow(expectedMeasurement - LaserData[i],2)/1000;


                if (expectedMeasurement > 0 && LaserData[i] > 0)
                {
                    n++;
                    weight += (expectedMeasurement - LaserData[i]) * (expectedMeasurement - LaserData[i]);
                }
                else
                {
                    weight += 1000 * 1000;
                }

                if (p == 0)
                {
                    //Console.WriteLine(p + "," + expectedMeasurement + "," + LaserData[i] + "," + weight);
                }

            }
            // if (n > 0)
            particles[p].w = Math.Exp(-(weight) / sigma);
            //else
            //  particles[p].w = 0.01;

        }

        // Particle filters work by setting the weight associated with each
        // particle, according to the difference between the real robot 
        // range measurements and the predicted measurements associated 
        // with the particle.
        // This function should calculate the weight associated with particle p.

        void CalculateWeight(int p)
        {
            long expectedMeasurement = 0;
            long weight = 0;
            int n = 0;
            int sigma = 100000;

            // ****************** Additional Student Code: Start ************
            for (int i = 0; i < LaserData.Length; i += 18)
            {
                expectedMeasurement = (long)(1000 * map.GetClosestWallDistance(particles[p].x, particles[p].y, particles[p].t - 1.570796327
 + laserAngles[i]));

                if (expectedMeasurement > 0 && LaserData[i] > 0)
                {
                    n++;
                    weight += (expectedMeasurement - LaserData[i]) * (expectedMeasurement - LaserData[i]);
                }

                if (p == 0)
                {
                    Console.WriteLine(p + "," + expectedMeasurement + "," + LaserData[i] + "," + weight);
                }


            }
            if (n > 0)
            {
                particles[p].w = Math.Exp(-(weight / n) / sigma);
            }


        }



        // This function is used to initialize the particle states 
        // for particle filtering. It should pick a random location in the 
        // environment for each particle by calling SetRandomPos

        void InitializeParticles()
        {


            // Set particles in random locations and orientations within environment
            for (int i = 0; i < numParticles; i++)
            {

                // Either set the particles at known start position [0 0 0],  
                // or set particles at random locations.

                if (jaguarControl.startMode == jaguarControl.UNKNOWN)
                    SetRandomPos(i);
                else if (jaguarControl.startMode == jaguarControl.KNOWN)
                    SetStartPos(i);
            }



        }


        // For particle p, this function will select a valid position. It should
        // select the position randomly, with equal likelihood of being anywhere 
        // in the environement. Should work for rectangular environments to make 
        // things easier.

        Random myRandom = new Random();
        public int randomParticleMode = PART_AROUND_XYT;
        public const int PART_EVERYWHERE = 0;
        public const int PART_AROUND_XY = 1;
        public const int PART_AROUND_XYT = 2;
        public int PART_xyrandomness = 2;
        public double PART_trandomness = 0.02;


        void SetRandomPos(int p)
        {
            switch (randomParticleMode)
            {
                case PART_EVERYWHERE:
                    particles[p].x = map.minX + myRandom.NextDouble() * (map.maxX - map.minX);
                    particles[p].y = map.minY + myRandom.NextDouble() * (map.maxY - map.minY);
                    particles[p].t = myRandom.NextDouble() * Math.PI * 2;
                    break;
                case PART_AROUND_XYT:
                    particles[p].x = x + RandomGaussian() * PART_xyrandomness;
                    particles[p].y = y + RandomGaussian() * PART_xyrandomness;
                    particles[p].t = iPhoneSensor.Theta() + RandomGaussian() * 3.1415 *PART_trandomness;
                    break;
                case PART_AROUND_XY:
                    particles[p].x = x + RandomGaussian() * PART_xyrandomness;
                    particles[p].y = y + RandomGaussian() * PART_xyrandomness;
                    particles[p].t = myRandom.NextDouble() * Math.PI * 2;
                    break;
            }

            particles[p].w = 1 / numParticles;
        }

        // For particle p, this function will select a start predefined position. 
        void SetStartPos(int p)
        {
            particles[p].x = initialX;
            particles[p].y = initialY;
            particles[p].t = initialT;
        }



        // Random number generator with gaussian distribution
        // Often random guassian numbers are used in particle filters. This
        // function might help.

        double RandomGaussian()
        {
            double U1, U2, V1 = 0, V2;
            double S = 2.0;
            while (S >= 1.0)
            {

                U1 = random.NextDouble();
                U2 = random.NextDouble();
                V1 = 2.0 * U1 - 1.0;
                V2 = 2.0 * U2 - 1.0;
                S = Math.Pow(V1, 2) + Math.Pow(V2, 2);
            }

            double gauss = V1 * Math.Sqrt((-2.0 * Math.Log(S)) / S);
            return gauss;
        }



        // Get the sign of a number
        double Sgn(double a)
        {
            if (a > 0)

                return 1.0;
            else if (a < 0)
                return -1.0;
            else

                return 0.0;
        }

        #endregion

        #region Misc. functions
        static public double boundAngle(double theta, double sthPi)
        {
            //if sthpi is 2, then it bounds between 0 and 2pi
            if (theta <= sthPi * Math.PI && theta >= sthPi * Math.PI - 2 * Math.PI)
            {
                return theta;
            }
            if (theta > sthPi * Math.PI)
            {
                theta -= 2 * Math.PI;
            }
            else if (theta < sthPi * Math.PI - 2 * Math.PI)
            {
                theta += 2 * Math.PI;
            }
            return boundAngle(theta, sthPi);
        }

        #endregion

    }
}
