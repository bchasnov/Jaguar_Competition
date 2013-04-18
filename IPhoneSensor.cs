using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Net;

namespace DrRobot.JaguarControl
{
    class IPhoneSensor
    {
        UdpClient recievingUdpClient;
        IPEndPoint RemoteIpEndPoint;
        public IPhoneSensor(int port)
        {
            recievingUdpClient = new UdpClient(port);
            RemoteIpEndPoint = new IPEndPoint(IPAddress.Any, 0);

            calibrateGPS(34.1064,-117.7119);
        }

        public Boolean newGps;
        public Boolean newCompass;

        private double headingDegrees = 0;
        private double headingTheta = 0;

        public double latitude = 0;
        public double longitude = 0;
        public double horizontalAccuracy = 65;
        public double verticalAccuracy = 10;

        private double xGPSZero = 0;
        private double yGPSZero = 0;

        public double Theta(){
            newCompass = false;
            return headingTheta;
        }

        private double latToY(double latty){ return latty*60*1852; }
        private double lonToX(double longy, double latty){ return longy*60*1852*Math.Cos(latty/180 * Math.PI); }

        public void calibrateGPS(double latty, double longy)
        {
            yGPSZero = latToY(latty);
            xGPSZero = lonToX(longy, latty);
        }
        
        public double GPSx()
        {
            return lonToX(longitude, latitude) - xGPSZero;
        }

        public double GPSy()
        {
            return latToY(latitude) - yGPSZero;
        }

        public void updateReadings()
        {
            string returnData = "";
            while (recievingUdpClient.Available > 0)
            {
                try
                {
                    Byte[] recieveBytes = recievingUdpClient.Receive(ref RemoteIpEndPoint);
                    returnData = Encoding.ASCII.GetString(recieveBytes);
                    Console.WriteLine(returnData);
                }
                catch (Exception e)
                {
                    //Console.WriteLine(e.ToString());
                    return;
                }

                string[] data = returnData.Split(',');

                int id = 0;
                try
                {
                    //int time = data[0];
                    id = Convert.ToInt32(data[1]);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e.ToString());
                }

                switch (id)
                {
                    case 1:
                        //GPS
                        newGps = true;
                        latitude = Convert.ToDouble(data[2]);
                        longitude = Convert.ToDouble(data[3]);
                        horizontalAccuracy = Convert.ToDouble(data[5]);
                        verticalAccuracy = Convert.ToDouble(data[5]);
                        break;
                    case 2:
                        //heading
                        newCompass = true;
                        headingDegrees = Convert.ToDouble(data[2]);
                        headingTheta = Navigation.boundAngle((90 - headingDegrees) * Math.PI / 180.0, 1);
                        Console.WriteLine("Heading: " + headingDegrees + ", " + headingTheta);
                        break;
                }
            }
        }

    }
}
