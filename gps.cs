using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace DrRobot.JaguarControl
{
    class Gps
    {
        
        public double latitude;
        public double longitude;

        double xZero;
        double yZero;

        double x, y;

        public JaguarCtrl jc;
        public Gps(JaguarCtrl _jc)
        {
            jc = _jc;
            calibrate(34.063849, -117.4271949);
            //calibrate(34.106315, -117.711974);
        }

        public void updateGps()
        {
            latitude = jc.gpsRecord.latitude/100;
            longitude = jc.gpsRecord.longitude/100 * -1;

            //Console.WriteLine(latitude + " " + longitude + " " + getX() + " " + getY());

            updateGpsToX(latitude, longitude);
        }

        public double getX()
        {

            return x - xZero;

        }
        public double getY()
        {
            return y - yZero;
        }

        public void calibrate(double lat, double lon)
        {
            updateGpsToX(lat, lon);
            xZero = x;
            yZero = y;
        }

        public void updateGpsToX(double lat, double lon)
        {
            double degToRad = Math.PI / 180.0 / 100;
            x = 6367.5 * 1000.0 * Math.Cos(lat * degToRad) * Math.Cos(lon * degToRad);
            y = 6367.5 * 1000.0 * Math.Cos(lat * degToRad) * Math.Sin(lon * degToRad);

            //Console.WriteLine((x-xZero) + " " + (y-yZero));
        }
    }
}
