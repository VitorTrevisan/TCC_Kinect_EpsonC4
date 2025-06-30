using Microsoft.Kinect;
using System;
using System.Linq;

public class KinectManager
{
    private static KinectManager _instance;
    public KinectSensor Sensor { get; private set; }

    private KinectManager()
    {
        Sensor = KinectSensor.KinectSensors.FirstOrDefault(s => s.Status == KinectStatus.Connected);
        if (Sensor != null)
        {
            if (!Sensor.ColorStream.IsEnabled)
                //Sensor.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
                Sensor.ColorStream.Enable(ColorImageFormat.RgbResolution1280x960Fps12);

            if (!Sensor.SkeletonStream.IsEnabled)
                Sensor.SkeletonStream.Enable();

            if (!Sensor.IsRunning)
                Sensor.Start();
        }
        else
        {
            throw new Exception("Kinect não encontrado.");
        }
    }

    public static KinectManager Instance
    {
        get
        {
            if (_instance == null)
                _instance = new KinectManager();

            return _instance;
        }
    }

    public void Stop()
    {
        if (Sensor != null && Sensor.IsRunning)
        {
            Sensor.Stop();
        }
    }
}
