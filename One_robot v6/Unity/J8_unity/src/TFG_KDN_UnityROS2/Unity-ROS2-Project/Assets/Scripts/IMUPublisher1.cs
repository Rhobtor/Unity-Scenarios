using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using System;

public class GPSIMUPublisher : MonoBehaviour
{
    ROSConnection ros;
    public GameObject fixpositionObject;

    // Referencias iniciales (coordenadas GPS)
    public double referenceLatitude = 36.717083;
    public double referenceLongitude = -4.489455;
    public double referenceAltitude = 50.0;
    private const double EarthRadius = 6378137.0;

    // Frecuencia de publicación
    public float gpsPublishFrequency = 1.0f;
    public float imuPublishFrequency = 200.0f;

    // Tiempos acumulados
    private float gpsTimeElapsed;
    private float imuTimeElapsed;

    // Estados previos
    private Vector3 lastPosition;
    private Vector3 lastVelocity;
    private Quaternion lastRotation;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>("/fixposition/navsatfix");
        ros.RegisterPublisher<ImuMsg>("/fixposition/imu");

        lastPosition = fixpositionObject.transform.position;
        lastVelocity = Vector3.zero;
    }

    void Update()
    {
        float dt = Time.deltaTime;

        gpsTimeElapsed += dt;
        imuTimeElapsed += dt;

        if (gpsTimeElapsed >= 1.0f / gpsPublishFrequency)
        {
            PublishGPSData();
            gpsTimeElapsed = 0;
        }

        if (imuTimeElapsed >= 1.0f / imuPublishFrequency)
        {
            PublishIMUData(imuTimeElapsed);
            imuTimeElapsed = 0;
        }
    }

    void PublishGPSData()
    {
        Vector3 relativePosition = fixpositionObject.transform.position;

        double deltaLatitude = (relativePosition.x / (EarthRadius * Math.Cos(referenceLatitude * Math.PI / 180.0))) * (180.0 / Math.PI);
        double deltaLongitude = (relativePosition.z / EarthRadius) * (180.0 / Math.PI);
        double deltaAltitude = relativePosition.y;

        double latitude = referenceLatitude + deltaLatitude;
        double longitude = referenceLongitude - deltaLongitude;
        double altitude = referenceAltitude + deltaAltitude;

        NavSatFixMsg gpsMessage = new NavSatFixMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
                    nanosec = (uint)(DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000 * 1000000)
                },
                frame_id = "GPS"
            },
            latitude = latitude,
            longitude = longitude,
            altitude = altitude,
            status = new NavSatStatusMsg
            {
                status = NavSatStatusMsg.STATUS_FIX,
                service = NavSatStatusMsg.SERVICE_GPS
            },
            position_covariance = new double[9],
            position_covariance_type = NavSatFixMsg.COVARIANCE_TYPE_UNKNOWN
        };

        ros.Publish("/fixposition/navsatfix", gpsMessage);
    }

    void PublishIMUData(float dt)
    {
        Vector3 currentPosition = fixpositionObject.transform.position;
        Vector3 currentVelocity = (currentPosition - lastPosition) / dt;
        Vector3 acceleration = (currentVelocity - lastVelocity) / dt;
        Vector3 angularVelocity;

        // Calcular velocidad angular usando quaternions

        Quaternion currentRotation = fixpositionObject.transform.rotation;
        Quaternion deltaRotation = currentRotation * Quaternion.Inverse(lastRotation);
        deltaRotation.ToAngleAxis(out float angle, out Vector3 axis);
        angularVelocity = axis * Mathf.Deg2Rad * angle / imuTimeElapsed;


        ImuMsg imuMessage = new ImuMsg
        {
            header = new RosMessageTypes.Std.HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
                    nanosec = (uint)(DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000 * 1000000)
                },
                frame_id = "IMU"
            },
            orientation = new RosMessageTypes.Geometry.QuaternionMsg
            {
                x = (float)currentRotation.x,
                y = (float)currentRotation.y,
                z = (float)currentRotation.z,
                w = (float)currentRotation.w
            },
            angular_velocity = new RosMessageTypes.Geometry.Vector3Msg()
            {
                x = angularVelocity.x,
                y = angularVelocity.y,
                z = angularVelocity.z
            },
            linear_acceleration = new RosMessageTypes.Geometry.Vector3Msg
            {
                x = acceleration.x,
                y = acceleration.y,
                z = acceleration.z
            },
            orientation_covariance = new double[9],
            angular_velocity_covariance = new double[9],
            linear_acceleration_covariance = new double[9]
        };

        ros.Publish("/fixposition/imu", imuMessage);

        lastPosition = currentPosition;
        lastVelocity = currentVelocity;
        lastRotation = currentRotation;
    }
}

