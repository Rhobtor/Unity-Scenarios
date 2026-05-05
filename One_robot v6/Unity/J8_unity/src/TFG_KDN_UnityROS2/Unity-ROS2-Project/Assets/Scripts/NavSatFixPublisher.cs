using UnityEngine;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;

public class NavSatFixPublisher : MonoBehaviour
{
    ROSConnection ros;
    public GameObject fixposition_GPS;
    // Coordenadas de referencia (latitud, longitud, altitud)
    public double referenceLatitude = 36.717083;
    public double referenceLongitude = -4.489455;
    public double referenceAltitude = 50.0;

    // Radio de la Tierra en metros
    private const double EarthRadius = 6378137.0;

    // Frecuencia de publicación
    public float publishFrequency = 1.0f;
    private float timeElapsed;

    void Start()
    {
        // Inicializar ROSConnection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<NavSatFixMsg>("/fixposition/navsatfix");
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > 1.0f / publishFrequency)
        {
            PublishGPSData();
            timeElapsed = 0;
        }
    }

    void PublishGPSData()
    {
        // Obtener la posición relativa del GameObject en Unity
        Vector3 relativePosition = fixposition_GPS.transform.position;

        // Calcular las coordenadas GPS
        double deltaLatitude = (relativePosition.x / (EarthRadius * Math.Cos(referenceLatitude * Math.PI / 180.0))) * (180.0 / Math.PI);
        double deltaLongitude = (relativePosition.z / EarthRadius) * (180.0 / Math.PI); 
        double deltaAltitude = relativePosition.y;

        double latitude = referenceLatitude + deltaLatitude;
        double longitude = referenceLongitude - deltaLongitude;
        double altitude = referenceAltitude + deltaAltitude;

        // Crear mensaje NavSatFix
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

        // Publicar el mensaje
        ros.Publish("/fixposition/navsatfix", gpsMessage);
        Debug.Log($"Publicado GPS: lat={latitude}, lon={longitude}, alt={altitude}");
    }
}
