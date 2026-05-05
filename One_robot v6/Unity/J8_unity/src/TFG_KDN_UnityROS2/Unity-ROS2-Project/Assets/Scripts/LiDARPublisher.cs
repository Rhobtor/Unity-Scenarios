using System;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Sensor;

public class LiDARPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/lidar_scan";
    public int publishRate = 10; // Hz
    private float timeElapsed;

    //Parametros del Velodyne
    public int numSamples = 431;
    public float minRange = 0.2f;
    public float maxRange = 35.0f;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<LaserScanMsg>(topicName);
        timeElapsed = 0;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > 1.0f / publishRate)
        {
            PublishLiDARData();
            timeElapsed = 0;
        }
    }

    void PublishLiDARData()
    {
        LaserScanMsg lidarMessage = new LaserScanMsg();

        //lidarMessage.header.stamp = new RosTime();
        lidarMessage.header.frame_id = "Velodyne_link";

        lidarMessage.angle_min = -Mathf.PI / 2; 
        lidarMessage.angle_max = Mathf.PI / 2;
        lidarMessage.angle_increment = (lidarMessage.angle_max - lidarMessage.angle_min) / numSamples;
        lidarMessage.time_increment = 1.0f / publishRate;
        lidarMessage.scan_time = 1.0f / publishRate;
        lidarMessage.range_min = minRange;
        lidarMessage.range_max = maxRange;

        // Simula los datos de distancia
        lidarMessage.ranges = new float[numSamples];
        lidarMessage.intensities = new float[numSamples]; // Añadimos intensidades para simular reflexiones, aunque sea 0

        for (int i = 0; i < numSamples; i++)
        {
            float angle = lidarMessage.angle_min + i * lidarMessage.angle_increment;

            // Calcula la dirección en base al ángulo
            Vector3 rayDirection = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle));
            RaycastHit hit;

            // Dibuja el rayo en la escena de Unity
            //Debug.DrawRay(transform.position, transform.TransformDirection(rayDirection) * maxRange, Color.yellow);

            // Raycast en dirección calculada
            if (Physics.Raycast(transform.position, transform.TransformDirection(rayDirection), out hit, maxRange))
            {
                lidarMessage.ranges[i] = hit.distance;
                lidarMessage.intensities[i] = 1.0f; // Valor de intensidad fijo para simular detección
            }
            else
            {
                lidarMessage.ranges[i] = maxRange;
                lidarMessage.intensities[i] = 0.0f; // Sin detección
            }
        }

        ros.Publish(topicName, lidarMessage);
    }
}
