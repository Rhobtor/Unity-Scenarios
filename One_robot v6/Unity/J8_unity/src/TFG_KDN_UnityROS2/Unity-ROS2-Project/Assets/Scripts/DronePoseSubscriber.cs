/******************************************************
 * Author: Óscar Almenara Reyes
 * Bachelor's Degree in Industrial Electronics Engineering
 * University of Málaga
 * Final Degree Project: "Towards digital twins in emergency robotics: 
    representation of real-world data in a virtual environment using Unity and ROS 2."
 * Year: 2025
 ******************************************************/

using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using System;

public class DronePoseSubscriber : MonoBehaviour
{
    public string gpsTopic = "/FX8/fix";
    private ROSConnection ros;

    private Vector3 currentPosition;

    // Coordenadas GPS de referencia (corresponden a posición (0, 0, 0) en Unity)
    private readonly double refLat = 36.717083;
    private readonly double refLon = -4.489410;
    private readonly float refAlt = 50f; // Altitud real de la posición de referencia

    [Header("Trayectoria")]
    public bool enableLineRenderer = true;
    public float minDistanceToAddPoint = 0.3f;
    private LineRenderer lineRenderer;
    private List<Vector3> pathPoints = new List<Vector3>();

    [Header("Hélices")]
    public Transform[] helices;
    public float helixSpeed = 1000f;

    [Tooltip("Dirección de giro de cada hélice (1 = horario, -1 = antihorario).")]
    public float[] helixDirections = new float[] { 1f, -1f, 1f, -1f, 1f, -1f };

    private bool droneRecibiendoGPS = false;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<NavSatFixMsg>(gpsTopic, OnGPSReceived);

        if (enableLineRenderer)
        {
            lineRenderer = GetComponent<LineRenderer>();
            if (lineRenderer == null)
            {
                lineRenderer = gameObject.AddComponent<LineRenderer>();
            }

            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.widthMultiplier = 0.05f;
            lineRenderer.positionCount = 0;
            lineRenderer.useWorldSpace = true;
            lineRenderer.startColor = Color.green;
            lineRenderer.endColor = Color.green;
        }
    }

    void Update()
    {
        if (droneRecibiendoGPS && helices != null)
        {
            for (int i = 0; i < helices.Length; i++)
            {
                if (helices[i] != null && i < helixDirections.Length)
                {
                    helices[i].Rotate(Vector3.right, helixDirections[i] * helixSpeed * Time.deltaTime, Space.Self);
                }
            }
        }
    }

    void OnGPSReceived(NavSatFixMsg msg)
    {
        Vector3 enu = ConvertGPSToENU(msg.latitude, msg.longitude);
        float altitud = (float)msg.altitude;

        // Corregimos restando la altitud de referencia (50 m) y otros 50 m por usar altura ortométrica
        float y = altitud - refAlt - 52;

        currentPosition = new Vector3(enu.x, y, enu.z);
        transform.position = currentPosition;

        droneRecibiendoGPS = true;

        if (enableLineRenderer)
        {
            if (pathPoints.Count == 0 || Vector3.Distance(currentPosition, pathPoints[^1]) > minDistanceToAddPoint)
            {
                pathPoints.Add(currentPosition);
                lineRenderer.positionCount = pathPoints.Count;
                lineRenderer.SetPositions(pathPoints.ToArray());
            }
        }
    }

    Vector3 ConvertGPSToENU(double lat, double lon)
    {
        double R = 6378137.0;
        double dLat = Mathf.Deg2Rad * (lat - refLat);
        double dLon = Mathf.Deg2Rad * (lon - refLon);
        double meanLat = Mathf.Deg2Rad * ((lat + refLat) / 2.0);

        float x = (float)(dLat * R);                          // Latitud → X
        float z = (float)(-dLon * R * Math.Cos(meanLat));    // Longitud oeste → Z

        return new Vector3(x, 0, z);
    }
}
