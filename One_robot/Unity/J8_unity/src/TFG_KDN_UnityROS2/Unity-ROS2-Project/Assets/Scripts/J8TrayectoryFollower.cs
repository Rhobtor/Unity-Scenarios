/******************************************************
 * Author: Óscar Almenara Reyes
 * Bachelor's Degree in Industrial Electronics Engineering
 * University of Málaga
 * Final Degree Project: "Towards digital twins in emergency robotics: 
    representation of real-world data in a virtual environment using Unity and ROS 2."
 * Year: 2025
 ******************************************************/

using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Nav;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

public class J8TrayectoryFollower : MonoBehaviour
{
    [Header("ROS Topics")]
    public string gpsTopic = "/fixposition/navsatfix";
    public string odomTopic = "/fixposition/odometry_enu";
    public bool drawPath = false;

    [Header("Ruedas")]
    public Transform[] leftWheels;
    public Transform[] rightWheels;
    public float wheelRadius = 0.3175f;
    public float wheelBase = 0.6f;

    [Header("Terreno y Altura")]
    public LayerMask terrainLayerMask;
    public float raycastHeight = 3f;
    public float heightOffset = 0.5f;

    [Header("Visualización")]
    public bool showLine = false;

    private ROSConnection ros;
    private LineRenderer lineRenderer;
    private Vector3 latestPosition;
    private Quaternion latestRotation;
    private float lastValidY;

    private bool hasReceivedGPS = false;
    private bool previousShowLine = false;

    private float linearVelocity = 0f;
    private float angularVelocity = 0f;

    private readonly double refLat = 36.717083;
    private readonly double refLon = -4.489410;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<NavSatFixMsg>(gpsTopic, OnGPSReceived);
        ros.Subscribe<OdometryMsg>(odomTopic, OnOdomReceived);

        lineRenderer = GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = gameObject.AddComponent<LineRenderer>();
            lineRenderer.startWidth = 0.05f;
            lineRenderer.endWidth = 0.05f;
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.startColor = Color.red;
            lineRenderer.endColor = Color.red;
        }

        lineRenderer.enabled = showLine;
        previousShowLine = showLine;

        latestRotation = transform.rotation;
        lastValidY = transform.position.y;
    }

    void Update()
    {
        if (!hasReceivedGPS) return;

        float dt = Time.deltaTime;

        float vLeft = linearVelocity - (angularVelocity * wheelBase / 2f);
        float vRight = linearVelocity + (angularVelocity * wheelBase / 2f);
        float rotationLeft = (vLeft * dt / wheelRadius) * Mathf.Rad2Deg;
        float rotationRight = (vRight * dt / wheelRadius) * Mathf.Rad2Deg;

        foreach (Transform wheel in leftWheels)
            wheel.Rotate(Vector3.right, rotationLeft);
        foreach (Transform wheel in rightWheels)
            wheel.Rotate(Vector3.right, rotationRight);

        UpdateRobotPose();

        // Cambiar visibilidad del LineRenderer en tiempo real
        if (showLine != previousShowLine)
        {
            lineRenderer.enabled = showLine;
            previousShowLine = showLine;

            if (showLine)
            {
                lineRenderer.positionCount = 1;
                lineRenderer.SetPosition(0, transform.position);
            }
        }

        if (showLine && lineRenderer != null && lineRenderer.enabled)
        {
            lineRenderer.positionCount++;
            lineRenderer.SetPosition(lineRenderer.positionCount - 1, transform.position);
        }
    }

    void OnGPSReceived(NavSatFixMsg msg)
    {
        Debug.Log($"GPS Received: Lat {msg.latitude}, Lon {msg.longitude}");
        latestPosition = ConvertGPSToUnity(msg.latitude, msg.longitude, msg.altitude);
        hasReceivedGPS = true;
    }

    void OnOdomReceived(OdometryMsg msg)
    {
        Debug.Log("Odom Received");
        latestRotation = msg.pose.pose.orientation.From<FLU>() * Quaternion.Euler(0, 180f, 0);
        linearVelocity = (float)msg.twist.twist.linear.x;
        angularVelocity = (float)msg.twist.twist.angular.z;
    }

    void UpdateRobotPose()
    {
        Vector3[] offsets = new Vector3[]
        {
            new Vector3(0.3f, 0, 0.3f),
            new Vector3(-0.3f, 0, 0.3f),
            new Vector3(0.3f, 0, -0.3f),
            new Vector3(-0.3f, 0, -0.3f)
        };

        List<Vector3> hitPoints = new List<Vector3>();
        List<Vector3> hitNormals = new List<Vector3>();

        foreach (var offset in offsets)
        {
            Vector3 origin = latestPosition + offset + Vector3.up * raycastHeight;
            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastHeight * 2f, terrainLayerMask))
            {
                hitPoints.Add(hit.point);
                hitNormals.Add(hit.normal);
            }

            Debug.DrawRay(origin, Vector3.down * raycastHeight * 2f, Color.green);
        }

        Vector3 targetPosition = transform.position;
        Quaternion targetRotation = transform.rotation;

        if (hitPoints.Count >= 3)
        {
            Vector3 avgPoint = Vector3.zero;
            Vector3 avgNormal = Vector3.zero;

            foreach (var p in hitPoints) avgPoint += p;
            foreach (var n in hitNormals) avgNormal += n;

            avgPoint /= hitPoints.Count;
            avgNormal.Normalize();

            targetPosition = new Vector3(latestPosition.x, avgPoint.y + heightOffset, latestPosition.z);
            lastValidY = targetPosition.y;

            Vector3 forwardProjected = Vector3.ProjectOnPlane(latestRotation * Vector3.forward, avgNormal).normalized;
            targetRotation = Quaternion.LookRotation(forwardProjected, avgNormal);
        }
        else
        {
            targetPosition = new Vector3(latestPosition.x, lastValidY, latestPosition.z);
            targetRotation = latestRotation;
        }

        Vector3 checkOrigin = latestPosition + Vector3.up * raycastHeight;
        if (Physics.Raycast(checkOrigin, Vector3.down, out RaycastHit groundHit, raycastHeight * 2f, terrainLayerMask))
        {
            float minY = groundHit.point.y + heightOffset;
            if (targetPosition.y < minY)
                targetPosition.y = minY;
        }

        float smoothSpeed = 5f;
        transform.position = Vector3.Lerp(transform.position, targetPosition, Time.deltaTime * smoothSpeed);
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, Time.deltaTime * smoothSpeed);
    }

    Vector3 ConvertGPSToUnity(double lat, double lon, double alt)
    {
        double R = 6378137.0;
        double dLat = Mathf.Deg2Rad * (lat - refLat);
        double dLon = Mathf.Deg2Rad * (lon - refLon);
        double meanLat = Mathf.Deg2Rad * ((lat + refLat) / 2.0);

        float x = (float)(dLat * R);
        float z = (float)(-dLon * R * Math.Cos(meanLat));
        float y = 0f;

        return new Vector3(x, y, z);
    }
}
