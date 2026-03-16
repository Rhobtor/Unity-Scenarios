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
using RosMessageTypes.Std;

public class HumanFollower : MonoBehaviour
{
    [Header("ROS Topic")]
    public string topicName = "/Helmet08/location";

    [Header("Animaciones")]
    public Animator animator;

    [Header("Movimiento")]
    public float moveSpeed = 2f;
    public float rotationSpeed = 5f;

    [Header("Clones radiales")]
    public bool isOriginal = true;
    public GameObject humanPrefab;
    public int cloneCount = 5;
    public float cloneRadius = 3f;

    [Header("Indicador del original")]
    public Transform markerHolder;

    [Header("Terreno")]
    public LayerMask terrainLayerMask;
    public float raycastHeight = 3f;
    public float heightOffset = 0.2f;

    [Header("Batería")]
    public string batteryTopic = "/Helmet08/battery";
    private TextMesh batteryText;
    private GameObject batteryGO;   // Guarda la referencia al GameObject del texto batería
    public bool showBattery = true;
    private bool previousShowBattery = true;

    [Header("Línea de trayectoria")]
    public bool showLine = false;
    private bool previousShowLine = false;
    private LineRenderer lineRenderer;

    private readonly double refLat = 36.717083;
    private readonly double refLon = -4.489415;

    private ROSConnection ros;
    private Vector3 latestReceivedPosition;
    private bool hasTarget = false;
    private Vector3 previousPosition;

    private HumanFollower originalReference;
    private Vector3 radialOffset;

    private static Vector3 sharedOriginalPosition;
    private static bool sharedHasTarget;

    private bool isFirstGPS = true;

    void Start()
    {
        if (isOriginal)
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<Float64MultiArrayMsg>(topicName, OnGPSReceived);
            ros.Subscribe<UInt8Msg>(batteryTopic, OnBatteryReceived);

            if (markerHolder != null)
            {
                CreateRedMarker();
                CreateBatteryText();
            }

            SpawnClones();
            SetupLineRenderer();
        }
        else
        {
            animator = GetComponent<Animator>();
        }

        previousPosition = transform.position;
    }

    void OnGPSReceived(Float64MultiArrayMsg msg)
    {
        if (msg.data.Length < 2) return;

        double lat = msg.data[0];
        double lon = msg.data[1];

        latestReceivedPosition = ConvertGPSToUnity(lat, lon);
        hasTarget = true;
        sharedOriginalPosition = latestReceivedPosition;
        sharedHasTarget = true;

        if (isFirstGPS)
        {
            isFirstGPS = false;

            Vector3 adjustedPosition = AdjustHeightWithRaycast(latestReceivedPosition, out Quaternion terrainRotation);
            transform.position = adjustedPosition;
            transform.rotation = terrainRotation;
            previousPosition = transform.position;

            if (isOriginal)
            {
                foreach (var clone in FindObjectsOfType<HumanFollower>())
                {
                    if (!clone.isOriginal && clone.originalReference == this)
                    {
                        Vector3 clonePosition = latestReceivedPosition + clone.radialOffset;
                        Vector3 adjustedClonePos = clone.AdjustHeightWithRaycast(clonePosition, out Quaternion cloneRotation);
                        clone.transform.position = adjustedClonePos;
                        clone.transform.rotation = cloneRotation;
                        clone.previousPosition = clone.transform.position;
                    }
                }
            }
        }
    }

    void OnBatteryReceived(UInt8Msg msg)
    {
        if (batteryText != null)
        {
            int battery = msg.data;
            batteryText.text = battery + "%";

            if (battery > 65)
                batteryText.color = Color.green;
            else if (battery > 20)
                batteryText.color = new Color(1f, 0.64f, 0f); // naranja
            else
                batteryText.color = Color.red;
        }
    }

    void Update()
    {
        if (isOriginal)
        {
            if (hasTarget)
            {
                Vector3 adjustedTarget = AdjustHeightWithRaycast(latestReceivedPosition, out Quaternion terrainRot);
                MoveTowards(adjustedTarget, terrainRot);
            }
        }
        else
        {
            if (sharedHasTarget && originalReference != null)
            {
                Vector3 targetPos = sharedOriginalPosition + radialOffset;
                Vector3 adjustedTarget = AdjustHeightWithRaycast(targetPos, out Quaternion terrainRot);
                MoveTowards(adjustedTarget, terrainRot);
            }
        }

        // Actualizar visibilidad del texto de batería solo si cambió el valor
        if (batteryGO != null && showBattery != previousShowBattery)
        {
            batteryGO.SetActive(showBattery);
            previousShowBattery = showBattery;
        }
        if (batteryText != null && batteryGO.activeSelf)
        {
            batteryText.transform.rotation = Camera.main.transform.rotation;
        }

        // Actualizar LineRenderer para la trayectoria del original
        if (isOriginal && lineRenderer != null)
        {
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

            if (showLine && lineRenderer.enabled)
            {
                if (Vector3.Distance(transform.position, previousPosition) > 0.1f)
                {
                    lineRenderer.positionCount++;
                    lineRenderer.SetPosition(lineRenderer.positionCount - 1, transform.position);
                    previousPosition = transform.position;
                }
            }
        }
    }

    void MoveTowards(Vector3 targetPos, Quaternion terrainRotation)
    {
        Vector3 direction = targetPos - transform.position;
        direction.y = 0;

        float distance = direction.magnitude;

        if (distance > 0.05f)
        {
            Vector3 move = direction.normalized * moveSpeed * Time.deltaTime;
            transform.position += move;

            Quaternion targetRotation = Quaternion.LookRotation(direction.normalized);
            transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, rotationSpeed * Time.deltaTime);

            if (animator != null)
                animator.SetBool("IsRunning", true);
        }
        else
        {
            if (animator != null)
                animator.SetBool("IsRunning", false);
        }

        // Corrige la altura en cada frame para mantenerse pegado al terreno
        Vector3 pos = transform.position;
        Vector3 adjusted = AdjustHeightWithRaycast(pos, out Quaternion _);
        transform.position = new Vector3(pos.x, adjusted.y, pos.z);
    }

    Vector3 ConvertGPSToUnity(double lat, double lon)
    {
        double R = 6378137.0;
        double dLat = Mathf.Deg2Rad * (lat - refLat);
        double dLon = Mathf.Deg2Rad * (lon - refLon);
        double meanLat = (lat + refLat) / 2.0;

        float x = (float)(dLat * R); // Norte
        float z = (float)(-dLon * R * Mathf.Cos((float)(meanLat * Mathf.Deg2Rad)));
        return new Vector3(x, 0, z);
    }

    Vector3 AdjustHeightWithRaycast(Vector3 basePosition, out Quaternion terrainRotation)
    {
        Vector3[] offsets = new Vector3[]
        {
            new Vector3(0.3f, 0, 0.3f),
            new Vector3(-0.3f, 0, 0.3f),
            new Vector3(0.3f, 0, -0.3f),
            new Vector3(-0.3f, 0, -0.3f)
        };

        List<Vector3> hitPoints = new();
        List<Vector3> normals = new();

        foreach (var offset in offsets)
        {
            Vector3 origin = basePosition + offset + Vector3.up * raycastHeight;
            if (Physics.Raycast(origin, Vector3.down, out RaycastHit hit, raycastHeight * 2f, terrainLayerMask))
            {
                hitPoints.Add(hit.point);
                normals.Add(hit.normal);
            }
        }

        if (hitPoints.Count >= 3)
        {
            Vector3 avgPoint = Vector3.zero;
            Vector3 avgNormal = Vector3.zero;

            foreach (var p in hitPoints) avgPoint += p;
            foreach (var n in normals) avgNormal += n;

            avgPoint /= hitPoints.Count;
            avgNormal.Normalize();

            terrainRotation = Quaternion.LookRotation(Vector3.ProjectOnPlane(transform.forward, avgNormal), avgNormal);
            return new Vector3(basePosition.x, avgPoint.y + heightOffset, basePosition.z);
        }
        else
        {
            terrainRotation = transform.rotation;
            return basePosition; // fallback sin cambiar altura
        }
    }

    void SpawnClones()
    {
        for (int i = 0; i < cloneCount; i++)
        {
            float angle = i * Mathf.PI * 2 / cloneCount;
            Vector3 offset = new Vector3(Mathf.Cos(angle), 0, Mathf.Sin(angle)) * cloneRadius;

            GameObject clone = Instantiate(humanPrefab, transform.position + offset, Quaternion.identity);

            HumanFollower hf = clone.GetComponent<HumanFollower>();
            if (hf != null)
            {
                hf.isOriginal = false;
                hf.originalReference = this;
                hf.radialOffset = offset;
                hf.animator = clone.GetComponent<Animator>();
                hf.terrainLayerMask = this.terrainLayerMask;
            }
        }
    }

    void CreateRedMarker()
    {
        GameObject marker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        marker.transform.SetParent(markerHolder);
        marker.transform.localPosition = Vector3.zero;
        marker.transform.localScale = Vector3.one * 0.1f;
        Destroy(marker.GetComponent<Collider>());
        marker.GetComponent<Renderer>().material.color = Color.red;
    }

    void CreateBatteryText()
    {
        batteryGO = new GameObject("BatteryText");
        batteryGO.transform.SetParent(markerHolder);
        batteryGO.transform.localPosition = new Vector3(0, 0.25f, 0);
        batteryText = batteryGO.AddComponent<TextMesh>();
        batteryText.fontSize = 48;
        batteryText.characterSize = 0.05f;
        batteryText.alignment = TextAlignment.Center;
        batteryText.anchor = TextAnchor.MiddleCenter;
        batteryText.text = "100%";
        batteryText.color = Color.green;
    }

    void SetupLineRenderer()
    {
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        lineRenderer.positionCount = 1;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.blue;
        lineRenderer.endColor = Color.blue;
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.enabled = false;
    }
}
