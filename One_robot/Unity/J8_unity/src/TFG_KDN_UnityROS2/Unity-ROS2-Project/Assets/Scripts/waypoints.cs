using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Geometry;

public class PoseArrayWaypointVisualizer : MonoBehaviour
{
    [Header("ROS")]
    public string topicName = "/waypoints";  // tu tópico PoseArray
    public ROSConnection ros;

    [Header("Placement / Frame")]
    public Transform rosWorldOrigin;   // opcional: origen del frame (map) en Unity
    public float yOffset = 0.05f;      // levanta un poco los puntos
    public bool assumeFLU = true;      // ROS normalmente FLU (x forward, y left, z up)

    [Header("Rendering")]
    public float pointScale = 0.25f;   // tamaño de los puntos
    public Color pointColor = new Color(1f, 0f, 0f, 0.95f);
    public bool showAllPoints = true;  // si false, muestra solo el primero
    public bool updateOnMessage = true;// si true, actualiza solo al recibir msg

    // pool
    readonly List<GameObject> _points = new List<GameObject>();
    Material _mat;

    void Start()
    {
        if (ros == null) ros = ROSConnection.GetOrCreateInstance();

        _mat = CreateUnlitMaterial(pointColor);

        ros.Subscribe<PoseArrayMsg>(topicName, OnPoseArray);
    }

    void OnPoseArray(PoseArrayMsg msg)
    {
        int count = msg.poses.Length;
        if (!showAllPoints) count = Mathf.Min(1, count);

        EnsurePool(count);

        // Si no hay puntos, desactiva todo
        for (int i = 0; i < _points.Count; i++)
            _points[i].SetActive(i < count);

        // Coloca puntos
        for (int i = 0; i < count; i++)
        {
            Vector3 unityPos;

            if (assumeFLU)
            {
                // ✅ Correcto: llamar From<FLU>() sobre PointMsg
                unityPos = msg.poses[i].position.From<FLU>();
            }
            else
            {
                var p = msg.poses[i].position;
                unityPos = new Vector3((float)p.x, (float)p.y, (float)p.z);
            }

            if (rosWorldOrigin != null)
                unityPos = rosWorldOrigin.TransformPoint(unityPos);

            unityPos.y += yOffset;
            _points[i].transform.position = unityPos;
        }
    }

    void EnsurePool(int needed)
    {
        while (_points.Count < needed)
        {
            var go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            go.name = "wp_point";
            go.transform.SetParent(transform, true);
            go.transform.localScale = Vector3.one * pointScale;

            // sin collider (mejor para perf)
            var col = go.GetComponent<Collider>();
            if (col) Destroy(col);

            var r = go.GetComponent<Renderer>();
            r.sharedMaterial = _mat;

            _points.Add(go);
        }

        // si cambias el scale en inspector en runtime, aplica a todos
        for (int i = 0; i < _points.Count; i++)
            _points[i].transform.localScale = Vector3.one * pointScale;
    }

    Material CreateUnlitMaterial(Color c)
    {
        // Unlit/Color funciona bien para que se vea claro en capturas
        Shader sh = Shader.Find("Unlit/Color");
        if (sh == null) sh = Shader.Find("Sprites/Default");
        var m = new Material(sh);

        // Unlit/Color usa _Color, Sprites/Default usa _Color también
        if (m.HasProperty("_Color")) m.SetColor("_Color", c);
        return m;
    }
}