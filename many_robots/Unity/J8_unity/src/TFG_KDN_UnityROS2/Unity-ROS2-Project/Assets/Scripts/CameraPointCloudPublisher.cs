
using System;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

[DisallowMultipleComponent]
public class CameraPointCloudTiledPublisher : MonoBehaviour
{
    [Header("Fuente")]
    public Camera sourceCamera;

    [Header("ROS")]
    public string topicName = "/camera/points";
    public string frameId   = "camera_link";

    [Header("Frecuencia")]
    public float maxPublishHz = 5f;          // Máximo de publicaciones por segundo

    [Header("Resolución lógica")]
    public int imageWidth  = 1920;
    public int imageHeight = 1080;
    public int pixelStep   = 3;              // 1=full, 2, 3...

    [Header("ROI vertical (fracción [0..1])")]
    [Range(0f,1f)] public float roiVMin = 0.35f;   // parte baja (suelo)
    [Range(0f,1f)] public float roiVMax = 1.00f;

    [Header("Rango (m)")]
    public float minRange = 0.2f;
    public float maxRange = 35f;

    [Header("Colisiones")]
    public LayerMask layers = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;

    [Header("Opciones")]
    public bool fillNaNForMiss = true;
    public bool useOpticalFrame = false;      // true => X right, Y down, Z forward
    public bool drawRays = false;

    [Header("Rendimiento (tiling)")]
    public int tilesY = 4;                    // nº de trozos verticales por nube (>=2 recomendado)
    public int batchSize = 128;               // para ScheduleBatch

    // ---- Internos ----
    ROSConnection ros;

    // Grid final (tras step + ROI)
    int W, H, N;
    int startPx, endPx;        // ROI en píxeles fuente
    int tileH;                 // altura (en filas) de cada tile
    int lastTileH;             // altura del último tile
    int tilesDone;             // tiles completados de la nube actual
    int curTile;               // índice del tile que vamos a calcular este frame [0..tilesY-1]

    // LUT completa de direcciones locales (H*W)
    Vector3[] dirLocalLut;

    // Buffers nativos por-tile (reutilizados con tamaño = tileMaxRays)
    NativeArray<RaycastCommand> commands;
    NativeArray<RaycastHit> results;

    // Buffer final ROS (nube completa)
    byte[] dataBuffer;
    bool anyNaN;
    byte[] nanBytes = BitConverter.GetBytes(float.NaN);

    // cachés
    float cachedVFOV, cachedAspect, cachedVMin, cachedVMax;
    int cachedStep, cachedW, cachedH, cachedTilesY;
    double lastPublishTime;

    void Awake()
    {
        if (sourceCamera == null) sourceCamera = GetComponent<Camera>();
        Physics.queriesHitBackfaces = false;
        if (triggerInteraction == QueryTriggerInteraction.Ignore)
            Physics.queriesHitTriggers = false;
    }

    void Start()
    {
        if (sourceCamera == null)
        {
            Debug.LogError("[CameraPCD Tiled] Asigna una Camera.");
            enabled = false; return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PointCloud2Msg>(topicName);

        ClampParams();
        RebuildAll();
    }

    void OnDestroy()
    {
        DisposeNative();
    }

    void Update()
    {
        if (NeedsRebuild())
            RebuildAll();

        // Calcula 1 tile por frame (puedes ampliar a 2 si te sobra CPU)
        ComputeOneTile(curTile);

        curTile++;
        tilesDone++;
        if (curTile >= tilesY) curTile = 0;    // siguiente ciclo

        // ¿Publicamos ya?
        if (tilesDone >= tilesY)
        {
            double now = Time.realtimeSinceStartupAsDouble;
            double minPeriod = 1.0 / Math.Max(0.1, maxPublishHz);
            if (now - lastPublishTime >= minPeriod)
            {
                PublishNow();
                lastPublishTime = now;
            }
            tilesDone = 0;
        }
    }

    // ----------------- Cálculo de un tile -----------------
    void ComputeOneTile(int tileIdx)
    {
        if (!commands.IsCreated || !results.IsCreated) return;

        int y0 = tileIdx * tileH;   // fila inicial dentro del grid H
        int h  = (tileIdx == tilesY - 1) ? lastTileH : tileH;
        if (h <= 0) return;

        Transform camT = sourceCamera.transform;
        Vector3 origin = camT.position;
        Quaternion rot = camT.rotation;

        int rays = W * h;
        for (int r = 0; r < rays; r++)
        {
            int jy = y0 + (r / W);
            int ix = r % W;
            int idx = jy * W + ix;
            Vector3 dirWorld = rot * dirLocalLut[idx];
            commands[r] = new RaycastCommand(origin, dirWorld, maxRange, layers, 1);
        }

        var handle = RaycastCommand.ScheduleBatch(commands, results, Mathf.Max(1, batchSize), default);
        handle.Complete();

        // Escribir en dataBuffer (organizada HxW) solo las filas del tile
        int pointStep = 12;
        for (int r = 0; r < rays; r++)
        {
            int jy = y0 + (r / W);
            int ix = r % W;
            int idx = jy * W + ix;
            int off = (idx) * pointStep;
            var hit = results[r];

            if (hit.collider != null && hit.distance >= minRange && hit.distance <= maxRange)
            {
                // pLocal = dirLocal * distance
                Vector3 pLocal = dirLocalLut[idx] * hit.distance;

                float x, y, z;
                if (useOpticalFrame) { x = +pLocal.x; y = -pLocal.y; z = +pLocal.z; }
                else                 { x =  pLocal.z; y = -pLocal.x; z =  pLocal.y; }

                Buffer.BlockCopy(BitConverter.GetBytes(x), 0, dataBuffer, off + 0, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(y), 0, dataBuffer, off + 4, 4);
                Buffer.BlockCopy(BitConverter.GetBytes(z), 0, dataBuffer, off + 8, 4);

                if (drawRays)
                    Debug.DrawRay(origin, (rot * dirLocalLut[idx]) * hit.distance, Color.green, 0.02f, false);
            }
            else
            {
                if (fillNaNForMiss)
                {
                    anyNaN = true;
                    Buffer.BlockCopy(nanBytes, 0, dataBuffer, off + 0, 4);
                    Buffer.BlockCopy(nanBytes, 0, dataBuffer, off + 4, 4);
                    Buffer.BlockCopy(nanBytes, 0, dataBuffer, off + 8, 4);
                }
                else anyNaN = true;

                if (drawRays)
                    Debug.DrawRay(origin, (rot * dirLocalLut[idx]) * maxRange, Color.red, 0.02f, false);
            }
        }
    }

    // ----------------- Publicación -----------------
    void PublishNow()
    {
        var utc = DateTime.UtcNow;
        int  sec  = (int)new DateTimeOffset(utc).ToUnixTimeSeconds();
        uint nsec = (uint)((utc.Ticks % TimeSpan.TicksPerSecond) * 100);

        var msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg { sec = sec, nanosec = nsec },
                frame_id = frameId
            },
            height = (uint)H,
            width  = (uint)W,
            fields = new PointFieldMsg[]
            {
                new PointFieldMsg("x", 0, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("y", 4, PointFieldMsg.FLOAT32, 1),
                new PointFieldMsg("z", 8, PointFieldMsg.FLOAT32, 1),
            },
            is_bigendian = false,
            point_step = 12,
            row_step   = (uint)(W * 12),
            data = dataBuffer,
            is_dense = !anyNaN
        };

        ros.Publish(topicName, msg);
    }

    // ----------------- Reconstrucción/LUT -----------------
    void ClampParams()
    {
        maxPublishHz = Mathf.Max(0.1f, maxPublishHz);
        imageWidth   = Mathf.Max(1, imageWidth);
        imageHeight  = Mathf.Max(1, imageHeight);
        pixelStep    = Mathf.Max(1, pixelStep);
        minRange     = Mathf.Max(0f, minRange);
        maxRange     = Mathf.Max(minRange + 0.01f, maxRange);
        roiVMin      = Mathf.Clamp01(roiVMin);
        roiVMax      = Mathf.Clamp01(roiVMax);
        if (roiVMax < roiVMin) { var t = roiVMin; roiVMin = roiVMax; roiVMax = t; }
        tilesY       = Mathf.Max(1, tilesY);
        batchSize    = Mathf.Max(1, batchSize);
    }

    bool NeedsRebuild()
    {
        float aspect = (float)imageWidth / Mathf.Max(1, imageHeight);
        int targetW = (imageWidth + pixelStep - 1) / pixelStep;

        // ROI en píxeles
        int sPx = Mathf.Clamp(Mathf.FloorToInt(roiVMin * imageHeight), 0, imageHeight - 1);
        int ePx = Mathf.Clamp(Mathf.CeilToInt (roiVMax * imageHeight) - 1, 0, imageHeight - 1);
        int Hraw = Mathf.Max(1, ePx - sPx + 1);
        int targetH = (Hraw + pixelStep - 1) / pixelStep;

        return
            targetW != W || targetH != H ||
            cachedStep != pixelStep ||
            !Mathf.Approximately(cachedVFOV, sourceCamera.fieldOfView) ||
            !Mathf.Approximately(cachedAspect, aspect) ||
            !Mathf.Approximately(cachedVMin, roiVMin) ||
            !Mathf.Approximately(cachedVMax, roiVMax) ||
            cachedTilesY != tilesY;
    }

    void RebuildAll()
    {
        ClampParams();

        startPx = Mathf.Clamp(Mathf.FloorToInt(roiVMin * imageHeight), 0, imageHeight - 1);
        endPx   = Mathf.Clamp(Mathf.CeilToInt (roiVMax * imageHeight) - 1, 0, imageHeight - 1);
        int Hraw = Mathf.Max(1, endPx - startPx + 1);

        W = (imageWidth + pixelStep - 1) / pixelStep;
        H = (Hraw      + pixelStep - 1) / pixelStep;
        N = W * H;

        cachedVFOV   = sourceCamera.fieldOfView;
        cachedAspect = (float)imageWidth / Mathf.Max(1, imageHeight);
        cachedVMin   = roiVMin;
        cachedVMax   = roiVMax;
        cachedStep   = pixelStep;
        cachedTilesY = tilesY;

        BuildDirLocalLUT();
        BuildTiling();
        AllocateNativeBuffers();

        // reset ciclo
        curTile = 0;
        tilesDone = 0;
        anyNaN = false;
    }

    void BuildDirLocalLUT()
    {
        if (W <= 0 || H <= 0) return;

        float tanHalf = Mathf.Tan(0.5f * Mathf.Deg2Rad * cachedVFOV);
        dirLocalLut = new Vector3[N];

        int idx = 0;
        for (int jy = 0; jy < H; jy++)
        {
            float py = startPx + jy * pixelStep + 0.5f;
            float v = py / imageHeight;
            float y = (1f - 2f * v) * tanHalf;

            for (int ix = 0; ix < W; ix++, idx++)
            {
                float px = ix * pixelStep + 0.5f;
                float u = px / imageWidth;
                float x = (2f * u - 1f) * tanHalf * cachedAspect;
                Vector3 d = new Vector3(x, y, 1f).normalized; // local cámara
                dirLocalLut[idx] = d;
            }
        }
    }

    void BuildTiling()
    {
        tileH = H / tilesY;
        lastTileH = H - tileH * (tilesY - 1);
        if (tileH <= 0) { tileH = H; lastTileH = H; tilesY = 1; }
    }

    void AllocateNativeBuffers()
    {
        DisposeNative();

        int tileMaxRows = Mathf.Max(tileH, lastTileH);
        int tileMaxRays = Mathf.Max(1, W * tileMaxRows);

        commands = new NativeArray<RaycastCommand>(tileMaxRays, Allocator.Persistent);
        results  = new NativeArray<RaycastHit>(tileMaxRays, Allocator.Persistent);

        dataBuffer = new byte[Mathf.Max(1, N * 12)];
    }

    void DisposeNative()
    {
        if (commands.IsCreated) commands.Dispose();
        if (results.IsCreated)  results.Dispose();
    }
}
