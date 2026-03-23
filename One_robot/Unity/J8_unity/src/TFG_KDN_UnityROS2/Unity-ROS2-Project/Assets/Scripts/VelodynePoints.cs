using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;

[DisallowMultipleComponent]
public class VelodyneVLP16RealisticPublisher : MonoBehaviour
{
    public enum VerticalAnglesMode
    {
        VLP16Preset,
        Custom
    }

    public enum ReturnMode
    {
        FirstReturnFast,      // rápido: 1er impacto con RaycastCommand
        StrongestApproxSlow,  // lento: varios impactos y elige mayor intensidad estimada
        LastApproxSlow,       // lento: varios impactos y elige el más lejano
        DualApproxSlow        // lento: publica strongest + last si son distintos
    }

    [Serializable]
    public class LayerResponseRule
    {
        public string name = "Rule";
        public LayerMask layers = ~0;

        [Range(0f, 1f)] public float reflectivity01 = 0.45f;
        [Range(0f, 1f)] public float extraDropout = 0.0f;
        [Range(0.1f, 5f)] public float rangeNoiseMultiplier = 1.0f;
    }

    // Orden típico de firing del VLP-16
    static readonly float[] kVlp16AnglesDeg =
    {
        -15f,  1f, -13f,  3f,
        -11f,  5f,  -9f,  7f,
         -7f,  9f,  -5f, 11f,
         -3f, 13f,  -1f, 15f
    };

    const float FIRING_SEQUENCE_SEC = 55.296e-6f;
    const float LASER_FIRE_SEC      =  2.304e-6f;

    [Header("Origen")]
    public Transform lidarOrigin;
    public Rigidbody motionSource; // opcional, para distorsión realista por movimiento

    [Header("ROS")]
    public string topicName = "/velodyne_points";
    public string frameId = "velodyne";
    public bool publishInRosFrame = true; // x forward, y left, z up

    [Header("Modelo VLP-16")]
    [Range(300f, 1200f)] public float rpm = 600f;
    public VerticalAnglesMode verticalAnglesMode = VerticalAnglesMode.VLP16Preset;
    public float[] customVerticalAnglesDeg = new float[16];

    [Header("Recorte horizontal")]
    public bool limitAzimuth = false;
    [Tooltip("0=delante, 90=izquierda, 180=detrás, 270=derecha")]
    public float azimuthStartDeg = -90f;
    [Range(0.1f, 360f)] public float azimuthSweepDeg = 180f;

    [Header("Returns")]
    public ReturnMode returnMode = ReturnMode.DualApproxSlow;
    [Tooltip("Separación mínima para considerar dos returns distintos en Dual.")]
    public float dualMinSeparationMeters = 0.20f;
    [Tooltip("Máximo número de impactos a inspeccionar por rayo en modos lentos.")]
    [Range(2, 16)] public int maxHitsPerRay = 8;

    [Header("Rango")]
    public float minRange = 1.0f;
    public float maxRange = 100f;

    [Header("Colisiones")]
    public LayerMask layers = ~0;
    public QueryTriggerInteraction triggerInteraction = QueryTriggerInteraction.Ignore;
    public bool ignoreBackfaces = true;

    [Header("Distorsión por movimiento")]
    public bool enableMotionDistortion = true;

    [Header("Ruido de distancia")]
    public bool enableRangeNoise = true;
    [Tooltip("Ruido base (1 sigma) en metros.")]
    public float rangeNoiseStdMeters = 0.02f;
    [Tooltip("Bias global en metros.")]
    public float globalRangeBiasMeters = 0.0f;
    [Tooltip("Bias adicional por anillo.")]
    public float[] rangeBiasByRingMeters = new float[16];
    [Tooltip("Multiplicador de ruido por anillo.")]
    public float[] rangeNoiseScaleByRing = new float[16];

    [Header("Jitter angular del haz")]
    public bool enableAngularJitter = true;
    [Tooltip("Sigma horizontal en grados.")]
    public float azimuthJitterStdDeg = 0.03f;
    [Tooltip("Sigma vertical en grados.")]
    public float elevationJitterStdDeg = 0.03f;

    [Header("Dropout / defectos")]
    [Range(0f, 1f)] public float baseDropoutProbability = 0.01f;
    public bool enableIncidenceDropout = true;
    [Range(0f, 89f)] public float grazingStartDeg = 75f;
    [Range(0f, 89.9f)] public float grazingEndDeg = 88.5f;
    [Tooltip("Aumenta dropout con distancia larga.")]
    [Range(0f, 1f)] public float farRangeDropoutBoost = 0.08f;

    [Header("Falsos retornos / speckles")]
    public bool enableFalseReturns = false;
    [Range(0f, 1f)] public float falseReturnProbability = 0.002f;
    public float falseReturnMinRange = 2.0f;
    public float falseReturnMaxRange = 12.0f;
    [Range(0f, 255f)] public float falseReturnIntensity = 18f;

    [Header("Intensidad")]
    [Range(0f, 1f)] public float defaultReflectivity01 = 0.45f;
    [Range(0.1f, 8f)] public float incidencePower = 1.5f;
    [Range(0f, 255f)] public float intensityNoiseStd = 4f;
    public List<LayerResponseRule> layerResponses = new List<LayerResponseRule>();

    [Header("Campos PointCloud2")]
    public bool addIntensityField = true;
    public bool addRingField = true;
    public bool addTimeField = true;
    public bool addReturnTypeField = true;

    [Header("Rendimiento")]
    public int batchSize = 512;
    public bool autoRebuildInPlayMode = true;

    [Header("Debug")]
    public bool drawRays = false;
    public float debugRayDuration = 0.02f;
    public Color hitColor = Color.green;
    public Color missColor = Color.red;

    [Header("Semilla")]
    public int randomSeed = 12345;

    // Internos
    ROSConnection ros;
    string registeredTopic;
    System.Random rng;

    float[] activeVerticalAngles;
    int channels;
    int sequenceCount;
    int pointCountSingle;
    int pointCapacityMax;

    NativeArray<RaycastCommand> commands;
    NativeArray<RaycastHit> results;

    Vector3[] cachedOrigins;
    Vector3[] cachedDirsWorld;
    float[] cachedTimes;
    ushort[] cachedRings;
    float[] cachedAzimuthsDeg;
    float[] cachedElevationsDeg;

    byte[] scratchBuffer;

    PointFieldMsg[] fields;
    int pointStep;
    int intensityOffset = -1;
    int ringOffset = -1;
    int timeOffset = -1;
    int returnTypeOffset = -1;

    double nextScanTime;

    // Cachés rebuild
    float cachedRpm;
    float cachedMinRange;
    float cachedMaxRange;
    ReturnMode cachedReturnMode;
    VerticalAnglesMode cachedVerticalAnglesMode;
    bool cachedPublishInRosFrame;
    bool cachedAddIntensity;
    bool cachedAddRing;
    bool cachedAddTime;
    bool cachedAddReturnType;
    int cachedMaxHitsPerRay;

    void Reset()
    {
        lidarOrigin = transform;
        motionSource = GetComponentInParent<Rigidbody>();
        LoadVLP16PresetIntoCustom();
        InitPerRingArrays();
    }

    void Awake()
    {
        if (lidarOrigin == null) lidarOrigin = transform;
        if (motionSource == null) motionSource = GetComponentInParent<Rigidbody>();

        LoadArraysIfNeeded();
        ClampParams();
        rng = new System.Random(randomSeed);

        Physics.queriesHitBackfaces = !ignoreBackfaces;
        if (triggerInteraction == QueryTriggerInteraction.Ignore)
            Physics.queriesHitTriggers = false;
    }

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        EnsurePublisherRegistered();
        RebuildAll();

        nextScanTime = Time.realtimeSinceStartupAsDouble;
    }

    void OnValidate()
    {
        LoadArraysIfNeeded();
        ClampParams();

        if (Application.isPlaying && autoRebuildInPlayMode && NeedsRebuild())
            RebuildAll();
    }

    void OnDestroy()
    {
        DisposeNative();
    }

    [ContextMenu("Cargar preset VLP-16 en Custom")]
    public void LoadVLP16PresetIntoCustom()
    {
        customVerticalAnglesDeg = new float[kVlp16AnglesDeg.Length];
        Array.Copy(kVlp16AnglesDeg, customVerticalAnglesDeg, kVlp16AnglesDeg.Length);
    }

    void LoadArraysIfNeeded()
    {
        if (customVerticalAnglesDeg == null || customVerticalAnglesDeg.Length == 0)
            LoadVLP16PresetIntoCustom();

        InitPerRingArrays();
    }

    void InitPerRingArrays()
    {
        int n = GetVerticalAngleArray().Length;
        EnsureArraySize(ref rangeBiasByRingMeters, n, 0f);
        EnsureArraySize(ref rangeNoiseScaleByRing, n, 1f);
    }

    static void EnsureArraySize(ref float[] arr, int size, float defaultValue)
    {
        if (arr != null && arr.Length == size) return;

        float[] newArr = new float[size];
        if (arr != null)
        {
            int copy = Mathf.Min(arr.Length, size);
            Array.Copy(arr, newArr, copy);
            for (int i = copy; i < size; i++) newArr[i] = defaultValue;
        }
        else
        {
            for (int i = 0; i < size; i++) newArr[i] = defaultValue;
        }
        arr = newArr;
    }

    float[] GetVerticalAngleArray()
    {
        return verticalAnglesMode == VerticalAnglesMode.VLP16Preset
            ? kVlp16AnglesDeg
            : customVerticalAnglesDeg;
    }

    void ClampParams()
    {
        rpm = Mathf.Clamp(rpm, 300f, 1200f);
        minRange = Mathf.Max(0.01f, minRange);
        maxRange = Mathf.Max(minRange + 0.01f, maxRange);
        batchSize = Mathf.Max(1, batchSize);
        maxHitsPerRay = Mathf.Clamp(maxHitsPerRay, 2, 16);
        dualMinSeparationMeters = Mathf.Max(0.01f, dualMinSeparationMeters);
        falseReturnMinRange = Mathf.Max(minRange, falseReturnMinRange);
        falseReturnMaxRange = Mathf.Max(falseReturnMinRange + 0.01f, falseReturnMaxRange);
        azimuthJitterStdDeg = Mathf.Max(0f, azimuthJitterStdDeg);
        elevationJitterStdDeg = Mathf.Max(0f, elevationJitterStdDeg);
        rangeNoiseStdMeters = Mathf.Max(0f, rangeNoiseStdMeters);
        intensityNoiseStd = Mathf.Max(0f, intensityNoiseStd);
        grazingEndDeg = Mathf.Clamp(grazingEndDeg, 0f, 89.9f);
        grazingStartDeg = Mathf.Clamp(grazingStartDeg, 0f, grazingEndDeg);
        azimuthSweepDeg = Mathf.Clamp(azimuthSweepDeg, 0.1f, 360f);
    }

    float Normalize360(float a)
    {
        a %= 360f;
        if (a < 0f) a += 360f;
        return a;
    }

    bool IsAzimuthInside(float azDeg)
    {
        if (!limitAzimuth || azimuthSweepDeg >= 360f)
            return true;

        float a = Normalize360(azDeg);
        float start = Normalize360(azimuthStartDeg);
        float end = Normalize360(azimuthStartDeg + azimuthSweepDeg);

        if (start <= end)
            return a >= start && a <= end;

        return a >= start || a <= end;
    }

    void Update()
    {
        EnsurePublisherRegistered();

        Physics.queriesHitBackfaces = !ignoreBackfaces;
        if (triggerInteraction == QueryTriggerInteraction.Ignore)
            Physics.queriesHitTriggers = false;

        if (autoRebuildInPlayMode && NeedsRebuild())
            RebuildAll();

        double now = Time.realtimeSinceStartupAsDouble;
        double scanPeriod = 60.0 / rpm;

        while (now >= nextScanTime)
        {
            SimulateAndPublishOneRevolution();
            nextScanTime += scanPeriod;
        }
    }

    void EnsurePublisherRegistered()
    {
        if (ros == null) return;

        if (registeredTopic != topicName)
        {
            ros.RegisterPublisher<PointCloud2Msg>(topicName);
            registeredTopic = topicName;
        }
    }

    bool NeedsRebuild()
    {
        float[] angles = GetVerticalAngleArray();

        if (activeVerticalAngles == null || activeVerticalAngles.Length != angles.Length) return true;
        for (int i = 0; i < angles.Length; i++)
            if (!Mathf.Approximately(activeVerticalAngles[i], angles[i])) return true;

        return
            !Mathf.Approximately(cachedRpm, rpm) ||
            !Mathf.Approximately(cachedMinRange, minRange) ||
            !Mathf.Approximately(cachedMaxRange, maxRange) ||
            cachedReturnMode != returnMode ||
            cachedVerticalAnglesMode != verticalAnglesMode ||
            cachedPublishInRosFrame != publishInRosFrame ||
            cachedAddIntensity != addIntensityField ||
            cachedAddRing != addRingField ||
            cachedAddTime != addTimeField ||
            cachedAddReturnType != addReturnTypeField ||
            cachedMaxHitsPerRay != maxHitsPerRay ||
            commands.Length == 0 ||
            scratchBuffer == null;
    }

    void RebuildAll()
    {
        ClampParams();

        activeVerticalAngles = (float[])GetVerticalAngleArray().Clone();
        channels = activeVerticalAngles.Length;

        float scanPeriod = 60f / rpm;
        sequenceCount = Mathf.Max(1, Mathf.FloorToInt(scanPeriod / FIRING_SEQUENCE_SEC));

        pointCountSingle = sequenceCount * channels;
        pointCapacityMax = pointCountSingle * 2;

        BuildFieldsLayout();
        AllocateBuffers();

        cachedRpm = rpm;
        cachedMinRange = minRange;
        cachedMaxRange = maxRange;
        cachedReturnMode = returnMode;
        cachedVerticalAnglesMode = verticalAnglesMode;
        cachedPublishInRosFrame = publishInRosFrame;
        cachedAddIntensity = addIntensityField;
        cachedAddRing = addRingField;
        cachedAddTime = addTimeField;
        cachedAddReturnType = addReturnTypeField;
        cachedMaxHitsPerRay = maxHitsPerRay;
    }

    void BuildFieldsLayout()
    {
        int offset = 0;
        List<PointFieldMsg> list = new List<PointFieldMsg>();

        list.Add(new PointFieldMsg("x", (uint)offset, PointFieldMsg.FLOAT32, 1)); offset += 4;
        list.Add(new PointFieldMsg("y", (uint)offset, PointFieldMsg.FLOAT32, 1)); offset += 4;
        list.Add(new PointFieldMsg("z", (uint)offset, PointFieldMsg.FLOAT32, 1)); offset += 4;

        intensityOffset = -1;
        ringOffset = -1;
        timeOffset = -1;
        returnTypeOffset = -1;

        if (addIntensityField)
        {
            intensityOffset = offset;
            list.Add(new PointFieldMsg("intensity", (uint)offset, PointFieldMsg.FLOAT32, 1));
            offset += 4;
        }

        if (addRingField)
        {
            ringOffset = offset;
            list.Add(new PointFieldMsg("ring", (uint)offset, PointFieldMsg.UINT16, 1));
            offset += 2;
        }

        if (addTimeField)
        {
            timeOffset = offset;
            list.Add(new PointFieldMsg("time", (uint)offset, PointFieldMsg.FLOAT32, 1));
            offset += 4;
        }

        if (addReturnTypeField)
        {
            returnTypeOffset = offset;
            list.Add(new PointFieldMsg("return_type", (uint)offset, PointFieldMsg.UINT8, 1));
            offset += 1;
        }

        fields = list.ToArray();
        pointStep = offset;
    }

    void AllocateBuffers()
    {
        DisposeNative();

        commands = new NativeArray<RaycastCommand>(Mathf.Max(1, pointCountSingle), Allocator.Persistent);
        results = new NativeArray<RaycastHit>(Mathf.Max(1, pointCountSingle), Allocator.Persistent);

        cachedOrigins = new Vector3[pointCountSingle];
        cachedDirsWorld = new Vector3[pointCountSingle];
        cachedTimes = new float[pointCountSingle];
        cachedRings = new ushort[pointCountSingle];
        cachedAzimuthsDeg = new float[pointCountSingle];
        cachedElevationsDeg = new float[pointCountSingle];

        scratchBuffer = new byte[Mathf.Max(1, pointCapacityMax * pointStep)];
    }

    void DisposeNative()
    {
        if (commands.IsCreated) commands.Dispose();
        if (results.IsCreated) results.Dispose();
    }

    void SimulateAndPublishOneRevolution()
    {
        if (ros == null || activeVerticalAngles == null || activeVerticalAngles.Length == 0)
            return;

        if (returnMode == ReturnMode.FirstReturnFast)
            SimulateFastFirstReturn();
        else
            SimulateSlowMultiReturn();
    }

    void SimulateFastFirstReturn()
    {
        Transform t = lidarOrigin != null ? lidarOrigin : transform;
        Vector3 origin0 = t.position;
        Quaternion rot0 = t.rotation;

        Vector3 linearVel = Vector3.zero;
        Vector3 angularVel = Vector3.zero;

        if (enableMotionDistortion && motionSource != null)
        {
            linearVel = motionSource.velocity;
            angularVel = motionSource.angularVelocity;
        }

        float degPerSec = rpm * 6f;

        int p = 0;
        for (int seq = 0; seq < sequenceCount; seq++)
        {
            float seqBaseTime = seq * FIRING_SEQUENCE_SEC;

            for (int ring = 0; ring < channels; ring++, p++)
            {
                float tRel = seqBaseTime + ring * LASER_FIRE_SEC;
                float azDeg = Mathf.Repeat(degPerSec * tRel, 360f);
                float elDeg = activeVerticalAngles[ring];

                cachedTimes[p] = tRel;
                cachedRings[p] = (ushort)ring;
                cachedAzimuthsDeg[p] = azDeg;
                cachedElevationsDeg[p] = elDeg;

                if (!IsAzimuthInside(azDeg))
                {
                    cachedOrigins[p] = origin0;
                    cachedDirsWorld[p] = Vector3.forward;
                    commands[p] = new RaycastCommand(origin0, Vector3.forward, 0.001f, layers, 1);
                    continue;
                }

                Vector3 origin, dirWorld;
                BuildRay(origin0, rot0, linearVel, angularVel, tRel, azDeg, elDeg, out origin, out dirWorld);

                cachedOrigins[p] = origin;
                cachedDirsWorld[p] = dirWorld;

                commands[p] = new RaycastCommand(origin, dirWorld, maxRange, layers, 1);
            }
        }

        var handle = RaycastCommand.ScheduleBatch(commands, results, Mathf.Max(1, batchSize), default);
        handle.Complete();

        int outCount = 0;

        for (int i = 0; i < pointCountSingle; i++)
        {
            if (!IsAzimuthInside(cachedAzimuthsDeg[i]))
                continue;

            RaycastHit hit = results[i];
            Vector3 origin = cachedOrigins[i];
            Vector3 dirWorld = cachedDirsWorld[i];
            float tRel = cachedTimes[i];
            ushort ring = cachedRings[i];

            if (hit.collider != null)
            {
                float measuredRange = ApplyRangeModel(hit.distance, ring, hit.collider.gameObject.layer);
                if (measuredRange >= minRange && measuredRange <= maxRange)
                {
                    if (!ShouldDropMeasurement(measuredRange, dirWorld, hit.normal, hit.collider.gameObject.layer))
                    {
                        float intensity = EvaluateIntensity(hit.collider.gameObject.layer, dirWorld, hit.normal, measuredRange);
                        WritePointByWorldDirection(
                            outCount++, origin, dirWorld, measuredRange, intensity, ring, tRel, 1);
                        if (drawRays)
                            Debug.DrawRay(origin, dirWorld * measuredRange, hitColor, debugRayDuration, false);
                        continue;
                    }
                }
            }

            if (TryMakeFalseReturn(out float falseRange))
            {
                WritePointByWorldDirection(
                    outCount++, origin, dirWorld, falseRange, falseReturnIntensity, ring, tRel, 1);
            }

            if (drawRays)
                Debug.DrawRay(origin, dirWorld * maxRange, missColor, debugRayDuration, false);
        }

        PublishBuffer(outCount);
    }

    void SimulateSlowMultiReturn()
    {
        Transform t = lidarOrigin != null ? lidarOrigin : transform;
        Vector3 origin0 = t.position;
        Quaternion rot0 = t.rotation;

        Vector3 linearVel = Vector3.zero;
        Vector3 angularVel = Vector3.zero;

        if (enableMotionDistortion && motionSource != null)
        {
            linearVel = motionSource.velocity;
            angularVel = motionSource.angularVelocity;
        }

        float degPerSec = rpm * 6f;
        RaycastHit[] hitBuffer = new RaycastHit[maxHitsPerRay];

        int outCount = 0;

        for (int seq = 0; seq < sequenceCount; seq++)
        {
            float seqBaseTime = seq * FIRING_SEQUENCE_SEC;

            for (int ring = 0; ring < channels; ring++)
            {
                float tRel = seqBaseTime + ring * LASER_FIRE_SEC;
                float azDeg = Mathf.Repeat(degPerSec * tRel, 360f);
                float elDeg = activeVerticalAngles[ring];

                if (!IsAzimuthInside(azDeg))
                    continue;

                Vector3 origin, dirWorld;
                BuildRay(origin0, rot0, linearVel, angularVel, tRel, azDeg, elDeg, out origin, out dirWorld);

                int hitCount = Physics.RaycastNonAlloc(origin, dirWorld, hitBuffer, maxRange, layers, triggerInteraction);
                if (hitCount > 1)
                    SortHitsByDistance(hitBuffer, hitCount);

                if (hitCount <= 0)
                {
                    if (TryMakeFalseReturn(out float falseRange))
                        WritePointByWorldDirection(outCount++, origin, dirWorld, falseRange, falseReturnIntensity, (ushort)ring, tRel, 1);

                    if (drawRays)
                        Debug.DrawRay(origin, dirWorld * maxRange, missColor, debugRayDuration, false);
                    continue;
                }

                if (returnMode == ReturnMode.LastApproxSlow)
                {
                    int idx = FindLastValidHit(hitBuffer, hitCount, dirWorld, ring);
                    if (idx >= 0)
                    {
                        RaycastHit h = hitBuffer[idx];
                        float mr = ApplyRangeModel(h.distance, ring, h.collider.gameObject.layer);
                        float intensity = EvaluateIntensity(h.collider.gameObject.layer, dirWorld, h.normal, mr);
                        WritePointByWorldDirection(outCount++, origin, dirWorld, mr, intensity, (ushort)ring, tRel, 2);
                        if (drawRays)
                            Debug.DrawRay(origin, dirWorld * mr, hitColor, debugRayDuration, false);
                        continue;
                    }
                }
                else if (returnMode == ReturnMode.StrongestApproxSlow)
                {
                    int idx = FindStrongestValidHit(hitBuffer, hitCount, dirWorld, ring);
                    if (idx >= 0)
                    {
                        RaycastHit h = hitBuffer[idx];
                        float mr = ApplyRangeModel(h.distance, ring, h.collider.gameObject.layer);
                        float intensity = EvaluateIntensity(h.collider.gameObject.layer, dirWorld, h.normal, mr);
                        WritePointByWorldDirection(outCount++, origin, dirWorld, mr, intensity, (ushort)ring, tRel, 1);
                        if (drawRays)
                            Debug.DrawRay(origin, dirWorld * mr, hitColor, debugRayDuration, false);
                        continue;
                    }
                }
                else
                {
                    int strongIdx = FindStrongestValidHit(hitBuffer, hitCount, dirWorld, ring);
                    int lastIdx = FindLastValidHit(hitBuffer, hitCount, dirWorld, ring);

                    bool wroteAny = false;

                    if (strongIdx >= 0)
                    {
                        RaycastHit hs = hitBuffer[strongIdx];
                        float rs = ApplyRangeModel(hs.distance, ring, hs.collider.gameObject.layer);
                        float ints = EvaluateIntensity(hs.collider.gameObject.layer, dirWorld, hs.normal, rs);
                        WritePointByWorldDirection(outCount++, origin, dirWorld, rs, ints, (ushort)ring, tRel, 1);
                        wroteAny = true;
                    }

                    if (lastIdx >= 0)
                    {
                        RaycastHit hl = hitBuffer[lastIdx];
                        float rl = ApplyRangeModel(hl.distance, ring, hl.collider.gameObject.layer);

                        bool distinct = true;
                        if (strongIdx >= 0)
                        {
                            float rs = ApplyRangeModel(hitBuffer[strongIdx].distance, ring, hitBuffer[strongIdx].collider.gameObject.layer);
                            distinct = Mathf.Abs(rl - rs) >= dualMinSeparationMeters;
                        }

                        if (distinct)
                        {
                            float intl = EvaluateIntensity(hl.collider.gameObject.layer, dirWorld, hl.normal, rl);
                            WritePointByWorldDirection(outCount++, origin, dirWorld, rl, intl, (ushort)ring, tRel, 2);
                            wroteAny = true;
                        }
                    }

                    if (wroteAny)
                    {
                        if (drawRays)
                            Debug.DrawRay(origin, dirWorld * maxRange, hitColor, debugRayDuration, false);
                        continue;
                    }
                }

                if (TryMakeFalseReturn(out float fr))
                    WritePointByWorldDirection(outCount++, origin, dirWorld, fr, falseReturnIntensity, (ushort)ring, tRel, 1);

                if (drawRays)
                    Debug.DrawRay(origin, dirWorld * maxRange, missColor, debugRayDuration, false);
            }
        }

        PublishBuffer(outCount);
    }

    int FindLastValidHit(RaycastHit[] hits, int count, Vector3 dirWorld, int ring)
    {
        for (int i = count - 1; i >= 0; i--)
        {
            RaycastHit h = hits[i];
            if (h.collider == null) continue;

            float mr = ApplyRangeModel(h.distance, ring, h.collider.gameObject.layer);
            if (mr < minRange || mr > maxRange) continue;
            if (ShouldDropMeasurement(mr, dirWorld, h.normal, h.collider.gameObject.layer)) continue;
            return i;
        }
        return -1;
    }

    int FindStrongestValidHit(RaycastHit[] hits, int count, Vector3 dirWorld, int ring)
    {
        float best = float.NegativeInfinity;
        int bestIdx = -1;

        for (int i = 0; i < count; i++)
        {
            RaycastHit h = hits[i];
            if (h.collider == null) continue;

            float mr = ApplyRangeModel(h.distance, ring, h.collider.gameObject.layer);
            if (mr < minRange || mr > maxRange) continue;
            if (ShouldDropMeasurement(mr, dirWorld, h.normal, h.collider.gameObject.layer)) continue;

            float predicted = PredictIntensityWithoutNoise(h.collider.gameObject.layer, dirWorld, h.normal, mr);
            if (predicted > best)
            {
                best = predicted;
                bestIdx = i;
            }
        }

        return bestIdx;
    }

    void BuildRay(
        Vector3 origin0, Quaternion rot0,
        Vector3 linearVel, Vector3 angularVelWorld,
        float tRel, float azDegBase, float elDegBase,
        out Vector3 origin, out Vector3 dirWorld)
    {
        origin = origin0;
        Quaternion rot = rot0;

        if (enableMotionDistortion && motionSource != null)
        {
            origin = origin0 + linearVel * tRel;

            if (angularVelWorld.sqrMagnitude > 1e-10f)
            {
                float angleDeg = angularVelWorld.magnitude * Mathf.Rad2Deg * tRel;
                Quaternion delta = Quaternion.AngleAxis(angleDeg, angularVelWorld.normalized);
                rot = delta * rot0;
            }
        }

        float az = azDegBase;
        float el = elDegBase;

        if (enableAngularJitter)
        {
            az += NextGaussian(0f, azimuthJitterStdDeg);
            el += NextGaussian(0f, elevationJitterStdDeg);
        }

        Vector3 localDir = BuildLocalDirection(az, el);
        dirWorld = rot * localDir;
    }

    static Vector3 BuildLocalDirection(float azDeg, float elDeg)
    {
        float az = azDeg * Mathf.Deg2Rad;
        float el = elDeg * Mathf.Deg2Rad;

        float cosEl = Mathf.Cos(el);
        float sinEl = Mathf.Sin(el);
        float cosAz = Mathf.Cos(az);
        float sinAz = Mathf.Sin(az);

        return new Vector3(-sinAz * cosEl, sinEl, cosAz * cosEl).normalized;
    }

    float ApplyRangeModel(float trueRange, int ring, int layer)
    {
        float measured = trueRange + globalRangeBiasMeters + rangeBiasByRingMeters[ring];

        if (enableRangeNoise)
        {
            float sigma = rangeNoiseStdMeters * rangeNoiseScaleByRing[ring] * GetLayerNoiseMultiplier(layer);
            measured += NextGaussian(0f, sigma);
        }

        return measured;
    }

    bool ShouldDropMeasurement(float range, Vector3 dirWorld, Vector3 normal, int layer)
    {
        float p = baseDropoutProbability + GetLayerExtraDropout(layer);

        float farT = Mathf.InverseLerp(0.6f * maxRange, maxRange, range);
        p += farRangeDropoutBoost * farT;

        if (enableIncidenceDropout)
        {
            float incidence = Vector3.Angle(-dirWorld, normal);
            float grazingT = Mathf.InverseLerp(grazingStartDeg, grazingEndDeg, incidence);
            p += Mathf.SmoothStep(0f, 0.35f, grazingT);
        }

        p = Mathf.Clamp01(p);
        return Next01() < p;
    }

    float EvaluateIntensity(int layer, Vector3 dirWorld, Vector3 normal, float range)
    {
        float value = PredictIntensityWithoutNoise(layer, dirWorld, normal, range);

        if (intensityNoiseStd > 0f)
            value += NextGaussian(0f, intensityNoiseStd);

        return Mathf.Clamp(value, 0f, 255f);
    }

    float PredictIntensityWithoutNoise(int layer, Vector3 dirWorld, Vector3 normal, float range)
    {
        float refl = GetLayerReflectivity(layer);

        float ndotl = Mathf.Clamp01(Vector3.Dot(normal.normalized, -dirWorld.normalized));
        float incidenceFactor = Mathf.Pow(ndotl, incidencePower);

        float distanceFactor = Mathf.Lerp(1.0f, 0.75f, Mathf.InverseLerp(minRange, maxRange, range));

        return 255f * refl * incidenceFactor * distanceFactor;
    }

    float GetLayerReflectivity(int layer)
    {
        for (int i = 0; i < layerResponses.Count; i++)
        {
            if ((layerResponses[i].layers.value & (1 << layer)) != 0)
                return layerResponses[i].reflectivity01;
        }
        return defaultReflectivity01;
    }

    float GetLayerExtraDropout(int layer)
    {
        for (int i = 0; i < layerResponses.Count; i++)
        {
            if ((layerResponses[i].layers.value & (1 << layer)) != 0)
                return layerResponses[i].extraDropout;
        }
        return 0f;
    }

    float GetLayerNoiseMultiplier(int layer)
    {
        for (int i = 0; i < layerResponses.Count; i++)
        {
            if ((layerResponses[i].layers.value & (1 << layer)) != 0)
                return layerResponses[i].rangeNoiseMultiplier;
        }
        return 1f;
    }

    bool TryMakeFalseReturn(out float falseRange)
    {
        falseRange = 0f;
        if (!enableFalseReturns) return false;
        if (Next01() >= falseReturnProbability) return false;

        falseRange = Mathf.Lerp(falseReturnMinRange, falseReturnMaxRange, Next01());
        return true;
    }

    void WritePointByWorldDirection(
        int outIndex,
        Vector3 originWorld,
        Vector3 dirWorld,
        float range,
        float intensity,
        ushort ring,
        float timeSec,
        byte returnType)
    {
        Transform frameT = lidarOrigin != null ? lidarOrigin : transform;

        Vector3 pWorld = originWorld + dirWorld.normalized * range;
        Vector3 pLocal = frameT.InverseTransformPoint(pWorld);

        float x, y, z;
        if (publishInRosFrame)
        {
            x = pLocal.z;
            y = -pLocal.x;
            z = pLocal.y;
        }
        else
        {
            x = pLocal.x;
            y = pLocal.y;
            z = pLocal.z;
        }

        int off = outIndex * pointStep;

        Buffer.BlockCopy(BitConverter.GetBytes(x), 0, scratchBuffer, off + 0, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(y), 0, scratchBuffer, off + 4, 4);
        Buffer.BlockCopy(BitConverter.GetBytes(z), 0, scratchBuffer, off + 8, 4);

        if (intensityOffset >= 0)
            Buffer.BlockCopy(BitConverter.GetBytes(intensity), 0, scratchBuffer, off + intensityOffset, 4);

        if (ringOffset >= 0)
            Buffer.BlockCopy(BitConverter.GetBytes(ring), 0, scratchBuffer, off + ringOffset, 2);

        if (timeOffset >= 0)
            Buffer.BlockCopy(BitConverter.GetBytes(timeSec), 0, scratchBuffer, off + timeOffset, 4);

        if (returnTypeOffset >= 0)
            scratchBuffer[off + returnTypeOffset] = returnType;
    }

    void PublishBuffer(int validPoints)
    {
        byte[] finalData = new byte[validPoints * pointStep];
        if (validPoints > 0)
            Buffer.BlockCopy(scratchBuffer, 0, finalData, 0, validPoints * pointStep);

        var utc = DateTime.UtcNow;
        int sec = (int)new DateTimeOffset(utc).ToUnixTimeSeconds();
        uint nsec = (uint)((utc.Ticks % TimeSpan.TicksPerSecond) * 100);

        PointCloud2Msg msg = new PointCloud2Msg
        {
            header = new HeaderMsg
            {
                stamp = new TimeMsg { sec = sec, nanosec = nsec },
                frame_id = frameId
            },
            height = 1,
            width = (uint)validPoints,
            fields = fields,
            is_bigendian = false,
            point_step = (uint)pointStep,
            row_step = (uint)(validPoints * pointStep),
            data = finalData,
            is_dense = true
        };

        ros.Publish(topicName, msg);
    }

    static void SortHitsByDistance(RaycastHit[] hits, int count)
    {
        for (int i = 1; i < count; i++)
        {
            RaycastHit key = hits[i];
            float d = key.distance;
            int j = i - 1;

            while (j >= 0 && hits[j].distance > d)
            {
                hits[j + 1] = hits[j];
                j--;
            }

            hits[j + 1] = key;
        }
    }

    float Next01()
    {
        return (float)rng.NextDouble();
    }

    float NextGaussian(float mean, float stdDev)
    {
        if (stdDev <= 0f) return mean;

        float u1 = Mathf.Max(1e-7f, Next01());
        float u2 = Mathf.Max(1e-7f, Next01());

        float mag = Mathf.Sqrt(-2f * Mathf.Log(u1));
        float z0 = mag * Mathf.Cos(2f * Mathf.PI * u2);

        return mean + z0 * stdDev;
    }
}