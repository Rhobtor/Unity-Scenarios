using System;
using UnityEngine;
using RosMessageTypes.BuiltinInterfaces;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;

[DisallowMultipleComponent]
public class FixpositionOdometryPublisher : MonoBehaviour
{
    [Header("Source")]
    public GameObject fixpositionObject;

    [Header("ROS")]
    public string topicName = "/fixposition/odometry_enu";
    public bool publishLegacyTopic = true;
    public string legacyTopicName = "/fixposition/odometry";
    public string frameId = "odom";
    public string childFrameId = "base_link";

    [Header("Timing")]
    public float publishFrequency = 10.0f;

    [Header("Debug")]
    public bool logPublishedPose = false;

    private ROSConnection ros;
    private float publishTimeElapsed;
    private Vector3 lastPosition;
    private Quaternion lastRotation;

    void Start()
    {
        if (fixpositionObject == null)
        {
            Debug.LogError("FixpositionOdometryPublisher: fixpositionObject no está asignado.");
            enabled = false;
            return;
        }

        if (publishFrequency <= 0f)
        {
            Debug.LogError("FixpositionOdometryPublisher: publishFrequency debe ser mayor que 0.");
            enabled = false;
            return;
        }

        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>(topicName);

        if (publishLegacyTopic && topicName != legacyTopicName)
        {
            ros.RegisterPublisher<OdometryMsg>(legacyTopicName);
        }

        lastPosition = fixpositionObject.transform.position;
        lastRotation = fixpositionObject.transform.rotation;

        Debug.Log($"FixpositionOdometryPublisher publicando {fixpositionObject.name} en {topicName}.");
    }

    void Update()
    {
        publishTimeElapsed += Time.deltaTime;

        if (publishTimeElapsed >= 1.0f / publishFrequency)
        {
            PublishOdometryData(publishTimeElapsed);
            publishTimeElapsed = 0f;
        }
    }

    void PublishOdometryData(float dt)
    {
        Vector3 currentPosition = fixpositionObject.transform.position;
        Quaternion currentRotation = fixpositionObject.transform.rotation;

        Vector3 linearVelocity = (currentPosition - lastPosition) / dt;

        Quaternion deltaRotation = currentRotation * Quaternion.Inverse(lastRotation);
        deltaRotation.ToAngleAxis(out float angleDegrees, out Vector3 axis);
        Vector3 angularVelocity = axis * Mathf.Deg2Rad * angleDegrees / dt;

        OdometryMsg odometryMessage = new OdometryMsg
        {
            header = new HeaderMsg
            {
                stamp = GetCurrentRosTime(),
                frame_id = frameId
            },
            child_frame_id = childFrameId,
            pose = new PoseWithCovarianceMsg
            {
                pose = new PoseMsg
                {
                    position = new PointMsg
                    {
                        x = currentPosition.x,
                        y = currentPosition.y,
                        z = currentPosition.z
                    },
                    orientation = new QuaternionMsg
                    {
                        x = currentRotation.x,
                        y = currentRotation.y,
                        z = currentRotation.z,
                        w = currentRotation.w
                    }
                },
                covariance = new double[36]
            },
            twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = new Vector3Msg
                    {
                        x = linearVelocity.x,
                        y = linearVelocity.y,
                        z = linearVelocity.z
                    },
                    angular = new Vector3Msg
                    {
                        x = angularVelocity.x,
                        y = angularVelocity.y,
                        z = angularVelocity.z
                    }
                },
                covariance = new double[36]
            }
        };

        ros.Publish(topicName, odometryMessage);
        if (publishLegacyTopic && topicName != legacyTopicName)
        {
            ros.Publish(legacyTopicName, odometryMessage);
        }

        if (logPublishedPose)
        {
            Debug.Log($"Odometry -> Pos=({currentPosition.x:F3}, {currentPosition.y:F3}, {currentPosition.z:F3}) Topic={topicName}");
        }

        lastPosition = currentPosition;
        lastRotation = currentRotation;
    }

    static TimeMsg GetCurrentRosTime()
    {
        DateTime utcNow = DateTime.UtcNow;
        int seconds = (int)new DateTimeOffset(utcNow).ToUnixTimeSeconds();
        uint nanoseconds = (uint)((utcNow.Ticks % TimeSpan.TicksPerSecond) * 100);
        return new TimeMsg(seconds, nanoseconds);
    }
}