using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

using RosMessageTypes.Tf2;               // tf2_msgs/TFMessage
using RosMessageTypes.Geometry;          // geometry_msgs/Transform, Vector3, Quaternion
using RosMessageTypes.Std;               // std_msgs/Header   <-- ¡IMPORTANTE!
using RosMessageTypes.BuiltinInterfaces; // builtin_interfaces/Time

[DisallowMultipleComponent]
public class BaseLinkTFPublisher : MonoBehaviour
{
    public string parentFrame = "odom";
    public string childFrame  = "base_link";
    public float publishRate  = 30f; // Hz

    ROSConnection ros;
    float tAccum;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<TFMessageMsg>("/tf");
    }

    void Update()
    {
        tAccum += Time.deltaTime;
        if (tAccum >= 1f / publishRate)
        {
            tAccum = 0f;
            PublishTF();
        }
    }

    void PublishTF()
    {
        // --- POSICIÓN: Unity -> ROS base (x=z_u, y=-x_u, z=y_u) ---
        Vector3 pU = transform.position;
        var translation = new Vector3Msg(pU.z, -pU.x, pU.y);

        // --- ORIENTACIÓN: yaw plano (ROS yaw = - Unity yaw) ---
        float yawUnity = Mathf.Atan2(transform.forward.x, transform.forward.z);
        float yawRos   = -yawUnity;
        float half     = 0.5f * yawRos;
        var rotation   = new QuaternionMsg(0f, 0f, Mathf.Sin(half), Mathf.Cos(half));

        var t = new TransformMsg(translation, rotation);

        // Header explícito
        var header = new HeaderMsg();
        header.stamp = NowRosTime();
        header.frame_id = parentFrame;

        var ts = new TransformStampedMsg();
        ts.header = header;
        ts.child_frame_id = childFrame;
        ts.transform = t;

        var tf = new TFMessageMsg(new TransformStampedMsg[] { ts });
        ros.Publish("/tf", tf);
    }

    static TimeMsg NowRosTime()
    {
        var utc = System.DateTime.UtcNow;
        int  sec  = (int)new System.DateTimeOffset(utc).ToUnixTimeSeconds();
        uint nsec = (uint)((utc.Ticks % System.TimeSpan.TicksPerSecond) * 100);
        return new TimeMsg(sec, nsec);
    }
}
