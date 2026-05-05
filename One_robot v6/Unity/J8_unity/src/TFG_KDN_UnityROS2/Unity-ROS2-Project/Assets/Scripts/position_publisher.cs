using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class PositionPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "robot_pose";
    public GameObject cube;

    // Frecuencia de publicación
    public float publishFrequency = 0.5f;
    private float timeElapsed;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PoseStampedMsg>(topicName);
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed >= publishFrequency)
        {
            PoseStampedMsg poseMsg = new PoseStampedMsg();
            poseMsg.pose.position = new PointMsg(
                transform.position.x,
                transform.position.y,
                transform.position.z
            );

            poseMsg.pose.orientation = new QuaternionMsg(
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            );

            ros.Publish(topicName, poseMsg);
            timeElapsed = 0;
        }
    }
}

