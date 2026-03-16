using UnityEngine;
using RosMessageTypes.Nav;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using System;

public class OdometryPublisher : MonoBehaviour
{
    ROSConnection ros;
    public GameObject fixpositionObject; // Objeto que representa el Fixposition
    public Vector3 lastPosition;        // Almacena la última posición del objeto
    public Quaternion lastRotation;     // Almacena la última orientación del objeto
    public float publishFrequency = 10.0f; // Frecuencia de publicación en Hz
    private float timeElapsed;

    void Start()
    {
        // Inicializar ROSConnection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<OdometryMsg>("/fixposition/odometry");

        // Inicializar variables
        lastPosition = fixpositionObject.transform.position;
        lastRotation = fixpositionObject.transform.rotation;
    }

    void Update()
    {
        timeElapsed += Time.deltaTime;
        if (timeElapsed > 1.0f / publishFrequency)
        {
            PublishOdometryData();
            timeElapsed = 0;
        }
    }

    void PublishOdometryData()
    {
        // Obtener posición y orientación actual
        Vector3 currentPosition = fixpositionObject.transform.position;
        Quaternion currentRotation = fixpositionObject.transform.rotation;

        // Calcular velocidades lineal y angular
        //Vector3 velocity = (currentPosition - lastPosition) / Time.deltaTime;
        //Vector3 angularVelocity = (currentRotation.eulerAngles - lastRotation.eulerAngles) / Time.deltaTime;

        // Crear mensaje de odometría
        OdometryMsg odometryMessage = new OdometryMsg
        {
            header = new HeaderMsg
            {
                stamp = new RosMessageTypes.BuiltinInterfaces.TimeMsg
                {
                    sec = (int)DateTimeOffset.UtcNow.ToUnixTimeSeconds(),
                    nanosec = (uint)(DateTimeOffset.UtcNow.ToUnixTimeMilliseconds() % 1000 * 1000000)
                },
                frame_id = "fix_position"
            },
            child_frame_id = "ODOM",
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
                covariance = new double[36] // Matriz de covarianza (dejar como ceros por simplicidad)
            }
           /* twist = new TwistWithCovarianceMsg
            {
                twist = new TwistMsg
                {
                    linear = new Vector3Msg
                    {
                        x = velocity.x,
                        y = velocity.y,
                        z = velocity.z
                    },
                    angular = new Vector3Msg
                    {
                        x = angularVelocity.x,
                        y = angularVelocity.y,
                        z = angularVelocity.z
                    }
                },
                covariance = new double[36] // Matriz de covarianza (dejar como ceros por simplicidad)
            }*/
        };

        // Publicar el mensaje
        ros.Publish("/fixposition/odometry", odometryMessage);

        // Actualizar posici�n y rotaci�n anteriores
        lastPosition = currentPosition;
        lastRotation = currentRotation;

        Debug.Log("Publicado odometr�a.");
    }
}

