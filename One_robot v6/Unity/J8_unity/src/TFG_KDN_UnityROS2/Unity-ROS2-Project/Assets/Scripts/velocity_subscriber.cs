using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocitySubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "cmd_vel";  // Nombre del tópico para recibir comandos de velocidad

    // El GameObject que queremos mover (por ejemplo, el cubo)
    public GameObject cube;

    // Factores para ajustar la velocidad del movimiento y la rotación
    public float moveSpeed = 1.0f;
    public float rotateSpeed = 0.0f;

    private Vector3 linearVelocity;
    private Vector3 angularVelocity;

    void Start()
    {
        // Obtener la instancia de la conexión ROS
        ros = ROSConnection.GetOrCreateInstance();
        // Suscribirse al tópico cmd_vel
        ros.Subscribe<TwistMsg>(topicName, MoveCube);
    }

    void MoveCube(TwistMsg msg)
    {
        // Guardar las velocidades lineales y angulares recibidas
        linearVelocity = new Vector3((float)msg.linear.x, (float)msg.linear.y, (float)msg.linear.z);
        angularVelocity = new Vector3((float)msg.angular.x, (float)msg.angular.y, (float)msg.angular.z);
    }
    void Update()
    {
        // Aplicar el movimiento y la rotación en función de las velocidades recibidas
        if (cube != null)
        {
            cube.transform.Translate(linearVelocity * moveSpeed * Time.deltaTime);
            cube.transform.Rotate(angularVelocity * rotateSpeed * Time.deltaTime);
        }
    }
}

