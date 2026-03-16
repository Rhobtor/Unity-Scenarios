using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class VelocityPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "cmd_vel"; // Nombre del tópico para enviar los comandos de velocidad

    // El GameObject que queremos mover (por ejemplo, el cubo)
    public GameObject cube;

    // Frecuencia de publicación
    public float publishMessageFrequency = 0.5f;

    // Control de tiempo para la publicación
    private float timeElapsed;

    void Start()
    {
        // Obtener la instancia de la conexión ROS
        ros = ROSConnection.GetOrCreateInstance();
        // Registrar el tipo de mensaje Twist para el tópico
        ros.RegisterPublisher<TwistMsg>(topicName);
    }

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            // Crear un nuevo mensaje Twist
            TwistMsg velocityMsg = new TwistMsg();

            // Asignar la velocidad del cubo (este es un ejemplo, puedes adaptar la lógica a tu necesidad)
            velocityMsg.linear.x = cube.transform.position.x;
            velocityMsg.linear.y = cube.transform.position.y;
            velocityMsg.linear.z = cube.transform.position.z;

            velocityMsg.angular.x = cube.transform.rotation.eulerAngles.x;
            velocityMsg.angular.y = cube.transform.rotation.eulerAngles.y;
            velocityMsg.angular.z = cube.transform.rotation.eulerAngles.z;

            // Publicar el mensaje en el tópico cmd_vel
            ros.Publish(topicName, velocityMsg);

            // Reiniciar el contador de tiempo
            timeElapsed = 0;
        }
    }
}