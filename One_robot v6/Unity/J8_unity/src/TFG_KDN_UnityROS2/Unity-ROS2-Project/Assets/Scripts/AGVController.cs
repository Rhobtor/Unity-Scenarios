using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using Unity.Robotics.UrdfImporter.Control;

namespace RosSharp.Control
{

    public class AGVController : MonoBehaviour

    {
        private ROSConnection ros;
        public string topicName = "/cmd_vel";  // Nombre del tópico para recibir comandos de velocidad
        private RotationDirection direction;
        private float rosLinear = 1f;
        private float rosAngular = 1f;

        //Ruedas A - Lado Izquierdo, Ruedas B - Lado Derecho
        public GameObject wheelA1;
        public GameObject wheelB1;
        public GameObject wheelA2;
        public GameObject wheelB2;
        public GameObject wheelA3;
        public GameObject wheelB3;
        public GameObject wheelA4;
        public GameObject wheelB4;

        private ArticulationBody wA1, wB1, wA2, wB2, wA3, wB3, wA4, wB4;

        //public ControlType control = ControlType.PositionControl;
        public float maxLinearSpeed = 500; //m/s;  30km/h = 500m/s
        public float maxRotationalSpeed = 250;//
        public float wheelRadius = 0.28f; //meters
        public float trackWidth = 1.40f; // meters Distance between tyres
        public float damping = 100f;
        public float forceLimit = 100f;
        
        public float ROSTimeout = 0.5f;
        private float lastCmdReceived = 0.1f;

        void Start()
        {
            //Inicializar las articulaciones de las ruedas
            wA1 = wheelA1.GetComponent<ArticulationBody>();
            wB1 = wheelB1.GetComponent<ArticulationBody>();
            wA2 = wheelA2.GetComponent<ArticulationBody>();
            wB2 = wheelB2.GetComponent<ArticulationBody>();
            wA3 = wheelA3.GetComponent<ArticulationBody>();
            wB3 = wheelB3.GetComponent<ArticulationBody>();
            wA4 = wheelA4.GetComponent<ArticulationBody>();
            wB4 = wheelB4.GetComponent<ArticulationBody>();

            //Inicializar los parámetros de las ruedas
            SetParameters(wA1);
            SetParameters(wB1);
            SetParameters(wA2);
            SetParameters(wB2);
            SetParameters(wA3);
            SetParameters(wB3);
            SetParameters(wA4);
            SetParameters(wB4);

            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>("/cmd_vel", ReceiveROSCmd);

        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
            Debug.Log($"Received linear: {rosLinear}, angular: {rosAngular}");

        }

        void FixedUpdate()
        {
            // Verificar timeout de los comandos ROS
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear = 0f;
                rosAngular = 0f;
            }

            // Actualizar la velocidad de las ruedas en base a los valores recibidos
            RobotInput(rosLinear, -rosAngular);
        }

        private void SetParameters(ArticulationBody joint)
        {
            ArticulationDrive drive = joint.xDrive;
            drive.forceLimit = forceLimit;
            drive.damping = damping;
            joint.xDrive = drive;
        }

        private void SetSpeed(ArticulationBody joint, float wheelSpeed = float.NaN)
        {
            ArticulationDrive drive = joint.xDrive;
            if (float.IsNaN(wheelSpeed))
            {
                drive.targetVelocity = ((2 * maxLinearSpeed) / wheelRadius) * Mathf.Rad2Deg * (int)direction;
            }
            else
            {
                drive.targetVelocity = wheelSpeed;
            }
            joint.xDrive = drive;
        }

        private void RobotInput(float linearSpeed, float angularSpeed) // m/s and rad/s
        {

            if (linearSpeed > maxLinearSpeed)
            {
                linearSpeed = maxLinearSpeed;
            }
            if (angularSpeed > maxRotationalSpeed)
            {
                angularSpeed = maxRotationalSpeed;
            }

            // Calcular la velocidad de rotación de las ruedas
            float leftWheelRotation = (linearSpeed / wheelRadius);
            float rightWheelRotation = leftWheelRotation;
            float wheelSpeedDiff = ((angularSpeed * trackWidth) / wheelRadius);
            

            if (angularSpeed != 0)
            {
                rightWheelRotation = (rightWheelRotation + (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                leftWheelRotation = (leftWheelRotation - (wheelSpeedDiff / 1)) * Mathf.Rad2Deg;
                Debug.Log("AngularSpeed != 0. Angular speed =");
                Debug.Log(angularSpeed);
                // Debug.Log(rotSpeed);
                Debug.Log("RightWheelRotation=");
                Debug.Log(rightWheelRotation);
                Debug.Log("LeftWheelRotation=");
                Debug.Log(leftWheelRotation);

            }

            else
            {
                //Debug.Log("AngularSpeed = 0");
                leftWheelRotation *= Mathf.Rad2Deg;
                rightWheelRotation *= Mathf.Rad2Deg;
            }

            // Aplicar la velocidad a todas las ruedas
            SetSpeed(wA1, leftWheelRotation);
            SetSpeed(wA2, leftWheelRotation);
            SetSpeed(wA3, leftWheelRotation);
            SetSpeed(wA4, leftWheelRotation);

            SetSpeed(wB1, rightWheelRotation);
            SetSpeed(wB2, rightWheelRotation);
            SetSpeed(wB3, rightWheelRotation);
            SetSpeed(wB4, rightWheelRotation);

        }
    }
}
