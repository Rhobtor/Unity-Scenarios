using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using NWH.WheelController3D;

namespace RosSharp.Control
{
    /// <summary>
    /// Skid-steer controller for Argo J8 (8 wheels) using NWH WheelController3D.
    /// Requires a Rigidbody on this GameObject (NOT ArticulationBody).
    /// Subscribes to /cmd_vel (geometry_msgs/Twist) and converts to differential torque.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class ArgoJ8_NWH_Controller : MonoBehaviour
    {
        [Header("ROS")]
        public string topicName = "/cmd_vel";
        public float ROSTimeout = 0.5f;

        [Header("Left Wheels (A-side)")]
        public WheelController wheelA1; // front left
        public WheelController wheelA2; // mid-front left
        public WheelController wheelA3; // mid-back left
        public WheelController wheelA4; // back left

        [Header("Right Wheels (B-side)")]
        public WheelController wheelB1; // front right
        public WheelController wheelB2; // mid-front right
        public WheelController wheelB3; // mid-back right
        public WheelController wheelB4; // back right

        [Header("Drive Parameters")]
        [Tooltip("Maximum motor torque applied to each wheel (N·m)")]
        public float maxMotorTorque = 500f;

        [Tooltip("Torque applied when braking to stop")]
        public float brakeTorque = 1000f;

        [Tooltip("Distance between left and right wheel tracks (meters)")]
        public float trackWidth = 1.40f;

        [Tooltip("Expected maximum forward speed command from ROS (m/s)")]
        public float maxLinearCommand = 2.0f;

        [Tooltip("Expected maximum yaw rate command from ROS (rad/s)")]
        public float maxAngularCommand = 1.5f;

        [Tooltip("How quickly wheel torque can change. Lower values feel smoother.")]
        public float torqueChangeRate = 1200f;

        [Tooltip("Invert torque direction for left-side wheels if they are mirrored.")]
        public bool invertLeftSide = false;

        [Tooltip("Invert torque direction for right-side wheels if they are mirrored.")]
        public bool invertRightSide = true;

        private ROSConnection ros;
        private float rosLinear = 0f;
        private float rosAngular = 0f;
        private float lastCmdReceived = 0f;
        private float currentLeftTorque = 0f;
        private float currentRightTorque = 0f;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            ros.Subscribe<TwistMsg>(topicName, ReceiveROSCmd);
        }

        void ReceiveROSCmd(TwistMsg cmdVel)
        {
            rosLinear  = (float)cmdVel.linear.x;
            rosAngular = (float)cmdVel.angular.z;
            lastCmdReceived = Time.time;
        }

        void FixedUpdate()
        {
            // Timeout: stop if no command received
            if (Time.time - lastCmdReceived > ROSTimeout)
            {
                rosLinear  = 0f;
                rosAngular = 0f;
            }

            float normalizedLinear = Mathf.Clamp(rosLinear / Mathf.Max(0.01f, maxLinearCommand), -1f, 1f);
            float normalizedAngular = Mathf.Clamp(rosAngular / Mathf.Max(0.01f, maxAngularCommand), -1f, 1f);

            // Skid-steer differential: angular.z > 0 = turn left in ROS convention.
            float leftInput = Mathf.Clamp(normalizedLinear + normalizedAngular, -1f, 1f);
            float rightInput = Mathf.Clamp(normalizedLinear - normalizedAngular, -1f, 1f);

            float targetLeftTorque = leftInput * maxMotorTorque * (invertLeftSide ? -1f : 1f);
            float targetRightTorque = rightInput * maxMotorTorque * (invertRightSide ? -1f : 1f);

            currentLeftTorque = Mathf.MoveTowards(currentLeftTorque, targetLeftTorque, torqueChangeRate * Time.fixedDeltaTime);
            currentRightTorque = Mathf.MoveTowards(currentRightTorque, targetRightTorque, torqueChangeRate * Time.fixedDeltaTime);

            // Apply brake when input is zero to prevent rolling
            float leftBrake  = (Mathf.Abs(leftInput)  < 0.01f) ? brakeTorque : 0f;
            float rightBrake = (Mathf.Abs(rightInput) < 0.01f) ? brakeTorque : 0f;

            SetWheelTorque(wheelA1, currentLeftTorque,  leftBrake);
            SetWheelTorque(wheelA2, currentLeftTorque,  leftBrake);
            SetWheelTorque(wheelA3, currentLeftTorque,  leftBrake);
            SetWheelTorque(wheelA4, currentLeftTorque,  leftBrake);

            SetWheelTorque(wheelB1, currentRightTorque, rightBrake);
            SetWheelTorque(wheelB2, currentRightTorque, rightBrake);
            SetWheelTorque(wheelB3, currentRightTorque, rightBrake);
            SetWheelTorque(wheelB4, currentRightTorque, rightBrake);
        }

        private void SetWheelTorque(WheelController wc, float motorTorque, float brake)
        {
            if (wc == null) return;
            wc.MotorTorque = motorTorque;
            wc.BrakeTorque = brake;
        }
    }
}
