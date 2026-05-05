using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using NWH.WheelController3D;
using System.Text;

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

        [Tooltip("Brake torque applied while coasting to reduce sliding.")]
        public float coastBrakeTorque = 120f;

        [Tooltip("Distance between left and right wheel tracks (meters)")]
        public float trackWidth = 1.40f;

        [Tooltip("Expected maximum forward speed command from ROS (m/s)")]
        public float maxLinearCommand = 2.0f;

        [Tooltip("Expected maximum yaw rate command from ROS (rad/s)")]
        public float maxAngularCommand = 1.5f;

        [Tooltip("How quickly wheel torque can change. Lower values feel smoother.")]
        public float torqueChangeRate = 1200f;

        [Tooltip("Converts side speed error in m/s to wheel torque in N·m.")]
        public float speedErrorToTorque = 450f;

        [Tooltip("Extra brake when vehicle side speed is above target speed.")]
        public float overspeedBrakeTorque = 220f;

        [Tooltip("Invert torque direction for left-side wheels if they are mirrored.")]
        public bool invertLeftSide = false;

        [Tooltip("Invert torque direction for right-side wheels if they are mirrored.")]
        public bool invertRightSide = true;

        [Header("Diagnostics")]
        public bool logWheelDiagnostics = false;
        public float diagnosticsInterval = 1.0f;

        private ROSConnection ros;
        private float rosLinear = 0f;
        private float rosAngular = 0f;
        private float lastCmdReceived = 0f;
        private float currentLeftTorque = 0f;
        private float currentRightTorque = 0f;
        private float lastDiagnosticsTime = 0f;
        private Rigidbody targetRigidbody;

        void Start()
        {
            targetRigidbody = GetComponent<Rigidbody>();
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

            // Differential side-speed targets in m/s. Positive angular.z is left turn in ROS.
            float targetLeftSpeed = rosLinear - rosAngular * trackWidth * 0.5f;
            float targetRightSpeed = rosLinear + rosAngular * trackWidth * 0.5f;

            Vector3 localVelocity = transform.InverseTransformDirection(targetRigidbody.linearVelocity);
            Vector3 localAngularVelocity = transform.InverseTransformDirection(targetRigidbody.angularVelocity);
            float currentForwardSpeed = localVelocity.z;
            float currentYawRate = localAngularVelocity.y;
            float currentLeftSpeed = currentForwardSpeed - currentYawRate * trackWidth * 0.5f;
            float currentRightSpeed = currentForwardSpeed + currentYawRate * trackWidth * 0.5f;

            float leftSpeedError = targetLeftSpeed - currentLeftSpeed;
            float rightSpeedError = targetRightSpeed - currentRightSpeed;

            float leftInput = Mathf.Clamp(targetLeftSpeed / Mathf.Max(0.01f, maxLinearCommand), -1f, 1f);
            float rightInput = Mathf.Clamp(targetRightSpeed / Mathf.Max(0.01f, maxLinearCommand), -1f, 1f);

            float targetLeftTorque = Mathf.Clamp(leftSpeedError * speedErrorToTorque, -maxMotorTorque, maxMotorTorque);
            float targetRightTorque = Mathf.Clamp(rightSpeedError * speedErrorToTorque, -maxMotorTorque, maxMotorTorque);
            targetLeftTorque *= invertLeftSide ? -1f : 1f;
            targetRightTorque *= invertRightSide ? -1f : 1f;

            currentLeftTorque = Mathf.MoveTowards(currentLeftTorque, targetLeftTorque, torqueChangeRate * Time.fixedDeltaTime);
            currentRightTorque = Mathf.MoveTowards(currentRightTorque, targetRightTorque, torqueChangeRate * Time.fixedDeltaTime);

            float leftBrake = CalculateBrakeTorque(targetLeftSpeed, currentLeftSpeed, leftInput);
            float rightBrake = CalculateBrakeTorque(targetRightSpeed, currentRightSpeed, rightInput);

            SetWheelTorque(wheelA1, currentLeftTorque,  leftBrake);
            SetWheelTorque(wheelA2, currentLeftTorque,  leftBrake);
            SetWheelTorque(wheelA3, currentLeftTorque,  leftBrake);
            SetWheelTorque(wheelA4, currentLeftTorque,  leftBrake);

            SetWheelTorque(wheelB1, currentRightTorque, rightBrake);
            SetWheelTorque(wheelB2, currentRightTorque, rightBrake);
            SetWheelTorque(wheelB3, currentRightTorque, rightBrake);
            SetWheelTorque(wheelB4, currentRightTorque, rightBrake);

            if (logWheelDiagnostics && Time.time - lastDiagnosticsTime >= diagnosticsInterval)
            {
                lastDiagnosticsTime = Time.time;
                Debug.Log(BuildDiagnosticsReport());
            }
        }

        private void SetWheelTorque(WheelController wc, float motorTorque, float brake)
        {
            if (wc == null) return;
            wc.MotorTorque = motorTorque;
            wc.BrakeTorque = brake;
        }

        private float CalculateBrakeTorque(float targetSideSpeed, float currentSideSpeed, float normalizedInput)
        {
            if (Mathf.Abs(normalizedInput) < 0.01f)
            {
                return brakeTorque;
            }

            if (Mathf.Abs(targetSideSpeed) < 0.05f)
            {
                return coastBrakeTorque;
            }

            if (Mathf.Abs(currentSideSpeed) > Mathf.Abs(targetSideSpeed) + 0.1f)
            {
                return overspeedBrakeTorque;
            }

            return 0f;
        }

        private string BuildDiagnosticsReport()
        {
            StringBuilder builder = new StringBuilder(512);
            builder.AppendLine("ArgoJ8 wheel diagnostics:");
            builder.Append("cmd linear=").Append(rosLinear.ToString("F3"))
                .Append(" angular=").Append(rosAngular.ToString("F3"))
                .Append(" leftTorque=").Append(currentLeftTorque.ToString("F1"))
                .Append(" rightTorque=").Append(currentRightTorque.ToString("F1"))
                .Append(" secondsSinceCmd=").Append((Time.time - lastCmdReceived).ToString("F2"))
                .AppendLine();
            AppendWheelDiagnostics(builder, "A1", wheelA1);
            AppendWheelDiagnostics(builder, "A2", wheelA2);
            AppendWheelDiagnostics(builder, "A3", wheelA3);
            AppendWheelDiagnostics(builder, "A4", wheelA4);
            AppendWheelDiagnostics(builder, "B1", wheelB1);
            AppendWheelDiagnostics(builder, "B2", wheelB2);
            AppendWheelDiagnostics(builder, "B3", wheelB3);
            AppendWheelDiagnostics(builder, "B4", wheelB4);
            return builder.ToString();
        }

        private void AppendWheelDiagnostics(StringBuilder builder, string label, WheelController wheel)
        {
            if (wheel == null)
            {
                builder.AppendLine(label + ": null");
                return;
            }

            builder.Append(label)
                .Append(" grounded=").Append(wheel.IsGrounded)
                .Append(" load=").Append(wheel.Load.ToString("F1"))
                .Append(" rpm=").Append(wheel.RPM.ToString("F1"))
                .Append(" torque=").Append(wheel.MotorTorque.ToString("F1"))
                .Append(" counter=").Append(wheel.CounterTorque.ToString("F1"))
                .Append(" longSlip=").Append(wheel.LongitudinalSlip.ToString("F2"))
                .Append(" latSlip=").Append(wheel.LateralSlip.ToString("F2"))
                .AppendLine();
        }
    }
}
