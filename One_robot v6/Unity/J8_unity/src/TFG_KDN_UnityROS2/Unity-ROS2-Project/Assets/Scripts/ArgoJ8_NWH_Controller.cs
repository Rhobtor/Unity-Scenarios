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

        [Tooltip("Maximum side speed used to normalize skid-steer wheel commands.")]
        public float maxSideSpeedCommand = 2.5f;

        [Tooltip("Commands smaller than this are treated as zero to avoid idle creep from ROS/control noise.")]
        public float commandDeadzone = 0.02f;

        [Tooltip("Side speed errors smaller than this are ignored to avoid residual torque while stopped.")]
        public float speedErrorDeadzone = 0.05f;

        [Tooltip("Vehicle forward speed below this is considered stopped.")]
        public float stopSpeedThreshold = 0.08f;

        [Tooltip("Vehicle yaw rate below this is considered stopped.")]
        public float stopYawRateThreshold = 0.08f;

        [Tooltip("Force Rigidbody velocities to zero and put it to sleep when there is no command and the robot is almost stopped.")]
        public bool hardStopAtIdle = true;

        [Tooltip("Linear deceleration applied directly to the Rigidbody when there is no command.")]
        public float idleLinearDamping = 6f;

        [Tooltip("Sideways damping applied in local X to suppress lateral drift from wheel slip.")]
        public float lateralDamping = 12f;

        [Tooltip("Yaw damping applied directly to the Rigidbody when there is no command.")]
        public float idleAngularDamping = 10f;

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

            if (Mathf.Abs(normalizedLinear) < commandDeadzone)
            {
                normalizedLinear = 0f;
                rosLinear = 0f;
            }

            if (Mathf.Abs(normalizedAngular) < commandDeadzone)
            {
                normalizedAngular = 0f;
                rosAngular = 0f;
            }

            // Differential side-speed targets in m/s. Positive angular.z is left turn in ROS.
            float targetLeftSpeed = rosLinear - rosAngular * trackWidth * 0.5f;
            float targetRightSpeed = rosLinear + rosAngular * trackWidth * 0.5f;

            Vector3 localVelocity = transform.InverseTransformDirection(targetRigidbody.linearVelocity);
            Vector3 localAngularVelocity = transform.InverseTransformDirection(targetRigidbody.angularVelocity);
            float currentForwardSpeed = localVelocity.z;
            float currentYawRate = localAngularVelocity.y;
            float currentLeftSpeed = currentForwardSpeed - currentYawRate * trackWidth * 0.5f;
            float currentRightSpeed = currentForwardSpeed + currentYawRate * trackWidth * 0.5f;

            bool isStopCommand = normalizedLinear == 0f && normalizedAngular == 0f;
            bool hasNoLinearCommand = normalizedLinear == 0f;
            bool hasNoAngularCommand = normalizedAngular == 0f;
            bool isNearlyStopped = Mathf.Abs(currentForwardSpeed) < stopSpeedThreshold
                && Mathf.Abs(currentYawRate) < stopYawRateThreshold;
            float leftSpeedError = targetLeftSpeed - currentLeftSpeed;
            float rightSpeedError = targetRightSpeed - currentRightSpeed;
            float leftInput = Mathf.Clamp(targetLeftSpeed / Mathf.Max(0.01f, maxSideSpeedCommand), -1f, 1f);
            float rightInput = Mathf.Clamp(targetRightSpeed / Mathf.Max(0.01f, maxSideSpeedCommand), -1f, 1f);
            float targetLeftTorque;
            float targetRightTorque;
            float appliedLeftTorque;
            float appliedRightTorque;

            if (isStopCommand)
            {
                targetLeftTorque = 0f;
                targetRightTorque = 0f;
                currentLeftTorque = 0f;
                currentRightTorque = 0f;
            }
            else
            {
                if (Mathf.Abs(leftSpeedError) < speedErrorDeadzone)
                {
                    leftSpeedError = 0f;
                }

                if (Mathf.Abs(rightSpeedError) < speedErrorDeadzone)
                {
                    rightSpeedError = 0f;
                }

                float feedForwardLeftTorque = leftInput * maxMotorTorque;
                float feedForwardRightTorque = rightInput * maxMotorTorque;
                float correctionLeftTorque = Mathf.Clamp(leftSpeedError * speedErrorToTorque, -maxMotorTorque, maxMotorTorque);
                float correctionRightTorque = Mathf.Clamp(rightSpeedError * speedErrorToTorque, -maxMotorTorque, maxMotorTorque);

                targetLeftTorque = Mathf.Clamp(feedForwardLeftTorque + correctionLeftTorque * 0.35f, -maxMotorTorque, maxMotorTorque);
                targetRightTorque = Mathf.Clamp(feedForwardRightTorque + correctionRightTorque * 0.35f, -maxMotorTorque, maxMotorTorque);

                currentLeftTorque = Mathf.MoveTowards(currentLeftTorque, targetLeftTorque, torqueChangeRate * Time.fixedDeltaTime);
                currentRightTorque = Mathf.MoveTowards(currentRightTorque, targetRightTorque, torqueChangeRate * Time.fixedDeltaTime);
            }

            if (isStopCommand)
            {
                appliedLeftTorque = 0f;
                appliedRightTorque = 0f;
            }
            else
            {
                appliedLeftTorque = currentLeftTorque * (invertLeftSide ? -1f : 1f);
                appliedRightTorque = currentRightTorque * (invertRightSide ? -1f : 1f);
            }

            float leftBrake = CalculateBrakeTorque(targetLeftSpeed, currentLeftSpeed, leftInput);
            float rightBrake = CalculateBrakeTorque(targetRightSpeed, currentRightSpeed, rightInput);

            SetWheelTorque(wheelA1, appliedLeftTorque,  leftBrake);
            SetWheelTorque(wheelA2, appliedLeftTorque,  leftBrake);
            SetWheelTorque(wheelA3, appliedLeftTorque,  leftBrake);
            SetWheelTorque(wheelA4, appliedLeftTorque,  leftBrake);

            SetWheelTorque(wheelB1, appliedRightTorque, rightBrake);
            SetWheelTorque(wheelB2, appliedRightTorque, rightBrake);
            SetWheelTorque(wheelB3, appliedRightTorque, rightBrake);
            SetWheelTorque(wheelB4, appliedRightTorque, rightBrake);

            ApplyCommandAxisConstraints(hasNoLinearCommand, hasNoAngularCommand);

            if (hardStopAtIdle && isStopCommand && isNearlyStopped)
            {
                targetRigidbody.linearVelocity = Vector3.zero;
                targetRigidbody.angularVelocity = Vector3.zero;
                targetRigidbody.Sleep();
            }
            else if (!isStopCommand)
            {
                targetRigidbody.WakeUp();
            }

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

        private void ApplyCommandAxisConstraints(bool hasNoLinearCommand, bool hasNoAngularCommand)
        {
            Vector3 constrainedLocalVelocity = transform.InverseTransformDirection(targetRigidbody.linearVelocity);
            constrainedLocalVelocity.x = Mathf.MoveTowards(constrainedLocalVelocity.x, 0f, lateralDamping * Time.fixedDeltaTime);

            if (hasNoLinearCommand)
            {
                constrainedLocalVelocity.z = Mathf.MoveTowards(constrainedLocalVelocity.z, 0f, idleLinearDamping * Time.fixedDeltaTime);
                if (Mathf.Abs(constrainedLocalVelocity.z) < stopSpeedThreshold)
                {
                    constrainedLocalVelocity.z = 0f;
                }
            }

            if (Mathf.Abs(constrainedLocalVelocity.x) < speedErrorDeadzone)
            {
                constrainedLocalVelocity.x = 0f;
            }

            targetRigidbody.linearVelocity = transform.TransformDirection(constrainedLocalVelocity);

            Vector3 constrainedLocalAngularVelocity = transform.InverseTransformDirection(targetRigidbody.angularVelocity);
            if (hasNoAngularCommand)
            {
                constrainedLocalAngularVelocity.y = Mathf.MoveTowards(constrainedLocalAngularVelocity.y, 0f, idleAngularDamping * Time.fixedDeltaTime);
                if (Mathf.Abs(constrainedLocalAngularVelocity.y) < stopYawRateThreshold)
                {
                    constrainedLocalAngularVelocity.y = 0f;
                }
            }

            targetRigidbody.angularVelocity = transform.TransformDirection(constrainedLocalAngularVelocity);
        }

        private float CalculateBrakeTorque(float targetSideSpeed, float currentSideSpeed, float normalizedInput)
        {
            if (Mathf.Abs(targetSideSpeed) < speedErrorDeadzone && Mathf.Abs(currentSideSpeed) < stopSpeedThreshold)
            {
                return brakeTorque;
            }

            if (Mathf.Abs(normalizedInput) < 0.01f)
            {
                return brakeTorque;
            }

            if (Mathf.Sign(targetSideSpeed) != Mathf.Sign(currentSideSpeed) && Mathf.Abs(currentSideSpeed) > stopSpeedThreshold)
            {
                return overspeedBrakeTorque;
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
