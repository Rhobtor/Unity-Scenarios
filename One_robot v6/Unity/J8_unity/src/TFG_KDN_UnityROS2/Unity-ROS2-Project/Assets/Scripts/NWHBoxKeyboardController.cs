using NWH.WheelController3D;
using UnityEngine;

namespace RosSharp.Control
{
    [RequireComponent(typeof(Rigidbody))]
    public class NWHBoxKeyboardController : MonoBehaviour
    {
        [Header("Wheels")]
        public WheelController frontLeft;
        public WheelController frontRight;
        public WheelController rearLeft;
        public WheelController rearRight;

        [Header("Drive")]
        public float maxMotorTorque = 350f;
        public float maxBrakeTorque = 1200f;
        public float idleBrakeTorque = 300f;
        public float maxSteerAngle = 28f;
        public bool invertLeftSide = false;
        public bool invertRightSide = false;

        private void FixedUpdate()
        {
            float throttle = Input.GetAxisRaw("Vertical");
            float steering = Input.GetAxisRaw("Horizontal");
            bool braking = Input.GetKey(KeyCode.Space);

            float steerAngle = steering * maxSteerAngle;
            float leftTorque = throttle * maxMotorTorque * (invertLeftSide ? -1f : 1f);
            float rightTorque = throttle * maxMotorTorque * (invertRightSide ? -1f : 1f);
            float brakeTorque = braking
                ? maxBrakeTorque
                : Mathf.Abs(throttle) < 0.01f
                    ? idleBrakeTorque
                    : 0f;

            ApplyDrive(frontLeft, leftTorque, brakeTorque, steerAngle);
            ApplyDrive(frontRight, rightTorque, brakeTorque, steerAngle);
            ApplyDrive(rearLeft, leftTorque, brakeTorque, 0f);
            ApplyDrive(rearRight, rightTorque, brakeTorque, 0f);
        }

        private static void ApplyDrive(WheelController wheel, float motorTorque, float brakeTorque, float steerAngle)
        {
            if (wheel == null)
            {
                return;
            }

            wheel.MotorTorque = motorTorque;
            wheel.BrakeTorque = brakeTorque;
            wheel.SteerAngle = steerAngle;
        }
    }
}