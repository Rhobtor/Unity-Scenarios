using System.Collections.Generic;
using NWH.WheelController3D;
using UnityEngine;

namespace RosSharp.Control
{
    [ExecuteAlways]
    public class WheelSetupHelper : MonoBehaviour
    {
        [System.Serializable]
        public class WheelPair
        {
            public WheelController left;
            public WheelController right;
        }

        [Header("References")]
        public Rigidbody targetRigidbody;
        public BoxCollider chassisBoxCollider;
        public List<WheelPair> wheelPairs = new List<WheelPair>();

        [Header("Mirroring")]
        [Tooltip("Vehicle symmetry plane in local space. Normally 0 for centered vehicles.")]
        public float localCenterX = 0f;

        [Tooltip("Additional center of mass offset after snapping to the chassis collider center.")]
        public Vector3 centerOfMassOffset = new Vector3(0f, -0.15f, 0f);

        [ContextMenu("Mirror Left To Right")]
        public void MirrorLeftToRight()
        {
            foreach (WheelPair pair in wheelPairs)
            {
                if (pair == null || pair.left == null || pair.right == null)
                {
                    continue;
                }

                Transform leftTransform = pair.left.transform;
                Transform rightTransform = pair.right.transform;

                Vector3 leftLocalPosition = leftTransform.localPosition;
                rightTransform.localPosition = new Vector3(
                    (localCenterX * 2f) - leftLocalPosition.x,
                    leftLocalPosition.y,
                    leftLocalPosition.z);

                Vector3 leftLocalEuler = leftTransform.localEulerAngles;
                rightTransform.localRotation = Quaternion.Euler(leftLocalEuler.x, -leftLocalEuler.y, -leftLocalEuler.z);
                rightTransform.localScale = leftTransform.localScale;
            }

            Debug.Log("WheelSetupHelper: mirrored left wheels to right wheels.");
        }

        [ContextMenu("Average Left/Right Heights")]
        public void AveragePairHeights()
        {
            foreach (WheelPair pair in wheelPairs)
            {
                if (pair == null || pair.left == null || pair.right == null)
                {
                    continue;
                }

                Transform leftTransform = pair.left.transform;
                Transform rightTransform = pair.right.transform;
                float averageY = (leftTransform.localPosition.y + rightTransform.localPosition.y) * 0.5f;

                Vector3 leftPosition = leftTransform.localPosition;
                Vector3 rightPosition = rightTransform.localPosition;
                leftTransform.localPosition = new Vector3(leftPosition.x, averageY, leftPosition.z);
                rightTransform.localPosition = new Vector3(rightPosition.x, averageY, rightPosition.z);
            }

            Debug.Log("WheelSetupHelper: averaged wheel pair heights.");
        }

        [ContextMenu("Snap Center Of Mass To Chassis")]
        public void SnapCenterOfMassToChassis()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (targetRigidbody == null)
            {
                Debug.LogWarning("WheelSetupHelper: no Rigidbody assigned.");
                return;
            }

            if (chassisBoxCollider == null)
            {
                Debug.LogWarning("WheelSetupHelper: no BoxCollider assigned.");
                return;
            }

            targetRigidbody.centerOfMass = chassisBoxCollider.center + centerOfMassOffset;
            Debug.Log($"WheelSetupHelper: centerOfMass set to {targetRigidbody.centerOfMass}.");
        }

        [ContextMenu("Log Wheel Local Transforms")]
        public void LogWheelLocalTransforms()
        {
            for (int i = 0; i < wheelPairs.Count; i++)
            {
                WheelPair pair = wheelPairs[i];
                if (pair == null)
                {
                    continue;
                }

                LogWheelTransform($"Pair {i + 1} Left", pair.left);
                LogWheelTransform($"Pair {i + 1} Right", pair.right);
            }
        }

        private static void LogWheelTransform(string label, WheelController wheel)
        {
            if (wheel == null)
            {
                Debug.Log(label + ": null");
                return;
            }

            Transform transform = wheel.transform;
            Debug.Log(label +
                      $": pos={transform.localPosition} rot={transform.localEulerAngles} scale={transform.localScale}");
        }
    }
}