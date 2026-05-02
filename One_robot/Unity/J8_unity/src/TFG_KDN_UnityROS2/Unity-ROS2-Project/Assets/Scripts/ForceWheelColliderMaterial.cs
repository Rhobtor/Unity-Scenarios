using UnityEngine;

public class ForceWheelColliderMaterial : MonoBehaviour
{
    [Header("Material físico que hace que la rueda agarre")]
    public PhysicMaterial wheelPhysicMaterial;

    [Header("Buscar también objetos desactivados")]
    public bool includeInactive = true;

    void Start()
    {
        ApplyMaterial();
    }

    void LateUpdate()
    {
        // Por si el plugin crea los colliders después del Start()
        ApplyMaterial();
    }

    void ApplyMaterial()
    {
        if (wheelPhysicMaterial == null)
            return;

        Collider[] colliders = GetComponentsInChildren<Collider>(includeInactive);

        foreach (Collider col in colliders)
        {
            bool isWheelCollider =
                col.name.ToLower().Contains("collider") ||
                col.transform.name.ToLower().Contains("wheelcollider") ||
                col.transform.parent != null &&
                col.transform.parent.name.ToLower().Contains("wheelcollider");

            if (isWheelCollider)
            {
                col.material = wheelPhysicMaterial;
            }
        }
    }
}