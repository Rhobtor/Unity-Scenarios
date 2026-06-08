using UnityEngine;

public class RandomMovingObstacle : MonoBehaviour
{
    public Transform pointA;
    public Transform pointB;

    [Header("Movimiento")]
    public float minSpeed = 0.8f;
    public float maxSpeed = 2.0f;
    public float reachDistance = 0.08f;

    [Header("Comportamiento aleatorio")]
    public float minWaitTime = 0.2f;
    public float maxWaitTime = 1.0f;

    [Range(0f, 1f)]
    public float minTargetPercent = 0.05f;

    [Range(0f, 1f)]
    public float maxTargetPercent = 0.95f;

    private Rigidbody rb;
    private Vector3 target;
    private float currentSpeed;
    private float waitTimer = 0f;

    void Start()
    {
        rb = GetComponent<Rigidbody>();

        if (rb == null)
        {
            Debug.LogError("Este objeto necesita un Rigidbody.");
            enabled = false;
            return;
        }

        if (pointA == null || pointB == null)
        {
            Debug.LogError("Faltan PointA o PointB en RandomMovingObstacle.");
            enabled = false;
            return;
        }

        PickNewRandomTarget();
    }

    void FixedUpdate()
    {
        if (waitTimer > 0f)
        {
            waitTimer -= Time.fixedDeltaTime;
            return;
        }

        Vector3 newPosition = Vector3.MoveTowards(
            rb.position,
            target,
            currentSpeed * Time.fixedDeltaTime
        );

        rb.MovePosition(newPosition);

        if (Vector3.Distance(rb.position, target) < reachDistance)
        {
            waitTimer = Random.Range(minWaitTime, maxWaitTime);
            PickNewRandomTarget();
        }
    }

    void PickNewRandomTarget()
    {
        float t = Random.Range(minTargetPercent, maxTargetPercent);

        target = Vector3.Lerp(pointA.position, pointB.position, t);

        currentSpeed = Random.Range(minSpeed, maxSpeed);

        Debug.Log("Nuevo objetivo: " + target + " | porcentaje: " + t);
    }

    void OnDrawGizmos()
    {
        if (pointA == null || pointB == null)
            return;

        Gizmos.color = Color.yellow;
        Gizmos.DrawLine(pointA.position, pointB.position);

        Gizmos.color = Color.green;
        Gizmos.DrawSphere(pointA.position, 0.25f);

        Gizmos.color = Color.red;
        Gizmos.DrawSphere(pointB.position, 0.25f);

        if (Application.isPlaying)
        {
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(target, 0.35f);
        }
    }
}