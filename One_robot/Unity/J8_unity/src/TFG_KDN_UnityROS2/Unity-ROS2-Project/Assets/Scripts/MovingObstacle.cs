using UnityEngine;

public class MovingObstacle : MonoBehaviour
{
    public Transform pointA;
    public Transform pointB;
    public float speed = 2.0f;

    private Rigidbody rb;
    private Vector3 target;
    private bool goingToB = true;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        target = pointB.position;
    }

    void FixedUpdate()
    {
        Vector3 newPosition = Vector3.MoveTowards(rb.position, target, speed * Time.fixedDeltaTime);
        rb.MovePosition(newPosition);

        if (Vector3.Distance(rb.position, target) < 0.05f)
        {
            goingToB = !goingToB;
            target = goingToB ? pointB.position : pointA.position;
        }
    }
}