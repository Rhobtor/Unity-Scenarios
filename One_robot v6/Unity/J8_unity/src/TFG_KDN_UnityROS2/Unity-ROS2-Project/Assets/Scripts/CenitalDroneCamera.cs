/******************************************************
 * Author: Óscar Almenara Reyes
 * Bachelor's Degree in Industrial Electronics Engineering
 * University of Málaga
 * Final Degree Project: "Towards digital twins in emergency robotics: 
    representation of real-world data in a virtual environment using Unity and ROS 2."
 * Year: 2025
 ******************************************************/

using UnityEngine;

public class CenitalDroneCamera : MonoBehaviour
{
    [Header("Target")]
    public Transform drone;               
    public Vector3 offset = new Vector3(0, 20, 0);  

    [Header("Rotación")]
    public float rotationSpeed = 100f;
    public bool enableRotation = true;

    private Vector3 currentOffset;

    void Start()
    {
        if (drone == null)
        {
            Debug.LogError("CenitalDroneCamera: no se ha asignado ningún objeto.");
            enabled = false;
            return;
        }

        currentOffset = offset;
        UpdateCameraPosition();
    }

    void Update()
    {
        if (enableRotation && Input.GetMouseButton(1)) 
        {
            float horizontal = Input.GetAxis("Mouse X");
            Quaternion rotation = Quaternion.Euler(0, horizontal * rotationSpeed * Time.deltaTime, 0);
            currentOffset = rotation * currentOffset;
        }

        UpdateCameraPosition();
    }

    void UpdateCameraPosition()
    {
        transform.position = drone.position + currentOffset;
        transform.LookAt(drone.position);
    }
}
