/******************************************************
 * Author: Óscar Almenara Reyes
 * Bachelor's Degree in Industrial Electronics Engineering
 * University of Málaga
 * Final Degree Project: "Towards digital twins in emergency robotics: 
    representation of real-world data in a virtual environment using Unity and ROS 2."
 * Year: 2025
 ******************************************************/


using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraSwitch : MonoBehaviour
{
    public GameObject Camera1; 
    public GameObject Camera2; 
    public GameObject Camera3; 
    public GameObject Camera4; 
    public GameObject Camera5; 
    public GameObject Camera6; 
    public GameObject Camera7; 

    private int count = 0;

    private List<GameObject> GetAssignedCameras()
    {
        List<GameObject> cameras = new List<GameObject>(7);

        if (Camera1 != null) cameras.Add(Camera1);
        if (Camera2 != null) cameras.Add(Camera2);
        if (Camera3 != null) cameras.Add(Camera3);
        if (Camera4 != null) cameras.Add(Camera4);
        if (Camera5 != null) cameras.Add(Camera5);
        if (Camera6 != null) cameras.Add(Camera6);
        if (Camera7 != null) cameras.Add(Camera7);

        return cameras;
    }

    void Start()
    {
        SetActiveCamera(0);
    }

    void Update()
    {
        List<GameObject> cameras = GetAssignedCameras();
        if (cameras.Count == 0)
        {
            return;
        }

        if (Input.GetKeyDown(KeyCode.C))
        {
            count = (count + 1) % cameras.Count;
            SetActiveCamera(count);
            Debug.Log("C�mara activa: " + (count + 1));
        }
    }

    void SetActiveCamera(int index)
    {
        List<GameObject> cameras = GetAssignedCameras();
        if (cameras.Count == 0)
        {
            return;
        }

        count = Mathf.Clamp(index, 0, cameras.Count - 1);

        for (int i = 0; i < cameras.Count; i++)
        {
            cameras[i].SetActive(i == count);
        }
    }
}
