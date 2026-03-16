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

    void Start()
    {
        SetActiveCamera(0);
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.C))
        {
            count = (count + 1) % 7; // 0 a 6
            SetActiveCamera(count);
            Debug.Log("C�mara activa: " + (count + 1));
        }
    }

    void SetActiveCamera(int index)
    {
        Camera1.SetActive(index == 0);
        Camera2.SetActive(index == 1);
        Camera3.SetActive(index == 2);
        Camera4.SetActive(index == 3);
        Camera5.SetActive(index == 4);
        Camera6.SetActive(index == 5);
        Camera7.SetActive(index == 6);
    }
}
