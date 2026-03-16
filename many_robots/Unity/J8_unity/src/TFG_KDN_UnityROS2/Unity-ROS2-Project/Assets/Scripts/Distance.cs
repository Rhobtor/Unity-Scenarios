using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using static Unity.Mathematics.math;

public class Distance : MonoBehaviour
{
    private Vector3 lastPosition;
    private Vector3 currentPosition;
    private float totalDistance;

    public TMP_Text travelledDistance;


    private void Start()
    {
        lastPosition = transform.position;
        
    }

    private void Update()
    {

        
        currentPosition = transform.position;
        float distance = Vector3.Distance(lastPosition, currentPosition);
        totalDistance = Mathf.Round(distance*100f)/100f; //Distancia redondeada a 2 decimales. 
       
        travelledDistance.SetText(totalDistance.ToString());
       // Debug.Log(transform.position);


    }

    private void OnDestroy()
    {
       // Debug.Log("Total distance travelled:" + totalDistance);
    }

    

}
