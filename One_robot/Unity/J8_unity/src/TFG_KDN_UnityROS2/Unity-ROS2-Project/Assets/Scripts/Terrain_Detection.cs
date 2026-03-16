using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Terrain_Detection : MonoBehaviour
{
    //The material I want to modify its characteristics 
    public PhysicMaterial physicMaterial;
    public LayerMask layerToIgnore; //To Assign the layer I want to ignore
    RaycastHit hit;
    
    void FixedUpdate()
    {
        //Set the raycast ignoring "map" layer assigned on the inspector by public value
        if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.down), out hit, Mathf.Infinity, ~layerToIgnore))
        {
            //Draw a yellow ray for testing.
            Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.down) * hit.distance, Color.yellow);
            if (hit.transform.CompareTag("Road"))
            {
                physicMaterial.dynamicFriction = 0.3f;
                physicMaterial.staticFriction = 0.3f;
                physicMaterial.bounciness = 0.0f;
                Debug.Log("Did Hit Road. df = " + physicMaterial.dynamicFriction + "sf= " + physicMaterial.staticFriction);
            }
            else if (hit.transform.CompareTag("Pavement"))
            {
                physicMaterial.dynamicFriction = 0.2f;
                physicMaterial.staticFriction = 0.2f;
                physicMaterial.bounciness = 0.0f;
                Debug.Log("Did Hit Pavement. df = " + physicMaterial.dynamicFriction + "sf= " + physicMaterial.staticFriction);
            }
            else if (hit.transform.CompareTag("RiverBed"))
            {
                physicMaterial.dynamicFriction = 0.6f;
                physicMaterial.staticFriction = 0.6f;
                physicMaterial.bounciness = 0.0f;
                Debug.Log("Did Hit RiverBed. df = " + physicMaterial.dynamicFriction + "sf= " + physicMaterial.staticFriction);
            }
            else
            {
                physicMaterial.dynamicFriction = 0.5f;
                physicMaterial.staticFriction = 0.5f;
                physicMaterial.bounciness = 0.0f;
                Debug.Log("Did Hit OffRoad. df = " + physicMaterial.dynamicFriction + "sf= " + physicMaterial.staticFriction);

            }
        }
    }
}



