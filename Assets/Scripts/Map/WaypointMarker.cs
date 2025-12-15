using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class WaypointMarker : MonoBehaviour
{

    public GameObject cone;
    public float timeAlive = 0f;
    private Vector3 initialConePos;

    // Start is called before the first frame update
    void Start()
    {
        initialConePos = cone.transform.localPosition;
    }

    // Update is called once per frame
    void Update()
    {
        timeAlive += Time.deltaTime;
        cone.transform.localPosition = initialConePos + new Vector3(0,Mathf.Abs(Mathf.Sin(timeAlive)),0);
    }
}
