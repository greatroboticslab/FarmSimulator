using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GripSegment : MonoBehaviour
{

    public GameObject touching;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }



	private void OnTriggerEnter(Collider other)
    {
        touching = other.gameObject;
    }

	private void OnTriggerExit(Collider other)
    {
        touching = null;
    }

}
