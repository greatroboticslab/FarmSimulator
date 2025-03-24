using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShiftStick : MonoBehaviour
{
	
	public int currentGear;
	public Rigidbody rb;
	
    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
	private void OnTriggerEnter(Collider other)
    {
		Debug.Log("GEAR SHIFT!");
        if (other.gameObject.layer == 7) {
			currentGear = other.gameObject.GetComponent<GearZone>().gear;
		}
    }
}
