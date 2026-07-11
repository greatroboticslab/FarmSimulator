using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UVCollider : MonoBehaviour
{
	
	public DebugRover rover;
	
    // Start is called before the first frame update
    void Start()
    {
        rover = transform.parent.gameObject.GetComponent<DebugRover>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
	
	private void OnTriggerEnter(Collider other) {
		
		if(other.gameObject.layer == 6) {
			Plant plant = other.gameObject.GetComponent<Plant>();
			if(plant == null) return;

			plant.killingMold = true;
			plant.rover = rover;
			
		}
		
	}
	
	private void OnTriggerExit(Collider other) {
		
		if(other.gameObject.layer == 6) {
			Plant plant = other.gameObject.GetComponent<Plant>();
			if(plant != null) plant.killingMold = false;
			
		}
		
	}
	
}
