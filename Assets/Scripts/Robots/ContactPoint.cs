using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ContactPoint : MonoBehaviour
{
	
	public int touching = 0;
	
    // Start is called before the first frame update
    void Start()
    {
        
    }

	
	private void OnTriggerEnter(Collider other)
    {
        touching += 1;
    }
	
	private void OnTriggerExit(Collider other)
    {
        touching -= 1;
    }
	
}
