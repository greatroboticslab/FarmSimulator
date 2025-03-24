using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class JointTest : MonoBehaviour
{
	
	public Rigidbody rb1;
	public Rigidbody rb2;
	public float t = 0;
	
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
		float trq = Mathf.Sin(t);
		rb1.AddRelativeTorque(trq,0,0);
		rb2.AddRelativeTorque(-trq,0,0);
			
		
		t += Time.deltaTime;
    }
}
