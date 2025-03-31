using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TempDirt : MonoBehaviour
{
	
	public float timeAlive;
	
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
		
		transform.position += new Vector3(0,-Time.deltaTime*0.01f,0);
		
		if(timeAlive > 5) {
			Destroy(gameObject);
		}
		
        timeAlive += Time.deltaTime;
    }
}
