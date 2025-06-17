using System.Collections;
using System.Collections.Generic;
using UnityEngine;



public class RotateWheel : MonoBehaviour
{
	public WheelCollider col;
	public Transform visWheel;
	public Vector2 initialStiffness;
	
    // Start is called before the first frame update
    void Start()
    {
        col = GetComponent<WheelCollider>();
		initialStiffness = new Vector2(col.forwardFriction.stiffness,col.sidewaysFriction.stiffness);
		visWheel.transform.localScale = transform.localScale;
    }

    // Update is called once per frame
    void Update()
    {
        //visWheel.transform.position = col.GetWorldPose().position;
		Vector3 wPos = new Vector3(0,0,0);
		Quaternion wRot = new Quaternion();
		col.GetWorldPose(out wPos, out wRot);
		
		visWheel.transform.position = wPos;
		visWheel.transform.rotation = wRot;
		
    }
}
