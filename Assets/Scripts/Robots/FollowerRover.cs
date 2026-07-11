using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowerRover : MonoBehaviour
{
	
	public GameObject leader;
	
	public WheelCollider leftDrive;
	public WheelCollider rightDrive;
	
	public Transform dropZone;
	public float payloadWeight;
	
	public float parkDistance = 0.5f;
	public float repositionDistance = 0.7f;
	public bool parked;
	
	public float torque;
	public float brakeTorque;
	
	public float forwardInput;
	public float turnInput;
	
	public float heading; // compass reading
	
	public Rigidbody rb;
	public Transform flipPoint;
	
	public void Flip() {
		
		transform.rotation = Quaternion.identity;
		
	}

	public void AddFruit(FruitBody f) {
		
		payloadWeight += f.weight;
		
	}
	
	public void FlipCheck() {
		
		if(flipPoint == null) return;
		if(flipPoint.position.y < transform.position.y) {
			
			Flip();
			
		}
		
	}
	
	public void DriveUpdate() {
		if(leader == null || rb == null || leftDrive == null || rightDrive == null) return;
		
		float targetHeading = Mathf.Atan2(leader.transform.position.x - transform.position.x,
		leader.transform.position.z - transform.position.z
		) * Mathf.Rad2Deg;
		
		if(targetHeading < 0) {
			targetHeading += 360;
		}
		
		float deltaHeading = heading - targetHeading;
		if(deltaHeading > 180) {
			targetHeading += 360;
		}
		if(deltaHeading < -180) {
			heading += 360;
		}
		
		deltaHeading = heading - targetHeading;
		
		var localVelocity = transform.InverseTransformDirection(rb.velocity);
						
		float forwardSpeed = localVelocity.z;
		
		if(Vector3.Distance(transform.position, leader.transform.position) <= parkDistance && !parked) {
			Debug.Log("PARKED");
			parked = true;
		}
		
		if(Mathf.Abs(deltaHeading) > 1.8f) {
			
			if(!parked) {
				
				//float turnMult = (Mathf.Abs(deltaHeading) / 180f)*0.2f;
				float turnMult = 0.0f;
				
				if(heading < targetHeading) {
					turnInput = 0.8f + turnMult;
				}
				if(heading > targetHeading) {
					turnInput = -0.8f - turnMult;
				}
				
				
				if(forwardSpeed > 2) {
					forwardInput = -1;
				}
				if(forwardSpeed < -2) {
					forwardInput = 1;
				}
				
			}
			
		}
		else {
			
			
			if(!parked) {
				if(forwardSpeed < 1f) {
					forwardInput = 1;
				}
			}
		}
		
		if(parked) {
			
			leftDrive.brakeTorque = brakeTorque;
			rightDrive.brakeTorque = brakeTorque;
			if(Vector3.Distance(transform.position, leader.transform.position) >= repositionDistance) {
				parked = false;
			}
			rb.isKinematic = true;
		}
		else {
			leftDrive.brakeTorque = rightDrive.brakeTorque = 0;
			rb.isKinematic = false;
		}
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
		rb = GetComponent<Rigidbody>();
    }

    // Update is called once per frame
    void Update()
    {
		if(leader == null || leftDrive == null || rightDrive == null) return;
		
		FlipCheck();
		
		heading = transform.eulerAngles.y;
		
		DriveUpdate();
		
		float ld = 0;
		float rd = 0;
		
		ld += forwardInput;
		rd += forwardInput;
		ld += turnInput;
		rd -= turnInput;
		
		if(rd > 1) {
			rd = 1;
		}
		if(rd < -1) {
			rd = -1;
		}
		
		if(ld > 1) {
			ld = 1;
		}
		if(ld < -1) {
			ld = -1;
		}
		
        leftDrive.motorTorque = ld * torque;
		rightDrive.motorTorque = rd * torque;
    }
}
