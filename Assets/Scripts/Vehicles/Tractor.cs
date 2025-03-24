using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Tractor : MonoBehaviour
{
	
	public bool manualControl;
	public float manualBrakeAmount = 0f;
	
	public Transform steeringWheel;
	public Transform key;
	public float lastSteeringRot;
	private float steeringDelta = 0;
	private float steeringRotation = 0;
	public float steeringMultiplier = 0.1f;
	public float brakePower = 1000f;
	public List<Gear> gears;
	public float brakes = 0.0f;
	public float clutchAmount = 0.0f;
	public int currentGear = 1;
	public ShiftStick stick;
	public float ignitionProgress = 0; //Engine starts once reaches 1.0
	public float rpm = 0;
	public float keyAngle = 0;
	
	public GameObject clutch;
	public GameObject brake;
	
	public TractorWheelControl rearLeftWheel;
	public TractorWheelControl rearRightWheel;
	public TractorWheelControl frontLeftWheel;
	public TractorWheelControl frontRightWheel;
	
	[System.Serializable]
	public class Gear {
		
		public float torque;
		public float maxSpeed;
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
        lastSteeringRot = steeringWheel.localRotation.eulerAngles.y;
    }

    // Update is called once per frame
    void Update()
    {
		
		if(manualControl) {
			
			//Input.GetAxis("Vertical")
			
			if(Input.GetKey("c")) {
				clutch.gameObject.GetComponent<Rigidbody>().AddRelativeTorque(0,0,Time.deltaTime*-1750f);
			}
			
			brake.gameObject.GetComponent<Rigidbody>().AddRelativeTorque(0,0,Input.GetAxis("Vertical")*Time.deltaTime*1750f);
			
			steeringWheel.gameObject.GetComponent<Rigidbody>().AddRelativeTorque(0,0,Input.GetAxis("Horizontal")*Time.deltaTime*-750f);
			
			if(Input.GetKey(KeyCode.Keypad4)) {
				stick.rb.AddRelativeTorque(0,0,50f*Time.deltaTime);
			}
			if(Input.GetKey(KeyCode.Keypad6)) {
				stick.rb.AddRelativeTorque(0,0,-50f*Time.deltaTime);
			}
			
			if(Input.GetKey(KeyCode.Keypad8)) {
				stick.rb.AddRelativeTorque(50f*Time.deltaTime,0,0);
			}
			if(Input.GetKey(KeyCode.Keypad2)) {
				stick.rb.AddRelativeTorque(-50f*Time.deltaTime,0,0);
			}
			
		}
		
		float sd1 = 0;
		float sd2 = 0;
		float sd3 = 0;
		float csteering = steeringWheel.localRotation.eulerAngles.y;
		sd1 = csteering - (lastSteeringRot);
		sd2 = csteering - (lastSteeringRot - 360);
		sd3 = csteering - (lastSteeringRot + 360);
		steeringDelta = sd1;
		if(Mathf.Abs(sd2) < Mathf.Abs(sd1) && Mathf.Abs(sd2) < Mathf.Abs(sd3)) {
			steeringDelta = sd2;
		}
		if(Mathf.Abs(sd3) < Mathf.Abs(sd1) && Mathf.Abs(sd3) < Mathf.Abs(sd2)) {
			steeringDelta = sd3;
		}
		steeringRotation += steeringDelta;
		
        //Debug.Log(steeringRotation);
		
		//Brakes
		float b = brake.transform.localRotation.eulerAngles.z;
		
		if(b < 180) {
			b *= -1f;
		}
		else {
			b = 350f - b;
		}
		
		brakes = b/10f;
		
		if(brakes < 0) {
			brakes = 0f;
		}
		
		//Clutch
		float cl = brake.transform.localRotation.eulerAngles.z;
		
		if(cl < 180) {
			cl *= -1f;
		}
		else {
			cl = 350f - cl;
		}
		
		clutchAmount = cl/10f;
		
		if(clutchAmount < 0) {
			clutchAmount = 0f;
		}
		
		if(clutchAmount > 0.5f) {
			currentGear = stick.currentGear;
		}
		float gearTorque = 0;
		if(currentGear > 0) {
			gearTorque = gears[currentGear].torque;
		}
		if(currentGear == -1) {
			gearTorque = -gears[1].torque;
		}
		
		//Lower torque applied to wheel based on wheel speed
		float wheelTorque = gearTorque;
		float avgWheelSpeed = (rearRightWheel.WheelCollider.rotationSpeed + rearLeftWheel.WheelCollider.rotationSpeed)/2f;
		int c = currentGear;
		if(currentGear < 0) {
			c = 0;
		}
		if(currentGear > 0) {
			wheelTorque *= ((gears[c].maxSpeed - avgWheelSpeed)/gears[c].maxSpeed);
		}
		
		if(wheelTorque < 0 && currentGear > 0) {
			wheelTorque = 0;
		}
		
		//Debug.Log(rearRightWheel.WheelCollider.rotationSpeed);
		//Debug.Log(wheelTorque);
		
		//Do nothing for park since it is initialized to zero
		
		frontRightWheel.WheelCollider.steerAngle = steeringRotation*steeringMultiplier;
		frontLeftWheel.WheelCollider.steerAngle = steeringRotation*steeringMultiplier;
		
		//Debug.Log("====================================================");
		//Debug.Log(avgWheelSpeed);
		rearRightWheel.WheelCollider.motorTorque = wheelTorque * (1f - brakes);
		rearLeftWheel.WheelCollider.motorTorque = wheelTorque * (1f - brakes);
		//Debug.Log(brakes);
		
		rearRightWheel.WheelCollider.brakeTorque = brakes*brakePower;
		rearLeftWheel.WheelCollider.brakeTorque = brakes*brakePower;
		
		//frontRightWheel.WheelCollider.steerAngle = Random.Range(0,90);
		
		lastSteeringRot = steeringWheel.localRotation.eulerAngles.y;
		
		//Key / Engine
		keyAngle = key.localRotation.eulerAngles.y;
		//Debug.Log(keyAngle);
		
    }
}
