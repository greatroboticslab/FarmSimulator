using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class MLHumanoidController : Agent
{
	
	public float fitness = 0;
	public Director director;
	public Vector3 startPos;
	public Quaternion startRot;
	
	public HumanoidRobot h;
	
	//MLAGENTS STUFF
	
	public override void OnEpisodeBegin() {
		
		h.curTime = 0f;
		ResetSelf();
		
	}
	
	public override void CollectObservations(VectorSensor sensor) {

		//camera pixels do not belong in the fixed-size vector sensor
		foreach(float f in h.GetInputs(false)) {
			sensor.AddObservation(f);
		}

	}
	
	//Zero-torque heuristic so running without a trainer idles quietly instead
	//of logging a warning every step
	public override void Heuristic(in ActionBuffers actionsOut) {
		var continuous = actionsOut.ContinuousActions;
		for(int i = 0; i < continuous.Length; i++) continuous[i] = 0f;
	}

	public override void OnActionReceived(ActionBuffers actions) {
		//Debug.Log(actions.ContinuousActions[0]);
		for(int i = 0; i < h.joints.Length*3; i+=3) {
			float _torque = h.torqueMult*h.joints[i/3].strength;
			if(h.joints[i/3].parent != null) {
			//if(i > 0) {
			
				h.joints[i/3].parent.rb.AddRelativeTorque(new Vector3(actions.ContinuousActions[i]*_torque, actions.ContinuousActions[i+1]*_torque,actions.ContinuousActions[i+2]*_torque)*-1);
				h.joints[i/3].rb.AddRelativeTorque(new Vector3(actions.ContinuousActions[i]*_torque, actions.ContinuousActions[i+1]*_torque,actions.ContinuousActions[i+2]*_torque));
			
			}
			//joints[i/3].rb.AddTorque(torqueMult, torqueMult,torqueMult);
		}
	}
	
    // Start is called before the first frame update
    void Start()
    {
		
		
        startPos = transform.position;
		startRot = transform.rotation;
    }

    // Update is called once per frame
    void Update()
    {
        if(Mathf.Abs(h.transform.position.x) > 3000) {
			SetReward(-9999f);
			EndEpisode();
		}
		if(Mathf.Abs(h.transform.position.y) > 3000) {
			SetReward(-9999f);
			EndEpisode();
		}
		if(Mathf.Abs(h.transform.position.z) > 3000) {
			SetReward(-9999f);
			EndEpisode();
		}
		
		fitness += director.FrameFitness(this, "walk");
		
		if(h.curTime > director.timeout) {
			SetReward(director.GetFitness(this, "walk") + fitness);
			EndEpisode();
		}
		
    }
	
	public void ResetSelf() {
		fitness = 0;
		
		//Primer
		foreach(HumanoidRobot.HumanoidJoint j in h.joints) {
			j.rb.velocity = Vector3.zero;
			j.rb.angularVelocity = Vector3.zero;
			//j.rb.enabled = false;
		}
		
		transform.position = startPos;
		transform.rotation = startRot;
		
		//End
		foreach(HumanoidRobot.HumanoidJoint j in h.joints) {
			j.rb.velocity = Vector3.zero;
			j.rb.angularVelocity = Vector3.zero;
			j.gameObject.transform.localPosition = j.startPos;
			j.gameObject.transform.localRotation = j.startRot;
			//j.rb.enabled = true;
		}
	}
}
