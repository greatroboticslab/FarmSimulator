using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class HumanoidRobot : Agent

{
	
	public Actor actor;
	public bool copy; //Copy an actor (kinematic)
	public bool mimic; //Mimic an actor by trying to apply torque to joints to match the pose
	public bool training;
	public float forward;
	public float leftRight;
	public int currentGear = -2;
	public int desiredGear = -2;
	public Tractor tractor;
	public bool ridingTractor;
	public bool selfDriving;
	public bool harvesting;
	public Plant currentCrop;
	public int currentFruit;
	public float walkSpeed = 1f;
	public float turnSpeed = 10f;
	public float rotation = 0f;
	public Vector3 kneelAngle;
	public float standHeight = 1.1f;
	public float kneelHeight = 0.5f;
	public List<int> harvestMask; //The list of ids of joints that are involved in harvesting/the only joints that move during harvesting
	public List<ContactPoint> rightHandContacts; //List of contacts that are the contact points for the right hand
	public List<ContactPoint> leftHandContacts; //List of contacts that are the contact points for the left hand
	public Transform rightGrab;
	private bool hasFruit;
	private float prevGrad = 9999f;
	private float maxRot = 0.02f;
	private float graspSpeed = 0.7f;
	private bool reachedFruit;
	private bool readyToHarvest;
	
	public FollowerRover basketRover;
	
	public float curTime = 0f;
	public float timeout = 5f;
	public float weightRange = 1.0f;
	public float biasRange = 1.0f;
	public float torqueMult = 3000f;
	public float samplingDistance = 0.01f;
	public float vradRatio = 0.5f;
	public float vradMultiplier = 0.5f;
	public Director director;
	public float fitness = 0;
	public Camera mainCamera;
	public PathMaker.Waypoint currentWaypoint;
	public bool gotWaypoint;
	public bool wantsToTeleport;
	public float heading;
	
	public Vector3 startPos;
	public Quaternion startRot;
	public ReferenceSkeleton refSkeleton;
	private float[] torques;
	
	public GameObject chest;
	
	public static float TanH(float input) {
		float e = 2.71f;
		//return (Mathf.Pow(e,2*input)-1)/(Mathf.Pow(e,2*input)+1);
		float numerator = (Mathf.Pow(e,input)-Mathf.Pow(e,-input));
		float denominator = (Mathf.Pow(e,input)+Mathf.Pow(e,-input));
		
		if(denominator == 0) {
			if(numerator > 0) {
				return 1f;
			}
			return -1f;
		}
		if(float.IsPositiveInfinity(numerator)) {
			return 1f;
		}
		if(float.IsNegativeInfinity(numerator)) {
			return -1f;
		}
		if(float.IsPositiveInfinity(denominator)) {
			return 0f;
		}
		if(float.IsNegativeInfinity(denominator)) {
			return 0f;
		}
		
		
		float output = numerator/denominator;
		//Debug.Log("raw : " + input + ", denom: " + denominator + ", output: " + output);
		return output;
	}
	
	[System.Serializable]
	public class ReferenceSkeleton {
		
		public Transform pelvis;
		public Transform leftFemur;
		public Transform leftTibia;
		public Transform leftFoot;
		public Transform rightFemur;
		public Transform rightTibia;
		public Transform rightFoot;
		
		public Transform lowerSpine;
		public Transform midSpine;
		public Transform upperSpine;
		public Transform neck;
		public Transform head;
		
		public Transform leftShoulder;
		public Transform leftBicep;
		public Transform leftForearm;
		public Transform leftHand;
		public Transform rightShoulder;
		public Transform rightBicep;
		public Transform rightForearm;
		public Transform rightHand;
		
		
		
	};
	
	private bool InMask(int i) {
		if(harvesting) {
			foreach(int j in harvestMask) {
				if(i == j) {
					return true;
				}
			}
		}
		return false;
	}
	
	//Kinematically copy a skeletons pose
	public void CopySkeleton(ReferenceSkeleton targetSkeleton) {
		
		//CopyJoint(refSkeleton.pelvis, targetSkeleton.pelvis);
		//refSkeleton.pelvis.rotation.eulerAngles.x = targetSkeleton.pelvis.rotation.eulerAngles.x;
		
		
		CopyJoint(refSkeleton.head, targetSkeleton.head);
		
		CopyJoint(refSkeleton.leftFemur, targetSkeleton.leftFemur);
		CopyJoint(refSkeleton.leftTibia, targetSkeleton.leftTibia);
		CopyJoint(refSkeleton.leftFoot, targetSkeleton.leftFoot);
		
		
		CopyJoint(refSkeleton.rightFemur, targetSkeleton.rightFemur);
		CopyJoint(refSkeleton.rightTibia, targetSkeleton.rightTibia);
		CopyJoint(refSkeleton.rightFoot, targetSkeleton.rightFoot);
		
		if(!InMask(1)) {
			CopyJoint(refSkeleton.lowerSpine, targetSkeleton.lowerSpine);
		}
		if(!InMask(2)) {
			CopyJoint(refSkeleton.midSpine, targetSkeleton.midSpine);
		}
		if(!InMask(3)) {
			CopyJoint(refSkeleton.upperSpine, targetSkeleton.upperSpine);
		}
		
		CopyJoint(refSkeleton.neck, targetSkeleton.neck);
		
		CopyJoint(refSkeleton.leftBicep, targetSkeleton.leftBicep);
		CopyJoint(refSkeleton.leftForearm, targetSkeleton.leftForearm);
		//CopyJoint(refSkeleton.leftHand, targetSkeleton.leftHand);
		
		if(!InMask(4)) {
			CopyJoint(refSkeleton.rightBicep, targetSkeleton.rightBicep);
		}
		if(!InMask(5)) {
			CopyJoint(refSkeleton.rightForearm, targetSkeleton.rightForearm);
		}
		if(!InMask(6)) {
			//CopyJoint(refSkeleton.rightHand, targetSkeleton.rightHand);
		}
	}
	
	public void CopyJoint(Transform _base, Transform _target) {
		
		_base.GetComponent<Rigidbody>().isKinematic = true;
		_base.localRotation = _target.localRotation;
		
	}
	
	//Used to physically mimic a reference animation
	public void MimicSkeleton(ReferenceSkeleton targetSkeleton) {
		
		//MimicJoint(refSkeleton.pelvis, targetSkeleton.pelvis);
		
		
		MimicJoint(refSkeleton.head, targetSkeleton.head);
		
		MimicJoint(refSkeleton.leftFemur, targetSkeleton.leftFemur);
		MimicJoint(refSkeleton.leftTibia, targetSkeleton.leftTibia);
		MimicJoint(refSkeleton.leftFoot, targetSkeleton.leftFoot);
		
		
		MimicJoint(refSkeleton.rightFemur, targetSkeleton.rightFemur);
		MimicJoint(refSkeleton.rightTibia, targetSkeleton.rightTibia);
		MimicJoint(refSkeleton.rightFoot, targetSkeleton.rightFoot);
		
		MimicJoint(refSkeleton.lowerSpine, targetSkeleton.lowerSpine);
		MimicJoint(refSkeleton.midSpine, targetSkeleton.midSpine);
		MimicJoint(refSkeleton.upperSpine, targetSkeleton.upperSpine);
		
		MimicJoint(refSkeleton.neck, targetSkeleton.neck);
		
		MimicJoint(refSkeleton.leftBicep, targetSkeleton.leftBicep);
		MimicJoint(refSkeleton.leftForearm, targetSkeleton.leftForearm);
		MimicJoint(refSkeleton.leftHand, targetSkeleton.leftHand);
		
		MimicJoint(refSkeleton.rightBicep, targetSkeleton.rightBicep);
		MimicJoint(refSkeleton.rightForearm, targetSkeleton.rightForearm);
		MimicJoint(refSkeleton.rightHand, targetSkeleton.rightHand);
		
	}
	
	public void MimicJoint(Transform _base, Transform _target) {
		
		float tMult = 1f * torqueMult;
		
		Vector3 baseAngles = _base.localEulerAngles;
		Vector3 targetAngles = _target.localEulerAngles;
		Vector3 deltaAngles = targetAngles - baseAngles;
		
		//deltaAngles = new Vector3(deltaAngles.x*deltaAngles.x,deltaAngles.y*deltaAngles.y,deltaAngles.z*deltaAngles.z);
		
		foreach(HumanoidJoint j in joints) {
			if(j.gameObject.transform == _base) {
				Vector3 vec = new Vector3(1,0,0);
				float aForce = 0f;
				float grad = 0;
				float vrad = 0;
				float finalGradient = 0;
				float s = 0;
				//j.rb.AddRelativeTorque(deltaAngles*tMult*j.strength);
				
				grad = GetGradient(j, _target, vec);
				vrad = GetVelGradient(j, _target, vec);
				s = InverseSpeed(j, vec);
				if(aForce > 1) {
					aForce = 1;
				}
				if(aForce < -1) {
					aForce = -1;
				}
				
				finalGradient = s;
				
				j.rb.AddRelativeTorque(vec*grad*tMult*j.strength);
				vec = new Vector3(0,1,0);
				grad = GetGradient(j, _target, vec);
				vrad = GetVelGradient(j, _target, vec);
				s = InverseSpeed(j, vec);
				finalGradient = s;
				j.rb.AddRelativeTorque(vec*grad*tMult*j.strength);
				
				vec = new Vector3(0,0,1);
				grad = GetGradient(j, _target, vec);
				vrad = GetVelGradient(j, _target, vec);
				s = InverseSpeed(j, vec);
				finalGradient = s;
				j.rb.AddRelativeTorque(vec*grad*tMult*j.strength);
			}
		}
		
	}
	
	float InverseSpeed(HumanoidJoint joint, Vector3 axis) {
		
		if(axis.x != 0) {
			return -joint.rb.angularVelocity.x;
		}
		if(axis.y != 0) {
			return -joint.rb.angularVelocity.y;
		}
		if(axis.z != 0) {
			return -joint.rb.angularVelocity.z;
		}
		
		return -1;
	}
	
	float GetGradient(HumanoidJoint joint, Transform targetJoint, Vector3 axis) {
		float output = 0f;
		
		float f_x = 0;
		float f_x_d = 0;
		
		f_x = Vector3.Angle(joint.gameObject.transform.forward, targetJoint.forward);
		joint.gameObject.transform.Rotate(axis*samplingDistance, Space.Self);
		f_x_d = Vector3.Angle(joint.gameObject.transform.forward, targetJoint.forward);
		joint.gameObject.transform.Rotate(axis*-samplingDistance, Space.Self);
			
		output = (f_x - f_x_d) / samplingDistance;
		
		return output;
	}
	
	//Gets the gradient of the angular velocity to the position
	float GetVelGradient(HumanoidJoint joint, Transform targetJoint, Vector3 axis) {
		float output = 0f;
		
		float f_x = 0;
		float f_x_d = 0;
		float aVel = 0f;
		if(axis.x != 0) {
			aVel = joint.rb.angularVelocity.x;
		}
		if(axis.y != 0) {
			aVel = joint.rb.angularVelocity.y;
		}
		if(axis.z != 0) {
			aVel = joint.rb.angularVelocity.z;
		}
		aVel *= vradMultiplier;
		
		f_x = Vector3.Angle(joint.gameObject.transform.forward, targetJoint.forward);
		joint.gameObject.transform.Rotate(axis*samplingDistance*aVel, Space.Self);
		f_x_d = Vector3.Angle(joint.gameObject.transform.forward, targetJoint.forward);
		joint.gameObject.transform.Rotate(axis*-samplingDistance*aVel, Space.Self);
			
		output = (f_x - f_x_d) / samplingDistance;
		
		return output;
	}
	
	public class NNLayer {
		
		public float[][] weights;
		public float[] biases;
		
	};
	
	public class NeuralNetwork {
		public float[] input;
		public List<NNLayer> layers;
		
		public float[] SaveNetwork() {
			
			List<float> lData = new List<float>();
			foreach(NNLayer l in layers) {
				foreach(float[] f in l.weights) {
					foreach(float _f in f) {
						lData.Add(_f);
					}
				}
				foreach(float f in l.biases) {
					lData.Add(f);
				}
			}
			
			return lData.ToArray();
			
		}
		
		public void LoadNetwork(float[] data) {
			int idx = 0;
			for(int i = 0; i < layers.Count; i++) {
				for(int j = 0; j < layers[i].weights.Length; j++) {
					for(int k = 0; k < layers[i].weights[j].Length; k++) {
						layers[i].weights[j][k] = data[idx];
						idx += 1;
					}
				}
				for(int j = 0; j < layers[i].biases.Length; j++) {
					layers[i].biases[j] = data[idx];
					idx += 1;
				}
			}
		}
		
		public float[] ForwardPass() {
			
			float[] layerInput = input;
			
			float[] output = new float[layers[layers.Count-1].weights[0].Length];
			for(int i = 0; i < layers.Count; i++) {
				float[] layerOutput = new float[layers[i].weights[0].Length];
				for(int j = 0; j < layerOutput.Length; j++) {
					for(int k = 0; k < layers[i].weights.Length; k++) {
						layerOutput[j] += layers[i].weights[k][j] * layerInput[k];
						layerOutput[j] += layers[i].biases[j];
					}
				}
				layerInput = layerOutput;
			}
			output = layerInput;
			for(int i = 0; i < output.Length; i++) {
				output[i] = TanH(output[i]);
			}
			
			return output;
		}
		
		public float[] ForwardPass(float[] _input) {
			
			
			for(int i = 0; i < _input.Length; i++) {
				input[i] = _input[i];
			}
			
			return ForwardPass();
			
		}
		
	}
	
	[System.Serializable]
	public class NeuralNetworkBase {
		public List<int> hiddenLayers;
	};
	
	[System.Serializable]
	public class HumanoidJoint {
		public GameObject gameObject;
		public HumanoidJoint parent;
		public Vector3 startPos;
		public Quaternion startRot;
		public Rigidbody rb;
		public float strength;
	};
	
	public NeuralNetworkBase neuralNetwork;
	public NeuralNetwork nn;
	
	public HumanoidJoint[] joints;
	public Collider[] contacts;
	public GameObject[] accelerometers;
	
	public void GenerateNeuralNetwork(bool randomize = false) {
		nn = new NeuralNetwork();
		nn.layers = new List<NNLayer>();
		List<float> inputs = new List<float>();
		//Contacts First
		foreach(Collider c in contacts) {
			inputs.Add(0);
		}
		
		
		//Output Layer, pitch yaw roll
		int outputNodes = 0;
		//outputNodes = GetInputs().Length;
		
		
		foreach(HumanoidJoint j in joints) {
			outputNodes += 3;
		}
		
		//nn.output = new float[outputNodes];
		
		nn.input = GetInputs();
		for(int i = 0; i < neuralNetwork.hiddenLayers.Count; i ++) {
			NNLayer newLayer = new NNLayer();
			int prevNodes = nn.input.Length;
			if(i > 0) {
				//prevNodes = nn.layers[i].weights[0].Length;
				prevNodes = neuralNetwork.hiddenLayers[i-1];
			}
			newLayer.weights = new float[prevNodes][];
			for(int j = 0; j < prevNodes; j++) {
				newLayer.weights[j] = new float[neuralNetwork.hiddenLayers[i]];
				if(randomize) {
					for(int k = 0; k < neuralNetwork.hiddenLayers[i]; k++) {
						newLayer.weights[j][k] = Random.Range(-weightRange,weightRange);
					}
				}
			}
			
			newLayer.biases = new float[neuralNetwork.hiddenLayers[i]];
			if(randomize) {
				for(int j = 0; j < neuralNetwork.hiddenLayers[i]; j++) {
					newLayer.biases[j] = Random.Range(-biasRange,biasRange);
				}
			}
			nn.layers.Add(newLayer);
			
		}
		//Add output layer
		
		//Debug.Log(nn.layers);
		int _prevNodes = nn.input.Length;
		if(nn.layers.Count > 0) {
			_prevNodes = nn.layers[neuralNetwork.hiddenLayers.Count-1].weights[0].Length;
		}
		NNLayer _newLayer = new NNLayer();
		_newLayer.weights = new float[_prevNodes][];
		for(int j = 0; j < _prevNodes; j++) {
			_newLayer.weights[j] = new float[outputNodes];
			if(randomize) {
				for(int k = 0; k < outputNodes; k++) {
					_newLayer.weights[j][k] = Random.Range(-weightRange,weightRange);
				}
			}
		}
		
		_newLayer.biases = new float[outputNodes];
		if(randomize) {
			for(int j = 0; j < outputNodes; j++) {
				_newLayer.biases[j] = Random.Range(-biasRange,biasRange);
			}
		}
		nn.layers.Add(_newLayer);
		
		
	}
	
	public void GetAllJoints() {
		
		List<GameObject> a = new List<GameObject>();
		List<Collider> c = new List<Collider>();
		List<HumanoidJoint> j = new List<HumanoidJoint>();
		List<GameObject> objs = GetChildren(gameObject);
		int v = 0;
		foreach(GameObject o in objs) {
			
			if(o.GetComponent<RobotJoint>()) {
				
				o.GetComponent<RobotJoint>().jointId = v;
				v += 1;
				
				if(o.GetComponent<RobotJoint>().accelerometer) {
					a.Add(o);
				}
				
				HumanoidJoint newJoint = new HumanoidJoint();
				
				
				
				newJoint.startPos = o.transform.localPosition;
				newJoint.startRot = o.transform.localRotation;
				newJoint.gameObject = o;
				newJoint.strength = o.GetComponent<RobotJoint>().muscleStrength;
				newJoint.rb = o.GetComponent<Rigidbody>();
				j.Add(newJoint);
			}
			else {
				Collider col = o.GetComponent<Collider>();
				if(col) {
					if(col.isTrigger) {
						c.Add(col);
					}
				}
			}
		}
		
		contacts = c.ToArray();
		joints = j.ToArray();
		accelerometers = a.ToArray();
		
		
		//foreach(HumanoidJoint jo in joints) {
		for(int i = 0; i < joints.Length; i++) {
			//Debug.Log("attempting_to_add");
			if(joints[i].gameObject.transform.parent) {
				GameObject potentialParent = joints[i].gameObject.transform.parent.gameObject;
				if(potentialParent.GetComponent<RobotJoint>()) {
					//Debug.Log("parent_added");
					joints[i].parent = joints[potentialParent.GetComponent<RobotJoint>().jointId];
				}
			}
		}
		
		
	}
	
	public List<GameObject> GetChildren(GameObject o) {
		List<GameObject> output = new List<GameObject>();
		
		output.Add(o);
		
		foreach(Transform child in o.transform) {
			output.AddRange(GetChildren(child.gameObject));
		}
		
		return output;
	}
	
	public void RunJoints(float[] torques) {
		
		
		
		//Debug.Log(torques[0]);
		
		for(int i = 0; i < torques.Length; i+=3) {
			joints[i/3].rb.AddTorque(new Vector3(torques[i]*torqueMult,torques[i+1]*torqueMult,torques[i+2]*torqueMult));
		}
	}
	
	public float[] GetInputs() {
		List<float> lInputs = new List<float>();
		
		//Time
		lInputs.Add(curTime);
		
		//Camera
		RenderTexture.active = mainCamera.targetTexture;
		Texture2D currenttexture = new Texture2D(1, 1, TextureFormat.RGBA32, false, true);
		currenttexture.ReadPixels(new Rect(0,0,mainCamera.targetTexture.width,mainCamera.targetTexture.height), 0, 0, false);
		for(int y = 0; y < mainCamera.targetTexture.height; y++) {
			for(int x = 0; x < mainCamera.targetTexture.width; x++) {
				
				
				
				Color p = currenttexture.GetPixel(x,y);
				lInputs.Add(p.r);
				lInputs.Add(p.g);
				lInputs.Add(p.b);
			}
		}
		
		//Velocity
		foreach(GameObject a in accelerometers) {
			lInputs.Add(a.GetComponent<Rigidbody>().velocity.x);
			lInputs.Add(a.GetComponent<Rigidbody>().velocity.y);
			lInputs.Add(a.GetComponent<Rigidbody>().velocity.z);
		}
		
		//Angular Velocity
		foreach(GameObject a in accelerometers) {
			lInputs.Add(a.GetComponent<Rigidbody>().angularVelocity.x);
			lInputs.Add(a.GetComponent<Rigidbody>().angularVelocity.y);
			lInputs.Add(a.GetComponent<Rigidbody>().angularVelocity.z);
		}
		
		//Joint orientations
		foreach(HumanoidJoint j in joints) {
			
			lInputs.Add(j.gameObject.transform.localRotation.x);
			lInputs.Add(j.gameObject.transform.localRotation.y);
			lInputs.Add(j.gameObject.transform.localRotation.z);
			lInputs.Add(j.gameObject.transform.localRotation.w);
			
		}
		
		//Contact points
		foreach(Collider c in contacts) {
			if(c.gameObject.GetComponent<ContactPoint>().touching > 0) {
				lInputs.Add(1f);
			}
			else {
				lInputs.Add(0f);
			}
		}
		return lInputs.ToArray();
	}
	
	
	public void ResetSelf() {
		fitness = 0;
		
		//Primer
		foreach(HumanoidJoint j in joints) {
			j.rb.velocity = Vector3.zero;
			j.rb.angularVelocity = Vector3.zero;
			//j.rb.enabled = false;
		}
		
		transform.position = startPos;
		transform.rotation = startRot;
		
		//End
		foreach(HumanoidJoint j in joints) {
			j.rb.velocity = Vector3.zero;
			j.rb.angularVelocity = Vector3.zero;
			j.gameObject.transform.localPosition = j.startPos;
			j.gameObject.transform.localRotation = j.startRot;
			//j.rb.enabled = true;
		}
	}
	
	//MLAGENTS STUFF
	
	public override void OnEpisodeBegin() {
		
		curTime = 0f;
		ResetSelf();
		
	}
	
	public override void CollectObservations(VectorSensor sensor) {
		
		foreach(float f in GetInputs()) {
			sensor.AddObservation(f);
		}
		
	}
	
	public override void OnActionReceived(ActionBuffers actions) {
		//Debug.Log(actions.ContinuousActions[0]);
		for(int i = 0; i < joints.Length*3; i+=3) {
			float _torque = torqueMult*joints[i/3].strength;
			if(joints[i/3].parent != null) {
			//if(i > 0) {
			
				joints[i/3].parent.rb.AddRelativeTorque(new Vector3(actions.ContinuousActions[i]*_torque, actions.ContinuousActions[i+1]*_torque,actions.ContinuousActions[i+2]*_torque)*-1);
				joints[i/3].rb.AddRelativeTorque(new Vector3(actions.ContinuousActions[i]*_torque, actions.ContinuousActions[i+1]*_torque,actions.ContinuousActions[i+2]*_torque));
			
			}
			//joints[i/3].rb.AddTorque(torqueMult, torqueMult,torqueMult);
		}
	}
	
	public float PoseDelta(int j) {
		
		Quaternion a = joints[j].gameObject.transform.localRotation;
		
		if(j == 1) {
			return Quaternion.Angle(a, actor.refSkeleton.lowerSpine.localRotation);
		}
		if(j == 2) {
			return Quaternion.Angle(a, actor.refSkeleton.midSpine.localRotation);
		}
		if(j == 3) {
			return Quaternion.Angle(a, actor.refSkeleton.upperSpine.localRotation);
		}
		if(j == 4) {
			return Quaternion.Angle(a, actor.refSkeleton.leftBicep.localRotation);
		}
		if(j == 5) {
			return Quaternion.Angle(a, actor.refSkeleton.leftForearm.localRotation);
		}
		
		return -1f;
		
	}
	
	
	//If all the contacts in a hand are touching something
	public bool Grasping(bool left) {
		if(left) {
			
		}
		else {
			foreach(ContactPoint c in rightHandContacts) {
				
				if(c.touching <= 0) {
					return false;
				}
				
			}
			return true;
		}
		return false;
	}
	
	//If at least one of the contacts in a hand are touching something
	public bool LooselyGrasping(bool left) {
		if(left) {
			
		}
		else {
			foreach(ContactPoint c in rightHandContacts) {
				
				if(c.touching > 0) {
					return true;
				}
				
			}
			return false;
		}
		return false;
	}
	
	public void OpenHand(bool left) {
		
		if(left) {
			
		}
		else {
			
			joints[6].gameObject.transform.localRotation = Quaternion.Slerp(joints[6].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 20), Time.deltaTime * graspSpeed);
			
			joints[7].gameObject.transform.localRotation = Quaternion.Slerp(joints[7].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 45), Time.deltaTime * graspSpeed);
			joints[8].gameObject.transform.localRotation = Quaternion.Slerp(joints[8].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -15), Time.deltaTime * graspSpeed);
			
			joints[9].gameObject.transform.localRotation = Quaternion.Slerp(joints[9].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -5), Time.deltaTime * graspSpeed);
			joints[10].gameObject.transform.localRotation = Quaternion.Slerp(joints[10].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -20), Time.deltaTime * graspSpeed);
			
			joints[11].gameObject.transform.localRotation = Quaternion.Slerp(joints[11].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 45), Time.deltaTime * graspSpeed);
			joints[12].gameObject.transform.localRotation = Quaternion.Slerp(joints[12].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -15), Time.deltaTime * graspSpeed);
			
			//joints[10].gameObject.transform.localRotation = Quaternion.Euler(0, 0, 20);
			
		}
		
	}
	
	public void CloseHand(bool left) {
		
		if(left) {
			
		}
		else {
			
			joints[6].gameObject.transform.localRotation = Quaternion.Slerp(joints[6].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 20), Time.deltaTime * graspSpeed);
			
			joints[7].gameObject.transform.localRotation = Quaternion.Slerp(joints[7].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 0), Time.deltaTime * graspSpeed);
			joints[8].gameObject.transform.localRotation = Quaternion.Slerp(joints[8].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -65), Time.deltaTime * graspSpeed);
			
			
			joints[9].gameObject.transform.localRotation = Quaternion.Slerp(joints[9].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -15), Time.deltaTime * graspSpeed);
			joints[10].gameObject.transform.localRotation = Quaternion.Slerp(joints[10].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 15), Time.deltaTime * graspSpeed);
			
			joints[11].gameObject.transform.localRotation = Quaternion.Slerp(joints[11].gameObject.transform.localRotation, Quaternion.Euler(0, 0, 0), Time.deltaTime * graspSpeed);
			joints[12].gameObject.transform.localRotation = Quaternion.Slerp(joints[12].gameObject.transform.localRotation, Quaternion.Euler(0, 0, -65), Time.deltaTime * graspSpeed);
			
			//joints[10].gameObject.transform.localRotation = Quaternion.Euler(0, 0, 20);
			
		}
		
	}
	
	public void HarvestCrop(Plant p) {
		actor.anim.SetBool("harvesting",true);
		Debug.Log("Harvesting!");
		harvesting = true;
		currentCrop = p;
		currentFruit = 0;
		transform.position = new Vector3(transform.position.x,  + transform.position.y - (standHeight - kneelHeight), transform.position.z);
		transform.Rotate(kneelAngle);
		prevGrad = 999f;
	}
	
	public void ReachFor(Vector3 pos) {
		
		Debug.Log("reaching");
		
		float avgGrad = 0;
		int ja = 0;
		float g;
		
		foreach(int i in harvestMask) {
			
			ja += 1;
			
			Vector3 a = new Vector3(1,0,0);
			g = GetHarvestGradient(i, a, pos);
			if(g > maxRot) {
				g = maxRot;
			}
			if(g < -maxRot) {
				g = -maxRot;
			}
			joints[i].gameObject.transform.Rotate(a * g * Time.deltaTime * 2150f);
			avgGrad += g;
			
			a = new Vector3(0,1,0);
			g = GetHarvestGradient(i, a, pos);
			if(g > maxRot) {
				g = maxRot;
			}
			if(g < -maxRot) {
				g = -maxRot;
			}
			joints[i].gameObject.transform.Rotate(a * g * Time.deltaTime * 2150f);
			avgGrad += g;
			
			a = new Vector3(0,0,1);
			g = GetHarvestGradient(i, a, pos);
			if(g > maxRot) {
				g = maxRot;
			}
			if(g < -maxRot) {
				g = -maxRot;
			}
			joints[i].gameObject.transform.Rotate(a * g * Time.deltaTime * 2150f);
			avgGrad += g;
			
		}
		
		avgGrad /= ja;
		prevGrad = avgGrad;
		
	}
	
	//Run this update while harvesting
	public void HarvestUpdate() {
		
		if(Vector3.Distance(transform.position, basketRover.transform.position) <= 3.0f) {
			readyToHarvest = true;
		}
		
		if(readyToHarvest) {
		
			float avgGrad = 0;
			int ja = 0;
			float g = 0;
			
			if(!hasFruit) {
			
				foreach(int i in harvestMask) {
					
					ja += 1;
					
					Vector3 a = new Vector3(1,0,0);
					g = GetHarvestGradient(i, a);
					if(g > maxRot) {
						g = maxRot;
					}
					if(g < -maxRot) {
						g = -maxRot;
					}
					joints[i].gameObject.transform.Rotate(a * g * Time.deltaTime * 2150f);
					avgGrad += g;
					
					a = new Vector3(0,1,0);
					g = GetHarvestGradient(i, a);
					if(g > maxRot) {
						g = maxRot;
					}
					if(g < -maxRot) {
						g = -maxRot;
					}
					joints[i].gameObject.transform.Rotate(a * g * Time.deltaTime * 2150f);
					avgGrad += g;
					
					a = new Vector3(0,0,1);
					g = GetHarvestGradient(i, a);
					if(g > maxRot) {
						g = maxRot;
					}
					if(g < -maxRot) {
						g = -maxRot;
					}
					joints[i].gameObject.transform.Rotate(a * g * Time.deltaTime * 2150f);
					avgGrad += g;
					
				}
				
				
				avgGrad /= ja;
				prevGrad = avgGrad;
			}
			
			if(HarvestDistance() < 0.015f) {
				
				reachedFruit = true;
				CloseHand(false);
				
			}
			else if(reachedFruit) {
				CloseHand(false);
			}
			else {
				
				OpenHand(false);
				
			}
			
			if(Grasping(false) && HarvestDistance() < 0.015f) {
				
				//currentCrop.fruits[currentFruit].GetComponent<FruitBody>().Detach();
				hasFruit = true;
				
			}
			if(!LooselyGrasping(false) || HarvestDistance() > 0.025f) {
				//hasFruit = false;
			}
			
			//Has grabbed a fruit successfully
			if(hasFruit) {
				
				//Debug.Log("HASFRUIT");
				currentCrop.fruits[currentFruit].transform.position = rightGrab.transform.position;
				ReachFor(basketRover.dropZone.position + new Vector3(0,1,0));
				
				if(currentCrop.fruits[currentFruit].GetComponent<FruitBody>().readyToDrop) {
					
					hasFruit = false;
					reachedFruit = false;
					currentCrop.fruits[currentFruit].GetComponent<FruitBody>().Detach();
					currentFruit += 1;
					if(currentFruit >= currentCrop.fruits.Count) {
						harvesting = false;
						transform.rotation = Quaternion.identity;
					}
					
					
				}
				
			}
		}
		
	}
	
	float HarvestDistance(Vector3 pos) {
		return Vector3.Distance(rightGrab.position, pos);
	}
	
	float HarvestDistance() {
		
		return Vector3.Distance(rightGrab.position, currentCrop.fruits[currentFruit].transform.position);
		
	}
	
	float GetHarvestGradient(int i, Vector3 axis) {
		float output = 0f;
		
		if(currentCrop.fruits[currentFruit] != null) {
			
			float f_x = 0;
			float f_x_d = 0;
			
			f_x = HarvestDistance() + (PoseDelta(i)*0.001f);
			joints[i].gameObject.transform.Rotate(axis * samplingDistance, Space.Self);
			f_x_d = HarvestDistance() + (PoseDelta(i)*0.001f);
			joints[i].gameObject.transform.Rotate(axis * -samplingDistance, Space.Self);
				
			output = (f_x - f_x_d) / samplingDistance;
			
		}
		
		
		
		return output;
	}
	
	float GetHarvestGradient(int i, Vector3 axis, Vector3 pos) {
		float output = 0f;
		
		if(currentCrop.fruits[currentFruit] != null) {
			
			float f_x = 0;
			float f_x_d = 0;
			
			f_x = HarvestDistance(pos) + (PoseDelta(i)*0.001f);
			joints[i].gameObject.transform.Rotate(axis * samplingDistance, Space.Self);
			f_x_d = HarvestDistance(pos) + (PoseDelta(i)*0.001f);
			joints[i].gameObject.transform.Rotate(axis * -samplingDistance, Space.Self);
				
			output = (f_x - f_x_d) / samplingDistance;
			
		}
		
		return output;
	}
	
	//Humanoid equivalent to self drive update
	public void MoveUpdate() {
		
		leftRight = 0;
		forward = 0;
		
		if(harvesting) {
			HarvestUpdate();
		}
		
		else {
			
			actor.anim.SetBool("harvesting",false);
			
			if(currentWaypoint != null) {
				
				Vector2 rPos = new Vector2(transform.position.x, transform.position.z);
				
				float targetHeading = Mathf.Atan2(currentWaypoint.pos.x - rPos.x,
				currentWaypoint.pos.y - rPos.y
				) * Mathf.Rad2Deg;
				
				if(targetHeading < 0) {
					targetHeading += 360;
				}
				
				//Debug.Log(targetHeading);
				
				float deltaHeading = heading - targetHeading;
				if(deltaHeading > 180) {
					targetHeading += 360;
				}
				if(deltaHeading < -180) {
					heading += 360;
				}
				
				deltaHeading = heading - targetHeading;
				
				
				bool reached = false;
				if(currentWaypoint.checkWater) {
					if(currentWaypoint.aimOnly) {
						if(Mathf.Abs(deltaHeading) <= 1.2f) {
							reached = true;
						}
					}
					else {
						if(Vector3.Distance(rPos, currentWaypoint.pos) < 2.5f) {
							if(Mathf.Abs(deltaHeading) <= 1.2f) {
								reached = true;
							}
						}
					}
				}
				else if(currentWaypoint.pickFruit) {
					if(Vector3.Distance(rPos, currentWaypoint.pos) < 2.5f) {
						if(Mathf.Abs(deltaHeading) <= 1.2f) {
							reached = true;
						}
					}
				}
				else {
					if(Vector3.Distance(rPos, currentWaypoint.pos) < 0.4f) {
						reached = true;
					}
				}
				
				
				//Reached Waypoint
				if(reached) {
					
					if(currentWaypoint.checkWater) {
						//TestPlant();
					}
					if(currentWaypoint.pickFruit) {
						readyToHarvest = false;
						HarvestCrop(currentWaypoint.plant.GetComponent<Plant>());
					}
					currentWaypoint = PathMaker.Instance.GetNextWaypoint(transform.position);
				}
				
				
				
				
				if(Mathf.Abs(deltaHeading) > 0.4f) {
					
					if(heading < targetHeading) {
						leftRight = Mathf.Sqrt(Mathf.Abs(deltaHeading)+1)/13f;
					}
					if(heading > targetHeading) {
						leftRight = -Mathf.Sqrt(Mathf.Abs(deltaHeading)+1)/13f;
					}
					
					
					
				}
				if(currentWaypoint.pickFruit) {
					if(Vector3.Distance(rPos, currentWaypoint.pos) > 2.0f) {
						forward = 1f/Mathf.Sqrt(Mathf.Abs(deltaHeading)+1);
					}
				}
				else {
					if(Vector3.Distance(rPos, currentWaypoint.pos) > 0.3f) {
						forward = 1f/Mathf.Sqrt(Mathf.Abs(deltaHeading)+1);
					}
				}
			}
		}
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
		if(training) {
			director = transform.parent.gameObject.GetComponent<Director>();
		}
		startPos = transform.position;
		startRot = transform.rotation;
        GetAllJoints();
		Debug.Log("Inputs count: " + GetInputs().Length);
		Debug.Log("Outputs count: " + joints.Length*3);
		GenerateNeuralNetwork(true);
		
		if(copy) {
			
			actor.anim.SetFloat("speed",0);
			//actor.anim.SetBool("tractor",true);
			
		}
		
		
    }
	
	void Update() {
		
		
		heading = transform.eulerAngles.y;
		
		if(wantsToTeleport) {
			
			if(PathMaker.Instance.waypoints.Count > 0) {
			
				wantsToTeleport = false;
				//Debug.Log("TELEPORTING...");
				Vector2 tPos = PathMaker.Instance.waypoints[0].pos;
				transform.position = new Vector3(tPos.x + 5.0f, transform.position.y + 10.0f, tPos.y);
			}
		}
		
		if(!gotWaypoint) {
			if(PathMaker.Instance.loaded) {
				gotWaypoint = true;
				currentWaypoint = PathMaker.Instance.GetNextWaypoint(transform.position);
			}
		}
		
		if(training) {
		
			if(Mathf.Abs(transform.position.x) > 3000) {
				SetReward(-9999f);
				EndEpisode();
			}
			if(Mathf.Abs(transform.position.y) > 3000) {
				SetReward(-9999f);
				EndEpisode();
			}
			if(Mathf.Abs(transform.position.z) > 3000) {
				SetReward(-9999f);
				EndEpisode();
			}
		}
		
		//float[] inputs = GetInputs();
		//torques = nn.ForwardPass(inputs);
		
		//TEST
		
		/*
		for(int i = 0; i < joints.Length*3; i+=3) {
			float t = torqueMult * Mathf.Sin(curTime*4)*0.01f;
			joints[i/3].rb.AddRelativeTorque(new Vector3(t,0,t));
			if(joints[i/3].parent != null) {
				joints[i/3].parent.rb.AddRelativeTorque(new Vector3(-t,0,0));
				joints[i/3].rb.AddRelativeTorque(new Vector3(t,0,0));
			}
		}
		*/
		
		
		
		//Play actor animations
		if(copy) {
			
			if(!selfDriving) {
				forward = Input.GetAxis("Vertical");
				leftRight = Input.GetAxis("Horizontal");
			}
			else {
				MoveUpdate();
			}
			
			transform.Rotate(new Vector3(0,leftRight*Time.deltaTime*turnSpeed,0), Space.Self);
			CopySkeleton(actor.refSkeleton);
			//rotation += leftRight*Time.deltaTime*turnSpeed;
			//transform.Rotate(new Vector3(0,rotation,0), Space.Self);
			
			transform.position += transform.forward*forward*Time.deltaTime*walkSpeed;
			
			if(forward != 0) {
				
				LayerMask layerMask = ~LayerMask.GetMask("humanoid");
				RaycastHit ghit;
				// Does the ray intersect any objects excluding the player layer
				if (Physics.Raycast(transform.position + new Vector3(0,1,0), transform.TransformDirection(-Vector3.up), out ghit, Mathf.Infinity, layerMask))
				{
					float hHeight = standHeight;
					if(harvesting) {
						hHeight = kneelHeight;
					}
					transform.position = new Vector3(transform.position.x, ghit.point.y + hHeight, transform.position.z);
				}
				
			}
			
			//joints[0].rb.constraints = RigidbodyConstraints.FreezeRotationX | RigidbodyConstraints.FreezeRotationZ;
			
			actor.transform.position = transform.position + new Vector3(0,3,0);
			
			bool brake = false;
			if(Input.GetKey("space")) {
				brake = true;
			}
			actor.anim.SetBool("brake",brake);
			actor.anim.SetFloat("speed",Mathf.Abs(forward) + Mathf.Abs(leftRight));
			if(forward == 0 && leftRight == 0) {
				actor.anim.SetBool("idle",true);
			}
			else {
				actor.anim.SetBool("idle",false);
			}
			
			if(currentGear == -2 && desiredGear == 1) {
				actor.anim.SetInteger("gear", desiredGear);
				actor.anim.SetBool("changeGear", true);
			}
			
			
			//transform.Rotate(new Vector3(0,rotation,0), Space.Self);
			
		}
		
		if(training) {
		
			fitness += director.FrameFitness(this, "walk");
			
			if(curTime > timeout) {
				SetReward(director.GetFitness(this, "walk") + fitness);
				EndEpisode();
			}
		
		}
		
		
	}

    // Update is called once per frame
    void FixedUpdate()
    {
		//RunJoints(torques);
		//refSkeleton.leftFemur.rotation = director.refSkeleton.leftFemur.rotation;
		curTime += Time.deltaTime;
		
		if(mimic) {
			MimicSkeleton(transform.parent.GetComponent<Director>().refSkeleton);
		}
		
		
	
    }
}
