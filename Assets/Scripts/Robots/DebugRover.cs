using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
using UnityEngine.UI;
//using System.Time;

public class DebugRover : MonoBehaviour
{

	public string robotName; // Name of the robot to be displayed
	public int robotType; //0 = Moisture only, 1 = Weed killer, 2 = Grabber
	public Rigidbody rb;  //?
	public float moveSpeed = 4f; //move setting
	public float turnSpeed; //turn speed setting
	public Camera camera;   //camera class
	public RobotCamera rCam; // robot camera class seperate from camera class?
	public bool recording; // evaluates whether or not its recording
	public string curRecordDir;  // current recording directory
	public int frame; //frame number
	public MapInfo mapInfo; //class map info
	public MqNode mqNode; //connects robot components together 
	public float longitude; //latitude coordinates
	public float latitude; // longitude coordinates
	public float heading; // compass reading
	public float vel; // velocity 
	public float charge;
	public float powerUsage;
	public List<AxleInfo> axles; //list of axles
	public List<RotateWheel> wheelRot;
	public PathMaker.Waypoint currentWaypoint;   //attribute of pathmaker attribute named current waypoint 
	public Vector2 currentWaypointLATLNG; // vector of current lat/lang
	public GameObject camOrg; //?
	public GameObject camOrgCenter; // Center point that camOrg orbits around
	public bool wantsToTeleport;
	public RobotArm robotArm;
	public int selectedJoint = -1;
	public Vector2 rStick;
	public float lTrigger;
	public float rTrigger;
	public PlayerControls controls;
	public bool armMode;
	public float currentFriction = 1f;
	public float frictionMultiplier = 1f;
	public bool stuck;
	
	public bool selfDriving;
	private float turnInput;
	private float forwardInput;
	private float probeInput;
	
	public bool netControlled;
	public float netForwardInput;
	public float netTurnInput;
	public float netLightInput;
	
	public float minDistance = 0.19f;
	public float checkDistance = 0.2f;
	public float closeSpeed = 0.3f;
	public float cruiseSpeed = 5f;
	public float headingThreshold = 1.2f; //If less than this value, rover is assumed to be facing the correct direction.
	public float harvestTimeLimit = 20f; //Max amount of time the rover will spend attempting to harvest a single fruit before skipping it.
	

	private float maxLightTime = 5f;
	private float lightTime;
	
	public float torqueVal = 5f;
	
	public GameObject uvLight;
	public bool lightOn;
	
	private bool oneLineRecord = false;
	private StreamWriter infoFile;
	private bool gotWaypoint;
	
	public Text outGui;
	
	public float recordSpeed;
	public bool scienceQA;
	private float recordTime;
	
	private bool triggerDown;
	
	private string dataHeader = "Forward, Turn, Light, Latitude, Longitude, Heading, Velocity, WaypointLatitude, WaypointLongitude\n";
	private string lmHeader = "Data:\n";
	
	public int lightMode = 1; //0 = Shine light when close to a waypoint, 1 = shine light along path to next node
	
	public bool blocked;
	private float blockTime;
	private bool _prevSelfDriving;
	
	public Transform probe;
	public float measuredWater;
	public float timeSinceMeasure;
	public Basket basket;
	public Transform flipPoint;
	private float timeSinceFlip;
	
	public void Flip() {
		
		timeSinceFlip = 0;
		//transform.rotation = Quaternion.identity;
		rb.rotation = Quaternion.identity;
		rb.velocity = Vector3.zero;
		rb.angularVelocity = Vector3.zero;
		rb.position += new Vector3(0,1,0);
		
	}

	public void FlipCheck() {
		
		if(!flipPoint) {
			GameObject nF = new GameObject("FlipPoint");
			flipPoint = nF.transform;
			flipPoint.SetParent(transform);
			flipPoint.localPosition = new Vector3(0,2,0);
		}

		if(flipPoint.position.y < transform.position.y && timeSinceFlip > 5) {
			
			Flip();
			
		}
		timeSinceFlip += Time.deltaTime;

		
	}

	//Used for using unity as a "display," where the physical gps will update the virtual rover's position, rotation, etc
	public void Sync(float _longitude, float _latitude, float _heading) {
		
		Vector2 _dPos = mapInfo.CoordsToPos(new Vector2(_latitude, _longitude));
		transform.position = new Vector3(_dPos.x, transform.position.y, _dPos.y);
		
	}
	
	public void TestPlant() {
		RaycastHit hit;
		//instantiates a raycast object
		timeSinceMeasure = 0;
		
		int layer = 6;
		//
		
		int layerMask = 1 << layer;   // means take 1 and rotate it left by "layer" bit positions
		
		//layerMask = ~layerMask;
		
		if (Physics.Raycast(probe.position, transform.TransformDirection(probe.forward), out hit, 10, layerMask))
		{
			Plant p = hit.collider.gameObject.GetComponent<Plant>();
			measuredWater = p.water;
			Debug.Log(measuredWater);
			
		}
	}
	
	//Returns a string of the current information about the rover, which is writtern to a frame .txt file
	//If scienceQA is true, it will be output in the form of a series of sentences about the rovers information
	//Otherwise, the rovers current inputs, lat, lng, etc will be output as a series of plaintext numbers.
	private string GetInfo(string delimeter) {
		
		if(scienceQA) {
			
			
			Vector2 lp = new Vector2(Mathf.Infinity, Mathf.Infinity);
			if(currentWaypoint != null) {
				lp = mapInfo.PosToCoords(new Vector3(currentWaypoint.pos.x, 0, currentWaypoint.pos.y));
			}
			
			string question = "Question: What actions should the rover perform given the current image and situation?";
			
			string context = "Context: ";
			context += "The current latitude is " + latitude.ToString("F13") + " degrees, and the longitude is " + longitude.ToString("F13") + " degrees.";
			context += " The current heading is " + heading.ToString("F13") + " degrees, with a velocity of " + vel + " meters per second.";
			context += " The waypoints latitude is " + lp.y.ToString("F13") + " degrees, and the longitude of the waypoint is " + lp.x.ToString("F13") + " degrees.";
			
			string answer = "Answer: ";
			if(forwardInput > 0) {
				answer += "Accelerate forward. ";
			}
			if(forwardInput < 0) {
				answer += "Accelerate backward. ";
			}
			if(turnInput > 0) {
				answer += "Turn left. ";
			}
			if(turnInput < 0) {
				answer += "Turn right. ";
			}
			if(lightOn) {
				answer += "Turn light on.";
			}
			else {
				answer += "Turn light off.";
			}
			
			string explanation = "Explanation: ";
			if(forwardInput > 0) {
				explanation += "We accelerate forward, because the rover is going slower than either the cruise speed, or the speed when it is close to the waypoint.";
			}
			else if(forwardInput < 0) {
				explanation += "We accelerate backwards, because the rover is going faster than the cruise speed or speed when it is close to the waypoint. ";
			}
			else {
				explanation += "The rover is going faster than the desired speed, thus it does not accelerate.";
			}
			if(turnInput > 0) {
				explanation += " The rover is turning left, because the rover is not facing the waypoint, and the waypoint is to the rovers left.";
			}
			else if(turnInput < 0) {
				explanation += " The rover is turning right, because the rover is not facing the waypoint, and the waypoint is to the rovers right.";
			}
			else {
				explanation += " The rover is not turning, because it is currently facing the waypoint.";
			}
			
			if(lightOn) {
				explanation += " The rover turns its light on, because it is driving in between the rows of soybean, or other crop.";
			}
			else {
				explanation += " The rover turns its light off, because it is not driving in between the rows of soybean, or other crop.";
			}
			
			return context + '\n' + question + '\n' + answer + '\n' + explanation;
			
		}
		else {
		
			int l = 0;
			if(lightOn) {
				l = 1;
			}
			
			Vector2 lp = new Vector2(Mathf.Infinity, Mathf.Infinity);
			if(currentWaypoint != null) {
				lp = mapInfo.PosToCoords(new Vector3(currentWaypoint.pos.x, 0, currentWaypoint.pos.y));
			}
			currentWaypointLATLNG = lp;
			
			//return "" + Input.GetAxis("Vertical") + delimeter + Input.GetAxis("Horizontal") + delimeter + l + delimeter + latitude.ToString("F13") + delimeter + longitude.ToString("F13") + delimeter + heading.ToString("F13") + delimeter + vel;
			return "" + forwardInput + delimeter + turnInput + delimeter + l + delimeter + latitude.ToString("F13") + delimeter + longitude.ToString("F13") + delimeter + heading.ToString("F13") + delimeter + vel
			+ delimeter + lp.y.ToString("F13") + delimeter + lp.x.ToString("F13")
			;
		}
		return "Error getting input string.";
		
	}
	
	//Defunct self drive function
	private void SelfDriveUpdate() {
		
		forwardInput = 0;
		turnInput = 0;
		lightOn = false;
		
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
			
			if(heading < targetHeading) {
				turnInput = 1;
			}
			if(heading > targetHeading) {
				turnInput = -1;
			}
			
			bool cruising = false;
			
			if(Vector2.Distance(rPos, currentWaypoint.pos) > 310) {
				cruising = true;
			}
			
			deltaHeading = Mathf.Abs(heading - targetHeading)/180;
			
			float closeMult = (Vector2.Distance(rPos, currentWaypoint.pos)/(minDistance*3))*5000*(deltaHeading/(Vector2.Distance(rPos, currentWaypoint.pos)))*
						(rb.velocity.magnitude + 0.03f);
			
			var localVelocity = transform.InverseTransformDirection(rb.velocity);
			
			float forwardSpeed = localVelocity.z;
			
			if(Vector2.Distance(rPos, currentWaypoint.pos) > minDistance) {
				
				if(cruising) {
					if(forwardSpeed < cruiseSpeed) {
						forwardInput = 1;
					}
				}
				else {
					if(forwardSpeed < closeSpeed) {
						
						float mult = closeMult;
						if(mult > 1) {
							mult = 1;
						}
						
						forwardInput = 1*mult;
					}
				}
			}
			else {
				
				currentWaypoint = PathMaker.Instance.GetNextWaypoint(transform.position);
				
				float mult = closeMult;
				if(mult > 1) {
					mult = 1;
				}
				//forwardInput = -1  * mult;
			}
			
			if(Vector2.Distance(rPos, currentWaypoint.pos) < minDistance+1.5f) {
				if(Mathf.Abs(heading - targetHeading) < 10) {
					lightOn = true;
				}
			}
			
			if(forwardSpeed < 0) {
				//turnInput *= -1;
			}
			
			deltaHeading = Mathf.Abs(heading - targetHeading)/30;
			if(deltaHeading > 1) {
				deltaHeading = 1;
			}
			turnInput *= deltaHeading;
			
			if(blocked) {
				blockTime = 3.5f;
			}
			
			if(blockTime > 0) {
				forwardInput *= -0.35f;
				turnInput = .35f;
				
				blockTime -= Time.deltaTime;
				
			}
			
			forwardInput = Mathf.Abs(forwardInput);
			
			//Debug.Log("" + heading + ", " + targetHeading + " | " + forwardInput + ", " + turnInput);
			
		}
		
	}
	
	//This function is performed every frame if the rover is set to drive to waypoints of its own volition in the simulation.
	private void SelfDriveUpdate_2() {
		
		if(netControlled) {
			forwardInput = netForwardInput;
			turnInput = netTurnInput;
			
			if(forwardInput > 1) {
				forwardInput = 1;
			}
			if(forwardInput < -1) {
				forwardInput = -1;
			}
			
			if(turnInput > 1) {
				turnInput = 1;
			}
			if(turnInput < -1) {
				turnInput = -1;
			}
			
			if(netLightInput >= 0.5f) {
				lightOn = true;
			}
			else {
				lightOn = false;
			}
		}
		else {
			
			forwardInput = 0;
			turnInput = 0;
			lightOn = false;
			
			bool _stop = false;
			if(robotArm != null) {
				if(robotArm.harvesting) {
					_stop = true;
					if(robotArm.timeHarvesting > harvestTimeLimit) {
						robotArm.SkipFruit();
					}
				}
			}
			
			if(!_stop) {
				
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
					
					
					if(currentWaypoint.light) {
						lightOn = true;
					}

					//UV robot
					//if(robotType == 1 && !currentWaypoint.pickFruit) {
					//	lightOn = true;
					//}
					
					var localVelocity = transform.InverseTransformDirection(rb.velocity);
						
					float forwardSpeed = localVelocity.z;
					
					
					
					bool reached = false;
					if(currentWaypoint.checkWater) {
						if(currentWaypoint.aimOnly) {
							if(Mathf.Abs(deltaHeading) <= headingThreshold) {
								reached = true;
							}
						}
						else {
							if(Vector3.Distance(rPos, currentWaypoint.pos) < checkDistance) {
								if(Mathf.Abs(deltaHeading) <= headingThreshold) {
									reached = true;
								}
							}
						}
					}
					else if(currentWaypoint.pickFruit) {
						if(Vector3.Distance(rPos, currentWaypoint.pos) < 2.5f) {
							//if(Mathf.Abs(deltaHeading) <= headingThreshold) {
								reached = true;
							//}
						}
					}
					else {
						if(Vector3.Distance(rPos, currentWaypoint.pos) < minDistance) {
							reached = true;
						}
					}
					
					
					//Reached Waypoint
					if(reached) {
						Debug.Log("[DebugRover] Waypoint reached at " + currentWaypoint.pos + ". Getting next waypoint.");
						if(currentWaypoint.checkWater) {
							TestPlant();
						}
						if(currentWaypoint.pickFruit) {
							robotArm.HarvestCrop(currentWaypoint.plant.GetComponent<Plant>());
						}
						currentWaypoint = PathMaker.Instance.GetNextWaypoint(transform.position);
					}
					
					
					
					
					if(Mathf.Abs(deltaHeading) > headingThreshold) {
						
						if(heading < targetHeading) {
							turnInput = 1;
						}
						if(heading > targetHeading) {
							turnInput = -1;
						}
						
						if(forwardSpeed > 0) {
							forwardInput = -1;
						}
						if(forwardSpeed < 0) {
							forwardInput = 1;
						}
						
					}
					else {
						
						
						if(forwardSpeed < closeSpeed) {
							forwardInput = 1;
						}
					}
				}
			}
			
			if(blocked) {
				blockTime = 3.5f;
			}
			
			if(blockTime > 0) {
				forwardInput = -0.35f;
				turnInput = .35f;
				
				blockTime -= Time.deltaTime;
				
			}
		}
		
	}
	
	private int placeAttempts = 0;

	private void PlaceDown()
	{
		//Prefer the map's designated spawn point; fall back to wherever the rover is now
		Vector3 target = transform.position;
		if (mapInfo != null && mapInfo.spawn != null)
		{
			target = mapInfo.spawn.position;
		}

		if (FindGroundHeight(target, transform, out float groundY))
		{
			transform.position = new Vector3(target.x, groundY + 1.4f, target.z);
			rb.position = transform.position;
			rb.velocity = Vector3.zero;
			rb.angularVelocity = Vector3.zero;
			return;
		}

		//The farm terrain may still be loading; retry next frame instead of giving up
		if (placeAttempts < 300)
		{
			placeAttempts++;
			placed = false;
			return;
		}
		Debug.LogWarning("DebugRover: no ground found under spawn point " + target + "; leaving rover at " + transform.position);
	}

	//Casts down from high above (x,z) and returns the highest hit that is not part of ignoreRoot
	public static bool FindGroundHeight(Vector3 target, Transform ignoreRoot, out float groundY)
	{
		groundY = target.y;
		RaycastHit[] hits = Physics.RaycastAll(new Vector3(target.x, 1000f, target.z), Vector3.down, 2000f, ~0, QueryTriggerInteraction.Ignore);
		bool found = false;
		foreach (RaycastHit h in hits)
		{
			if (ignoreRoot != null && h.collider.transform.IsChildOf(ignoreRoot)) continue;
			if (!found || h.point.y > groundY)
			{
				groundY = h.point.y;
				found = true;
			}
		}
		return found;
	}
	private float timeSpawned = 0;
	private bool placed = false;
	
    // Start is called before the first frame update
    void Start()
    {
		
		wheelRot = new List<RotateWheel>();
		foreach(AxleInfo axle in axles) {
			wheelRot.Add(axle.wheels[0].GetComponent<RotateWheel>());
			wheelRot.Add(axle.wheels[1].GetComponent<RotateWheel>());
		}
		
		controls = new PlayerControls();
		controls.Gameplay.Enable();
		
		PathMaker.Instance.rover = this;
		PathMaker.Instance.currentRobot = gameObject;
		
		if(mqNode != null) {
			mqNode.PublishInfo(forwardInput, turnInput, lightOn);
		}
        rb = GetComponent<Rigidbody>();
		
    }

	//Do things related to traction / slipperyness of map
	void TractionUpdate() {
		
		float stuckFactor = 0;
		
		//Debug.Log(mapInfo.tractionZone.GetTraction(transform.position));
		if(mapInfo != null && mapInfo.tractionZone != null) {
			currentFriction = mapInfo.tractionZone.GetTraction(transform.position);
		}
		else {
			currentFriction = 1f;
		}
		//Debug.Log(mapInfo.tractionZone.tractionGrid[0,0]);
		
		float rayDistance = 1.7f;
		int layer17Mask = 1 << 17;
		
		Ray ray = new Ray(transform.position + new Vector3(0f,0.9f,0f), Vector3.down);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, rayDistance, layer17Mask))
        {
			//Debug.Log("Traction Zone!");
			TractionSpot tractionSpot = hit.collider.gameObject.GetComponent<TractionSpot>();
			if(tractionSpot != null) {
				currentFriction = tractionSpot.traction;
				stuckFactor = tractionSpot.stuckFactor;
			}
		}
		float stuckScore = Mathf.Abs(forwardInput) * stuckFactor * 0.01f;
		if(Random.value < stuckScore) {
			stuck = true;
		}
		
		foreach(RotateWheel w in wheelRot) {
			if(w == null || w.col == null) continue;
			WheelFrictionCurve ff = w.col.forwardFriction;
			ff.stiffness = w.initialStiffness[0] * currentFriction * frictionMultiplier;
			if(stuck) {
				ff.stiffness = 0;
			}
			w.col.forwardFriction = ff;
			
			WheelFrictionCurve sf = w.col.sidewaysFriction;
			sf.stiffness = w.initialStiffness[1] * currentFriction * frictionMultiplier;
			if(stuck) {
				sf.stiffness = 0;
			}
			w.col.sidewaysFriction = sf;
			
		}
		
	}

    // Update is called once per frame
    void Update()
    {
		
		FlipCheck();
		TractionUpdate();
		
		
		controls.Gameplay.toggleArm.performed += ctx => armMode = !armMode;
		
		controls.Gameplay.rightStick.performed += ctx => rStick = ctx.ReadValue<Vector2>();
		controls.Gameplay.rightStick.canceled += ctx => rStick = Vector2.zero;
		
		controls.Gameplay.rightTrigger.performed += ctx => rTrigger = ctx.ReadValue<float>();
		controls.Gameplay.rightTrigger.canceled += ctx => rTrigger = 0f;
		
		controls.Gameplay.leftTrigger.performed += ctx => lTrigger = ctx.ReadValue<float>();
		controls.Gameplay.leftTrigger.canceled += ctx => lTrigger = 0f;








		if(wantsToTeleport) {
			
			if(PathMaker.Instance.waypoints.Count > 0) {
			
				wantsToTeleport = false;
				//Debug.Log("TELEPORTING...");



				rb.isKinematic = true;
				Vector2 mDir = PathMaker.Instance.waypoints[0].pos - PathMaker.Instance.waypoints[1].pos;
				Vector2 tPos = PathMaker.Instance.waypoints[0].pos;
				tPos += mDir;

				Vector3 startPos = new Vector3(tPos.x, transform.position.y + 20.0f, tPos.y);

                Debug.Log("Start Pos: " + startPos);

				if(robotType == 1) {
				    startPos += new Vector3(4,0,4);
				}

				RaycastHit hit;

                if (Physics.Raycast(startPos, Vector3.down, out hit, 150.0f))
                {
                    Debug.Log("Hit!!!: " + hit.collider.name);
                }
                Vector3 endPos = hit.point + new Vector3(0,1,0);

				transform.position = endPos;
				rb.isKinematic = false;
				rb.MovePosition(endPos);
				
			}
		}
		
		//Debug.Log("Connected: " + mClient.mqttClientConnected);
		
		timeSinceMeasure += Time.deltaTime;
		
		if(currentWaypoint != null) {
			currentWaypointLATLNG = mapInfo.PosToCoords(new Vector3(currentWaypoint.pos.x, 0, currentWaypoint.pos.y));
		}
		else {
			currentWaypointLATLNG = new Vector2(0,0);
		}
		
		if(timeSpawned >= 1.4f) {
			if(!placed) {
				placed = true;
				PlaceDown();
			}
		}
		else {
			timeSpawned += Time.deltaTime;
			//Debug.Log("TS: " + timeSpawned);
		}
		
		if(!gotWaypoint) {
			if(PathMaker.Instance.loaded) {
				gotWaypoint = true;
				currentWaypoint = PathMaker.Instance.GetNextWaypoint(transform.position);
			}
		}
		
		
		
		charge -= (Time.deltaTime*powerUsage);
		
		vel = rb.velocity.magnitude;
        //float forwardF = Time.deltaTime * moveSpeed * Input.GetAxis("Vertical");
        //float sideF = Time.deltaTime * turnSpeed * Input.GetAxis("Horizontal");
		
		

		heading = transform.eulerAngles.y;
	
		powerUsage = 0;
		powerUsage += (Mathf.Abs(forwardInput) * Time.deltaTime * 0.5f);
		if(lightOn) {
			powerUsage += Time.deltaTime * 2.7f;
		}
        
		/*
        rb.AddForce(transform.forward * forwardF);
        //rb.AddForce(transform.right * sideF);
        rb.AddTorque(transform.up * sideF);
    	*/

		if(Input.GetMouseButtonDown(0) || 
		(!triggerDown && OVRInput.Get(OVRInput.RawButton.RIndexTrigger))) {
			triggerDown = true;
			if(lightOn) {
				lightOn = false;
				//uvLight.SetActive(false);
			}
			else {
				lightOn = true;
				//uvLight.SetActive(true);
			}
		}
		
		uvLight.SetActive(lightOn);
		
		
		if(!OVRInput.Get(OVRInput.RawButton.RIndexTrigger)) {
			triggerDown = false;
		}
		
		if(Input.GetKeyDown("[3]")) {
			Sync(30,30,0);
		}
		
    	if(Input.GetKeyDown("[5]")) {
    		if(recording) {
    			StopRecording();
				
    		}
    		else {
    			StartRecording();
    		}
    	}
    	
		
		/*
    	if(recording) {
		if(recordTime >= 1f/recordSpeed) {
    			rCam.CaptureFrame(curRecordDir);
				
				if(oneLineRecord) {
					infoFile.Write(GetInfo(", ")+'\n');
				}
				else {
					//Debug.Log(Application.dataPath + "/Recordings/" + curRecordDir);
					string moveOutput = GetInfo(", ");
					//File.WriteAll(Application.dataPath + "/Recordings/" + curRecordDir + "/" + frame + ".txt", moveOutput);
					
					StreamWriter writer = new StreamWriter(Application.dataPath + "/Recordings/" + curRecordDir + "/" + frame + ".txt", true);
					
					writer.Write(dataHeader);
					writer.Write(moveOutput);
					
					writer.Close();
				}
				frame++;
    			recordTime = 0;
		}
		recordTime += Time.deltaTime;
    	}
		*/
    	
    	if(mapInfo != null) {
    		//Debug.Log(mapInfo.PosToCoords(transform.position));
    		Vector2 lp = mapInfo.PosToCoords(transform.position);
    		longitude = lp.x;
    		latitude = lp.y;
    	}
	
		
		if(mqNode != null) {
			mqNode.PublishInfo(forwardInput, turnInput, lightOn);
		}
    
    }
	
	public void StartRecording() {
		string hdr = dataHeader;
		if(scienceQA) {
			hdr = lmHeader;
		}
		
		recording = true;
				
		curRecordDir = System.DateTime.Now.ToString("yyyy_MM_ddTHH_mm_ss", System.Globalization.CultureInfo.InvariantCulture);
		
		Directory.CreateDirectory(Application.dataPath + "/Recordings/" + curRecordDir);
		
		if(oneLineRecord) {
			infoFile = new StreamWriter(Application.dataPath + "/Recordings/" + curRecordDir + "/" + "posInfo" + ".txt", true);
			infoFile.Write(hdr);
		}
	}
	public void StopRecording() {
		recording = false;
		if(infoFile != null) {
			infoFile.Close();
		}
	}
	
	public void ManualDriveUpdate() {
		
		//Robot arm controls
		selectedJoint = -1;
		if(robotArm != null) {
			for(int i = 0; i < 9; i++) {
				string c = (i+1).ToString();
				if(i < robotArm.joints.Count) {
					if(Input.GetKey(c)) {
						selectedJoint = i;
					}
				}
			}
		}
		
		
		if(PathMaker.Instance.VR) {
			forwardInput = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick).y;
			turnInput = OVRInput.Get(OVRInput.Axis2D.SecondaryThumbstick).x;
		}
		else {
			
			if(armMode) {
				
				robotArm.joints[6].input = -lTrigger + rTrigger;
				
				robotArm.joints[1].input = -rStick.x;
				
				robotArm.joints[0].input = Input.GetAxis("Horizontal");
				robotArm.joints[2].input = Input.GetAxis("Vertical");
				
				robotArm.joints[3].input = rStick.y;
				//robotArm.joints[1].input = rStick.x;
				
				forwardInput = turnInput = 0f;
				
			}
			else {
			
				if(selectedJoint == -1) {
					forwardInput = Input.GetAxis("Vertical");
					turnInput = Input.GetAxis("Horizontal");
				}
				else {
					robotArm.joints[selectedJoint].input = Input.GetAxis("Vertical");
					forwardInput = turnInput = 0f;
				}
			}
		}
		
	}
	
	public void FixedUpdate() {

		if(selfDriving) {
			//SelfDriveUpdate();
			SelfDriveUpdate_2();
		}
		else {
			ManualDriveUpdate();
		}

		if(selfDriving && !_prevSelfDriving)
		{
			Debug.Log("[DebugRover] Self-driving STARTED. currentWaypoint=" +
				(currentWaypoint == null ? "null (no plot placed?)" : currentWaypoint.pos.ToString()) +
				", loaded=" + PathMaker.Instance.loaded +
				", waypointCount=" + PathMaker.Instance.waypoints.Count);
		}
		_prevSelfDriving = selfDriving;
		
		float leftInput = forwardInput;
		float rightInput = forwardInput;
		
		//Debug.Log(forwardInput);
		
		leftInput += turnInput*1f;
		rightInput -= turnInput*1f;
		
		if(leftInput > 1) {
			leftInput = 1f;
		}
		if(rightInput > 1) {
			rightInput = 1f;
		}
		
		
		if(leftInput < -1) {
			leftInput = -1f;
		}
		if(rightInput < -1) {
			rightInput = -1f;
		}
		
		//Debug.Log("Forward " + forwardInput + "| Turn: " + leftInput + ", " + rightInput);
		
		//Wheels
		foreach(AxleInfo axle in axles) {
			axle.wheels[0].motorTorque = leftInput * moveSpeed;
			axle.wheels[1].motorTorque = rightInput * moveSpeed;
		}
		
		if(recording) {
			if(recordTime >= 1f/recordSpeed) {
					rCam.CaptureFrame(curRecordDir);
					
					if(oneLineRecord) {
						infoFile.Write(GetInfo(", ")+'\n');
					}
					else {
						
						string hdr = dataHeader;
						if(scienceQA) {
							hdr = lmHeader;
						}
						
						string moveOutput = GetInfo(", ");
						
						StreamWriter writer = new StreamWriter(Application.dataPath + "/Recordings/" + curRecordDir + "/" + frame + ".txt", true);
						
						writer.Write(hdr);
						writer.Write(moveOutput);
						
						writer.Close();
					}
					frame++;
					recordTime = 0;
			}
			recordTime += Time.deltaTime;
    	}
		
	}

}
