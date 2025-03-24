using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PathMaker : MonoBehaviour
{
	
	public class Waypoint {
		public Vector2 pos;
		public bool light;
		public bool checkWater;
		public bool aimOnly;
		public bool pickFruit;
		public GameObject plant;
	};
	
	public static PathMaker Instance;
	public Actor actor;
	public DebugRover rover;
	public RoverControls roverControls;
	
	public string currentLocationName;
	
	[System.Serializable]
	public class Subscene {
		public string name;
		public GameObject prefab;
	};
	
	public List<Subscene> subscenes;
	
	public List<Waypoint> waypoints;
	//public List<bool> lightPoints;
	public bool loaded;
	public GameObject waypointPrefab;
	public GameObject currentWayPointObj;
	//public bool lightWayPoint;
	
	public Vector2 mapCoords;
	
	public string mqttIp = "127.0.0.1";
	
	public bool mapReady;
	public bool placedDown;
	
	public OnlineMaps map;
	
	public MainCam mainCam;
	public GameObject placeCamOrg;
	public GameObject coordMenu;
	public PosDisplay posDisplay;
	
	public Manip manip;
	public GameObject plotControls;
	
	public List<GameObject> cropList;
	
	public GameObject selectedCrop;
	public int selectedCropId;
	public RobotInfo selectedRobot;
	public bool humanoid;
	public bool useTractor;
	public GameObject tractorPrefab;
	public GameObject basketRoverPrefab;
	public HumanoidRobot humanoidRobot;
	public Tractor tractor;
	public FollowerRover basketRover;
	private float hSpawnTime = 0f;
	public GameObject currentRobot;
	
	public bool VR;
	
	
	public float weedDensity;
	public float xDensity;
	public float yDensity;
	
    // Start is called before the first frame update
    void Start()
    {
		
		Instance = this;
        waypoints = new List<Waypoint>();
		Instance.selectedRobot = GetComponent<SelectMenu>().robotPrefabs[0].GetComponent<RobotInfo>();
    }
	
	public Waypoint GetNextWaypoint(Vector3 pos) {
		Vector2 pos2 = new Vector2(pos.x,pos.z);
		int least = 0;
		for(int i = 0; i < waypoints.Count; i++) {
			
			if(Vector2.Distance(pos2, waypoints[i].pos) < Vector2.Distance(pos2, waypoints[least].pos)) {
				least = i;
			}
			
		}
		
		least = 0;
		
		Waypoint le = waypoints[least];
		if(currentWayPointObj != null) {
			Destroy(currentWayPointObj);
		}
		currentWayPointObj = Instantiate(waypointPrefab);
		currentWayPointObj.transform.position = new Vector3(le.pos.x,0,le.pos.y);
		waypoints.RemoveAt(least);
		
		return le;
	}
	
	public void PlaceDown(HumanoidRobot hr) {
		
		float initialHeight = 350f; // Start height for the raycast
		float stepHeight = 10f; // Step height for each progressive raycast
		int maxSteps = 50; // Maximum number of steps to attempt
		Vector3 rayPos = new Vector3(512, initialHeight, 512);
		RaycastHit hit;

		for (int i = 0; i < maxSteps; i++)
		{
			Debug.Log(rayPos);
			// Check if the raycast hits the terrain
			if (Physics.Raycast(rayPos, Vector3.down, out hit, stepHeight))
			{
				hr.transform.position = new Vector3(rayPos.x, hit.point.y + 1.4f, rayPos.z);
				//hr.GetComponent<Rigidbody>().isKinematic = false;
				
				//Spawn with a tractor
				if(useTractor) {
					if(tractor == null) {
						GameObject newTractor = Instantiate(tractorPrefab);
						tractor = newTractor.GetComponent<Tractor>();
					}
					tractor.transform.position = new Vector3(rayPos.x + 3, hit.point.y + 2.4f, rayPos.z + 3);
				}
				//Spawn without a tractor, use a basket rover instead
				else {
					if(basketRover == null) {
						GameObject newBask = Instantiate(basketRoverPrefab);
						basketRover = newBask.GetComponent<FollowerRover>();
					}
					basketRover.leader = hr.gameObject;
					basketRover.transform.position = new Vector3(rayPos.x + 3, hit.point.y + 1.2f, rayPos.z + 3);
					hr.basketRover = basketRover;
				}
				
				/*
				for(int j = 0; j < hr.joints.Length; j++) {
					hr.joints[j].rb.isKinematic = true;
					hr.joints[j].gameObject.transform.position += new Vector3(rayPos.x, hit.point.y + 1.4f, rayPos.z);
					hr.joints[j].rb.velocity = Vector3.zero;
					hr.joints[j].rb.angularVelocity = Vector3.zero;
					hr.joints[j].rb.isKinematic = false;
				}
				*/
				return;
			}
			// Decrease the raycast height for the next step
			rayPos.y -= stepHeight;
		}
		
	}
	
	public void LoadSubscene(string n) {
		currentLocationName = n;
		foreach(Subscene s in subscenes) {
			if(s.name == n) {
				GameObject newSubscene = Instantiate(s.prefab);
				newSubscene.transform.position = Vector3.zero;
			}
		}
	}
	
    // Update is called once per frame
    void Update()
    {
		
		if(humanoid) {
			if(mapReady && !placedDown) {
				if(hSpawnTime > 3f) {
					placedDown = true;
					PlaceDown(Instance.currentRobot.GetComponent<HumanoidRobot>());
				}
				if(!placedDown) {
					hSpawnTime += Time.deltaTime;
				}
			}
		}
		
		//Debug.Log(rStick);
		
		//Debug.Log(controls.Gameplay.rightStick.ReadValue<Vector2>());
		
		/*
		if(Input.GetKeyDown("c")) {
			selectedCropId += 1;
			if(selectedCropId >= cropList.Count) {
				selectedCropId = 0;
			}
		}
		*/
		if(Input.GetKeyDown("c")) {
			PlaceDown(PathMaker.Instance.currentRobot.GetComponent<HumanoidRobot>());
		}
		
        selectedCrop = cropList[selectedCropId];
    }
}
