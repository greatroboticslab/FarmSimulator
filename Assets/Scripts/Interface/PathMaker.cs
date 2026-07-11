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
	public AnimationLoader animationLoader;
	public Actor actor;
	public Actor trainingActor;
	public DebugRover rover;
	public RoverControls roverControls;
	
	public GameObject roverControlsANIM;
	public bool training;
	public bool useTerrain;
	public bool animationMode; //Use the generic tasks animation player mode
	public bool singlePlace; //Place a single object/crop instead of plots
	public GameObject placeMarker; //Blue placement marker used for placing objects
	public Director director;
	public GameObject selectMenuObj;
	public List<GameObject> tractionSpots;
	public Material ghostMaterial; //Material used for showing the guide/outline for placing items on the map
	
	public Transform selectMenuCameraPos;
	public Transform selectMenuCameraFocus;

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
	public GameObject activeSubscene;
	public MapInfo activeMapInfo;
	public GameObject emptyTerrain; //Blank plane used for training
	
	public MainCam mainCam;
	public GameObject placeCamOrg;
	public GameObject coordMenu;
	public PosDisplay posDisplay;
	public SelectMenu selectMenu;
	
	public Manip manip;
	public GameObject plotControls;
	
	public int cropCategory; //0 = Regular, 1 = VGGT
	public List<GameObject> cropList;
	public List<GameObject> vggtCropList;	

    public GameObject plotHuman; //Prefab of a human to spawn next to a placed plot
	public GameObject selectedCrop;
	public int selectedCropId;
	public int selectedVGGTCropId;
	public RobotInfo selectedRobot;
	public bool humanoid;
	public bool useTractor;
	public GameObject tractorPrefab;
	public GameObject basketRoverPrefab;
	public GameObject humanoidRobotPrefab;
	public HumanoidRobot humanoidRobot;
	public Tractor tractor;
	public FollowerRover basketRover;
	private float hSpawnTime = 0f;
	public GameObject currentRobot;
	public RobotConfig robotConfig;
	
	public bool VR;
	
	
	public float weedDensity;
	public float xDensity;
	public float yDensity;
	
    // Start is called before the first frame update
    void Start()
    {
		
		Instance = this;
        waypoints = new List<Waypoint>();
		SelectMenu menu = GetComponent<SelectMenu>();
		if(menu != null && menu.robotGroups.Count > 0 && menu.robotGroups[0].robotPrefabs.Count > 0 && menu.robotGroups[0].robotPrefabs[0] != null) {
			Instance.selectedRobot = menu.robotGroups[0].robotPrefabs[0].GetComponent<RobotInfo>();
		}
    }
	
	public Waypoint GetNextWaypoint(Vector3 pos) {
		if(waypoints == null || waypoints.Count == 0) {
			return new Waypoint { pos = new Vector2(pos.x, pos.z) };
		}
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

        Vector3 startPos = new Vector3(le.pos.x,120,le.pos.y);

		RaycastHit hit;

        if (Physics.Raycast(startPos, Vector3.down, out hit, 150.0f))
        {
            //Debug.Log("Hit!!!: " + hit.collider.name);
        }
        Vector3 endPos = hit.point;

		currentWayPointObj = Instantiate(waypointPrefab);
		currentWayPointObj.transform.position = endPos;
		waypoints.RemoveAt(least);
		
		return le;
	}

	public void FlipRobot() {
		if(rover) {
			rover.Flip();
		}
	}
	
	public void PlaceDown(HumanoidRobot hr) {
		
		float initialHeight = 350f; // Start height for the raycast
		float stepHeight = 10f; // Step height for each progressive raycast
		int maxSteps = 50; // Maximum number of steps to attempt
		Vector3 rayPos = new Vector3(512, initialHeight, 512);
		RaycastHit hit;

		for (int i = 0; i < maxSteps; i++)
		{
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
						hr.tractor = tractor;
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
					if(hr.training) {
						hr.joints[j].rb.isKinematic = false;
					}
				}
				*/
				return;
			}
			// Decrease the raycast height for the next step
			rayPos.y -= stepHeight;
		}
		
	}
	
	public void GoToMainMenu() {
		
		if(rover) {
			Destroy(rover.gameObject);
		}
		if(humanoidRobot) {
			Destroy(humanoidRobot.gameObject);
		}
		if(basketRover) {
			Destroy(basketRover.gameObject);
		}
		if(tractor) {
			Destroy(tractor.gameObject);
		}
		
		if(map && map.gameObject != activeSubscene) {
			Destroy(map.gameObject);
			
		}
		if(activeSubscene) {
			Destroy(activeSubscene);
		}
		if(emptyTerrain) {
			Destroy(emptyTerrain);
		}
		activeSubscene = null;
		activeMapInfo = null;
		map = null;
		emptyTerrain = null;
		currentRobot = null;
		
		if(manip != null && manip.cam != null) {
			manip.cam.mode = 3;
			manip.cam.targetPos = Vector3.zero;
			manip.cam.camPos = new Vector3(3,5,0);
			manip.cam.chaseCam = false;
			manip.cam.transform.position = new Vector3(3,5,0);
			manip.cam.currentFocusPoint = new Vector3(0,0,0);
			manip.started = false;
		}
		if(selectMenuObj != null) selectMenuObj.SetActive(true);
		if(trainingActor != null) trainingActor.gameObject.SetActive(false);
		mapReady = false;
		FindObjectOfType<RobotChat>()?.HideChat();
		placedDown = false;
		hSpawnTime = 0;
		if(roverControls != null) roverControls.gameObject.SetActive(false);
		
		
		if(actor != null && actor.anim != null) {
			actor.anim.SetInteger("gear", 0);
			actor.anim.SetBool("changeGear", false);
			actor.anim.SetBool("idle", true);
			actor.anim.SetBool("brake",false);
			actor.anim.SetFloat("speed",0);
			actor.anim.SetBool("tractor",false);
			actor.anim.SetBool("harvesting",false);
			actor.anim.SetFloat("turning",0);
		}
		
		
	}

	public void TogglePlaceMenu() {

		if(posDisplay != null) posDisplay.CropButtonPressed();
		if(manip != null) manip.placingPlot = !manip.placingPlot;

	}
	
	public void LoadSubscene(string n) {
		currentLocationName = n;
		if(subscenes == null || subscenes.Count == 0) {
			Debug.LogWarning("PathMaker: no subscenes configured for location '" + n + "'.");
			return;
		}

		foreach(Subscene s in subscenes) {
			if(s != null && s.name == n) {
				if(s.prefab == null) {
					Debug.LogWarning("PathMaker: subscene '" + n + "' has no prefab assigned.");
					return;
				}
				if(activeSubscene != null) {
					Destroy(activeSubscene);
				}
				if(emptyTerrain != null) {
					Destroy(emptyTerrain);
					emptyTerrain = null;
				}
				GameObject newSubscene = Instantiate(s.prefab);
				newSubscene.transform.position = Vector3.zero;
				activeSubscene = newSubscene;
				activeMapInfo = newSubscene.GetComponent<MapInfo>();
				if(activeMapInfo == null) activeMapInfo = newSubscene.GetComponentInChildren<MapInfo>();
				map = newSubscene.GetComponent<OnlineMaps>();
				if(map == null) map = newSubscene.GetComponentInChildren<OnlineMaps>();
				if(manip != null) manip.terrain = newSubscene;
				loaded = true;
				return;
			}
		}

		Debug.LogWarning("PathMaker: no subscene found for location '" + n + "'.");
	}

	public MapInfo GetActiveMapInfo() {
		if(activeMapInfo != null) return activeMapInfo;
		if(activeSubscene != null) {
			activeMapInfo = activeSubscene.GetComponent<MapInfo>();
			if(activeMapInfo == null) activeMapInfo = activeSubscene.GetComponentInChildren<MapInfo>();
			if(activeMapInfo != null) return activeMapInfo;
		}
		if(map != null) return map.GetComponent<MapInfo>();
		return null;
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
		if(Input.GetKeyDown("c") && PathMaker.Instance.currentRobot != null && PathMaker.Instance.currentRobot.GetComponent<HumanoidRobot>() != null) {
			PlaceDown(PathMaker.Instance.currentRobot.GetComponent<HumanoidRobot>());
		}
		
	/*
	if(cropCategory == 0) {
        	selectedCrop = cropList[selectedCropId];
	}
	if(cropCategory == 1) {
		selectedCrop = vggtCropList[selectedCropId];
	}
	*/
    }
}
