using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.UI;
using UnityEngine.EventSystems;
using TMPro;

public class Manip : MonoBehaviour
{

	public GameObject pointer;
	public SelectMenu selectMenu;
	public MainCam cam;
	public GameObject curRobot;
	public GameObject selectedTerrain;
	public GameObject terrain;
	public GameObject blankTerrainPrefab;
	public bool blankTerrain;
	public Camera botCam;
	public bool freeCam;
	public PosDisplay pd;
	
	public GameObject smallCamScreen;
	public GameObject fullCamScreen;
	private bool fullView;

	public InputDevice rightController;
	public InputDevice leftController;
	public InputDevice HMD;
	
	public GameObject testRob;
	
	public GameObject plotHelper;
	public GameObject plotHelperPrefab;
	public List<GameObject> plotHelpers;
	public bool placingPlot;
	public bool paintingTraction;
	public bool placingTraction;

	public Vector2 plotTopLeft;
	public Vector2 plotBottomRight;
	
	private bool placeTL;
	private bool placeBR;
	
	public bool quickStart;
	public GameObject defaultRobot;
	public bool started;
	
	public GameObject weedDensityDisplay;
	public float weedDensity;
	
	private bool validPlot;
	
	public RectTransform cropPanel;
	
	public GameObject placeGhost;
	
	public void UpdatePlacer(int m) {
		
		if(placeGhost != null) {
			Destroy(placeGhost);
		}
		placeGhost = new GameObject();
		placeGhost.AddComponent<MeshFilter>();
		placeGhost.GetComponent<MeshFilter>().mesh = PathMaker.Instance.tractionSpots[m].GetComponent<TractionSpot>().mesh;
		placeGhost.AddComponent<MeshRenderer>();
		placeGhost.GetComponent<MeshRenderer>().material = PathMaker.Instance.ghostMaterial;
		
	}
	
	public bool MouseInPanel() {
		
		return EventSystem.current.IsPointerOverGameObject();
	}

	//Place down plants in plots without exiting edit mode.
	public void PlacePlots() {
		Debug.Log("PLACING PLOTS");
		foreach(GameObject plot in plotHelpers) {
			plot.GetComponent<PlotHelper>().weedDensity = PathMaker.Instance.weedDensity;
			plot.GetComponent<PlotHelper>().PlacePlot();
		}
		while(plotHelpers.Count > 0) {
			Debug.Log("DELETING");
			GameObject p = plotHelpers[0];
			plotHelpers.RemoveAt(0);
			Destroy(p);
		}
	}
	
	public void EndEditMode() {
		
		PlacePlots();

		placingPlot = false;
		placingTraction = false;
		if(placeGhost) {
			Destroy(placeGhost);
		}
		cam.mode = 1;
		
	}

    // Start is called before the first frame update
    void Start()
    {
		
    }
	
	private void InitializeInputDevice(InputDeviceCharacteristics inputCharacteristics, ref InputDevice inputDevice) {
		
		List<InputDevice> devices = new List<InputDevice>();
		InputDevices.GetDevicesWithCharacteristics(inputCharacteristics, devices);
		
		if(devices.Count > 0) {
			inputDevice = devices[0];
		}
		
	}
	
	private void InitializeInputDevices() {
		
		if(!rightController.isValid) {
			InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Right, ref rightController);
		}
		if(!leftController.isValid) {
			InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.Left, ref leftController);
		}
		if(!rightController.isValid) {
			InitializeInputDevice(InputDeviceCharacteristics.Controller | InputDeviceCharacteristics.HeadMounted, ref HMD);
		}
		
	}
    
    public void StartRobot(RobotInfo robotInfo) {

		if(robotInfo.animMode) {
			PathMaker.Instance.animationMode = true;
			PathMaker.Instance.singlePlace = true;
		}

		if(PathMaker.Instance.animationMode) {
			PathMaker.Instance.roverControls.gameObject.SetActive(false);
			PathMaker.Instance.roverControlsANIM.SetActive(true);
		}

		cam.mode = 1;
		Cursor.lockState = CursorLockMode.None;
    	cam.chaseCam = robotInfo.chaseCam;
    	curRobot = Instantiate(robotInfo.robot);
		if(PathMaker.Instance.humanoid) {
			curRobot.GetComponent<HumanoidRobot>().actor = PathMaker.Instance.actor;
			PathMaker.Instance.humanoidRobot = curRobot.GetComponent<HumanoidRobot>();
			if(PathMaker.Instance.training) {
				PathMaker.Instance.director.gameObject.SetActive(true);
				PathMaker.Instance.director.actor.SetActive(true);
				PathMaker.Instance.humanoidRobot.training = true;
			}
		}
		
		PathMaker.Instance.selectMenuObj.SetActive(false);
		
		PathMaker.Instance.currentRobot = curRobot;

	if(PathMaker.Instance.humanoid) {
		PathMaker.Instance.robotConfig.humanoid = curRobot.GetComponent<HumanoidRobot>();
	}
	else {
		PathMaker.Instance.robotConfig.rover = curRobot.GetComponent<DebugRover>();
	}
	PathMaker.Instance.robotConfig.InitializeValues();
	PathMaker.Instance.robotConfig.sync = true;

    	curRobot.transform.position = Vector3.zero;
    	cam.camPos = new Vector3(0.4f,1,-0.4f);
    	cam.targetPos = new Vector3(0,1.0f,0);
    	cam.focusedRobot = curRobot;
    	if(robotInfo.terrain) {
    		terrain = Instantiate(selectedTerrain);
			
			if(!PathMaker.Instance.humanoid) {
				curRobot.GetComponent<DebugRover>().mapInfo = terrain.GetComponent<MapInfo>();
			}
			else {
				curRobot.GetComponent<HumanoidRobot>().mapInfo = terrain.GetComponent<MapInfo>();
			}
			curRobot.transform.position = terrain.GetComponent<MapInfo>().spawn.position;
			cam.camPos = terrain.GetComponent<MapInfo>().spawn.position;
    	}
		else {
			terrain = Instantiate(blankTerrainPrefab);
			PathMaker.Instance.emptyTerrain = terrain;
			curRobot.transform.position = new Vector3(0, 1.4f, 0);
		}
		if(!PathMaker.Instance.humanoid) {
			botCam = curRobot.GetComponent<DebugRover>().camera;
			pd.rover = curRobot.GetComponent<DebugRover>();
			if(!PathMaker.Instance.VR) {
				cam.transform.position = curRobot.GetComponent<DebugRover>().camOrg.transform.position;
			}
			else {
				cam.cc.enabled = false;
				cam.transform.position = curRobot.GetComponent<DebugRover>().camOrg.transform.position;
				cam.cc.enabled = true;
			}
			
			
			if(SocketInterace.Instance != null) {
				SocketInterace.Instance.Connect();
				SocketInterace.Instance.rover = curRobot.GetComponent<DebugRover>();
			}
		}
		PathMaker.Instance.placeCamOrg.transform.position = curRobot.transform.position + new Vector3(0,20,0);
    }

	public void UpdatePlaceMarker(Vector3 pos) {

		PathMaker.Instance.placeMarker.SetActive(true);
		PathMaker.Instance.placeMarker.transform.position = pos;

	}

    // Update is called once per frame
    void Update()
    {
		
	//Debug.Log(placingPlot);

		if(PathMaker.Instance != null) {
			if(!started && PathMaker.Instance.mapReady) {
				if(quickStart) {
					started = true;
					StartRobot(PathMaker.Instance.selectedRobot.GetComponent<RobotInfo>());
					if(PathMaker.Instance.useTerrain) {
						PathMaker.Instance.map = terrain.GetComponent<OnlineMaps>();
						PathMaker.Instance.map.SetPosition(PathMaker.Instance.mapCoords.x,PathMaker.Instance.mapCoords.y);
					}
				}
			}
		}
		
		
		//Placing traction spots
		if(placingTraction) {
			
			RaycastHit hit;
			Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
				if (Physics.Raycast (ray, out hit, 4000f)) {
					
					if(placeGhost) {
						placeGhost.transform.position = hit.point;
						
						if(Input.GetMouseButtonDown(0)) {
							
							if(!MouseInPanel()) {
								
								GameObject newTractionSpot = Instantiate(PathMaker.Instance.tractionSpots[PathMaker.Instance.roverControls.tractionDropdown.value]);
								newTractionSpot.transform.position = hit.point;
								
							}
						}
						
					}
					
				}
			
		}
		
		//Place plot
		if(Input.GetKeyDown("p")) {
			placingPlot = !placingPlot;
		}
		
		
		//Placing crop plots
		if(placingPlot) {
			
			if(Input.GetKeyDown("[+]")) {
				PathMaker.Instance.weedDensity += 0.5f;
			}
			if(Input.GetKeyDown("[-]")) {
				PathMaker.Instance.weedDensity -= 0.5f;
			}
			
			//weedDensityDisplay.SetActive(true);
			
			//weedDensityDisplay.GetComponent<TMP_Text>().text = "Weed Density: " + weedDensity;
			
			RaycastHit hit;
			Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
			if (Physics.Raycast (ray, out hit, 4000f)) {
				UpdatePlaceMarker(hit.point);
			}
			
			if(Input.GetMouseButtonDown(0)) {

				if(PathMaker.Instance.selectedCrop) {

                    if(PathMaker.Instance.singlePlace) {
                        if(!MouseInPanel()) {
                            GameObject newTractionSpot = Instantiate(PathMaker.Instance.selectedCrop);
                            newTractionSpot.transform.position = hit.point;
                        }
                    }

                    else {

                        if(!MouseInPanel()) {

                            validPlot = false;

                            //Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
                            //if (Physics.Raycast (ray, out hit, 4000f)) {

                                plotTopLeft = new Vector2(hit.point.x, hit.point.z);
                                plotHelper = Instantiate(plotHelperPrefab);
                                plotHelper.GetComponent<PlotHelper>().plant = PathMaker.Instance.selectedCrop;
                                plotHelpers.Add(plotHelper);
                                //placeTL = true;
                                placeBR = true;


                                validPlot = true;
                            //}
                        }

                    }
				}
				
				
			}
			if(Input.GetMouseButtonUp(0) && validPlot) {
				
				placeTL = false;
				placeBR = false;

			 	//Place plants as soon as mouse is let go
				PlacePlots();
				
			}
			
			if(placeBR && validPlot) {
				
				//Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
				if (Physics.Raycast (ray, out hit, 4000f)) {
					//Debug.Log (hit.transform.name);
					//Debug.Log ("hit");
				}
				
				plotBottomRight = new Vector2(hit.point.x, hit.point.z);
				//plotHelper = Instantiate(plotHelper);
				
			}
			
			if(plotHelper != null && validPlot) {
			
				plotHelper.transform.position = new Vector3((plotTopLeft.x+plotBottomRight.x)/2,500,(plotTopLeft.y+plotBottomRight.y)/2);
				plotHelper.GetComponent<PlotHelper>().size = new Vector2(Mathf.Abs(plotTopLeft.x-plotBottomRight.x)/2, Mathf.Abs(plotTopLeft.y-plotBottomRight.y)/2);
			
			}
			
			if(Input.GetKeyDown(KeyCode.Return)) {
				//Defunct
			}
			
		}
		else {
			PathMaker.Instance.placeMarker.SetActive(false);
		}
		
		
        
		if(!rightController.isValid || !leftController.isValid || !HMD.isValid) {
			InitializeInputDevices();
		}
		
		if(Input.GetKeyDown("[0]")) {
			ToggleView();
		}
		
        //Mouseover
		/*
        RaycastHit hit1;
		if (Physics.Raycast(pointer.transform.position, pointer.transform.TransformDirection(Vector3.forward), out hit1, Mathf.Infinity))
		{
			if(hit1.collider.gameObject.tag == "SelectableRobot") {
				hit1.collider.GetComponent<RobotInfo>().hovering = true;
			}
		}
		*/
        
        if (Input.GetMouseButtonDown(0) || OVRInput.Get(OVRInput.RawButton.RIndexTrigger)) {
		
			/*
			RaycastHit hit;
			if (Physics.Raycast(pointer.transform.position, pointer.transform.TransformDirection(Vector3.forward), out hit, Mathf.Infinity))
			{
				if(hit.collider.gameObject.tag == "SelectableRobot") {
						//Debug.Log("Did Hit");
						RobotInfo ri = hit.collider.GetComponent<RobotInfo>();
						StartRobot(ri);
						selectMenu.HideSelections();
						
						
					}
			}
			*/
			
			
        	
        }
		
        
        //Switch camera
		/*
        if(Input.GetKeyDown("space")) {
        	if(freeCam) {
        		freeCam = false;
        		cam.tag = "None";
        		botCam.tag = "MainCamera";
        		cam.enabled = false;
        		botCam.enabled = true;
        	}
        	else {
        		freeCam = true;
        		botCam.tag = "MainCamera";
        		cam.tag = "None";
        		cam.enabled = true;
        		botCam.enabled = false;
        	}
        }
		*/
        
    }
	
	public void ToggleView() {
		if(fullView) {
				fullView = false;
				fullCamScreen.SetActive(false);
		}
		else {
			fullView = true;
			fullCamScreen.SetActive(true);
			fullCamScreen.GetComponent<RectTransform>().sizeDelta = new Vector2(Screen.width,Screen.height);
		}
	}
}
