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
		if(PathMaker.Instance == null || PathMaker.Instance.tractionSpots == null || m < 0 || m >= PathMaker.Instance.tractionSpots.Count) return;
		GameObject tractionSpot = PathMaker.Instance.tractionSpots[m];
		if(tractionSpot == null || tractionSpot.GetComponent<TractionSpot>() == null) return;
		
		if(placeGhost != null) {
			Destroy(placeGhost);
		}
		placeGhost = new GameObject();
		placeGhost.AddComponent<MeshFilter>();
		placeGhost.GetComponent<MeshFilter>().mesh = tractionSpot.GetComponent<TractionSpot>().mesh;
		placeGhost.AddComponent<MeshRenderer>();
		placeGhost.GetComponent<MeshRenderer>().material = PathMaker.Instance.ghostMaterial;
		
	}
	
	public bool MouseInPanel() {
		
		return EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();
	}

	//Place down plants in plots without exiting edit mode.
	public void PlacePlots() {
		if(plotHelpers == null) return;
		foreach(GameObject plot in plotHelpers) {
			if(plot == null) continue;
			PlotHelper helper = plot.GetComponent<PlotHelper>();
			if(helper == null) continue;
			helper.weedDensity = PathMaker.Instance != null ? PathMaker.Instance.weedDensity : 0;
			helper.PlacePlot();
		}
		while(plotHelpers.Count > 0) {
			GameObject p = plotHelpers[0];
			plotHelpers.RemoveAt(0);
			if(p != null) Destroy(p);
		}
	}
	
	public void EndEditMode() {
		
		PlacePlots();

		placingPlot = false;
		placingTraction = false;
		if(placeGhost) {
			Destroy(placeGhost);
		}
		if(cam != null) cam.mode = 1;
		
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
		if(robotInfo == null || robotInfo.robot == null || PathMaker.Instance == null) return;

		PathMaker.Instance.selectedRobot = robotInfo;
		PathMaker.Instance.humanoid = robotInfo.humanoid;
		PathMaker.Instance.training = robotInfo.training;
		PathMaker.Instance.useTractor = robotInfo.includeTractor;

		if(robotInfo.animMode) {
			PathMaker.Instance.animationMode = true;
			PathMaker.Instance.singlePlace = true;
		}

		if(PathMaker.Instance.animationMode) {
			if(PathMaker.Instance.roverControls != null) PathMaker.Instance.roverControls.gameObject.SetActive(false);
			if(PathMaker.Instance.roverControlsANIM != null) PathMaker.Instance.roverControlsANIM.SetActive(true);
		}

		if(cam != null) cam.mode = 1;
		Cursor.lockState = CursorLockMode.None;
		if(cam != null) cam.chaseCam = robotInfo.chaseCam;
		curRobot = Instantiate(robotInfo.robot);
		if(PathMaker.Instance.humanoid) {
			HumanoidRobot humanoidComponent = curRobot.GetComponent<HumanoidRobot>();
			if(humanoidComponent != null) humanoidComponent.actor = PathMaker.Instance.actor;
			PathMaker.Instance.humanoidRobot = humanoidComponent;
			if(PathMaker.Instance.training) {
				if(PathMaker.Instance.director != null) {
					PathMaker.Instance.director.gameObject.SetActive(true);
					if(PathMaker.Instance.director.actor != null) PathMaker.Instance.director.actor.SetActive(true);
				}
				if(PathMaker.Instance.humanoidRobot != null) PathMaker.Instance.humanoidRobot.training = true;
			}
		}
		
		if(PathMaker.Instance.selectMenuObj != null) PathMaker.Instance.selectMenuObj.SetActive(false);
		
		PathMaker.Instance.currentRobot = curRobot;

		if(PathMaker.Instance.robotConfig != null) {
			if(PathMaker.Instance.humanoid) {
				PathMaker.Instance.robotConfig.humanoid = curRobot.GetComponent<HumanoidRobot>();
			}
			else {
				PathMaker.Instance.robotConfig.rover = curRobot.GetComponent<DebugRover>();
			}
			PathMaker.Instance.robotConfig.InitializeValues();
			PathMaker.Instance.robotConfig.sync = true;
		}

		curRobot.transform.position = Vector3.zero;
		if(cam != null) {
			cam.camPos = new Vector3(0.4f,1,-0.4f);
			cam.targetPos = new Vector3(0,1.0f,0);
			cam.focusedRobot = curRobot;
		}
		MapInfo mapInfo = PathMaker.Instance.GetActiveMapInfo();
		bool hasLoadedFarm = PathMaker.Instance.activeSubscene != null && mapInfo != null;
		bool useTerrain = robotInfo.terrain || hasLoadedFarm;
		PathMaker.Instance.useTerrain = useTerrain;

		if(useTerrain) {
			if(hasLoadedFarm) {
				terrain = PathMaker.Instance.activeSubscene;
			}
			else if(selectedTerrain != null) {
				terrain = Instantiate(selectedTerrain);
				PathMaker.Instance.activeSubscene = terrain;
				mapInfo = terrain.GetComponent<MapInfo>();
				if(mapInfo == null) mapInfo = terrain.GetComponentInChildren<MapInfo>();
				PathMaker.Instance.activeMapInfo = mapInfo;
			}

			if(terrain != null && mapInfo != null) {
				if(!PathMaker.Instance.humanoid && curRobot.GetComponent<DebugRover>() != null) {
					curRobot.GetComponent<DebugRover>().mapInfo = mapInfo;
				}
				else if(curRobot.GetComponent<HumanoidRobot>() != null) {
					curRobot.GetComponent<HumanoidRobot>().mapInfo = mapInfo;
				}
				if(mapInfo.spawn != null) {
					curRobot.transform.position = mapInfo.spawn.position;
					if(cam != null) cam.camPos = mapInfo.spawn.position;
				}
			}
    	}
		else {
			if(blankTerrainPrefab != null) terrain = Instantiate(blankTerrainPrefab);
			PathMaker.Instance.emptyTerrain = terrain;
			curRobot.transform.position = new Vector3(0, 1.4f, 0);
		}
		if(!PathMaker.Instance.humanoid) {
			DebugRover debugRover = curRobot.GetComponent<DebugRover>();
			if(debugRover != null) {
				botCam = debugRover.camera;
				if(pd != null) pd.rover = debugRover;
				if(cam != null && debugRover.camOrg != null) {
					if(!PathMaker.Instance.VR) {
						cam.transform.position = debugRover.camOrg.transform.position;
					}
					else if(cam.cc != null) {
						cam.cc.enabled = false;
						cam.transform.position = debugRover.camOrg.transform.position;
						cam.cc.enabled = true;
					}
				}
			}
			
			
			if(SocketInterace.Instance != null) {
				SocketInterace.Instance.rover = debugRover;
			}
		}
		if(PathMaker.Instance.placeCamOrg != null) PathMaker.Instance.placeCamOrg.transform.position = curRobot.transform.position + new Vector3(0,20,0);
    }

	public void UpdatePlaceMarker(Vector3 pos) {

		if(PathMaker.Instance != null && PathMaker.Instance.placeMarker != null) {
			PathMaker.Instance.placeMarker.SetActive(true);
			PathMaker.Instance.placeMarker.transform.position = pos;
		}

	}

    // Update is called once per frame
    void Update()
    {
		
	//Debug.Log(placingPlot);

		if(PathMaker.Instance != null) {
			if(!started && PathMaker.Instance.mapReady) {
				if(quickStart) {
					started = true;
					StartRobot(PathMaker.Instance.selectedRobot);
					if(PathMaker.Instance.useTerrain && terrain != null) {
						PathMaker.Instance.map = terrain.GetComponent<OnlineMaps>();
						if(PathMaker.Instance.map != null) PathMaker.Instance.map.SetPosition(PathMaker.Instance.mapCoords.x,PathMaker.Instance.mapCoords.y);
					}
				}
			}
		}
		
		
		//Placing traction spots
		if(placingTraction) {
			
			RaycastHit hit;
			if(Camera.main == null) return;
			Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
				if (Physics.Raycast (ray, out hit, 4000f)) {
					
					if(placeGhost) {
						placeGhost.transform.position = hit.point;
						
						if(Input.GetMouseButtonDown(0)) {
							
							if(!MouseInPanel()) {
								
								if(PathMaker.Instance != null && PathMaker.Instance.roverControls != null && PathMaker.Instance.roverControls.tractionDropdown != null) {
									int tractionIndex = PathMaker.Instance.roverControls.tractionDropdown.value;
									if(PathMaker.Instance.tractionSpots != null && tractionIndex >= 0 && tractionIndex < PathMaker.Instance.tractionSpots.Count && PathMaker.Instance.tractionSpots[tractionIndex] != null) {
										GameObject newTractionSpot = Instantiate(PathMaker.Instance.tractionSpots[tractionIndex]);
										newTractionSpot.transform.position = hit.point;
									}
								}
								
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
				if(PathMaker.Instance != null) PathMaker.Instance.weedDensity += 0.5f;
			}
			if(Input.GetKeyDown("[-]")) {
				if(PathMaker.Instance != null) PathMaker.Instance.weedDensity -= 0.5f;
			}
			
			//weedDensityDisplay.SetActive(true);
			
			//weedDensityDisplay.GetComponent<TMP_Text>().text = "Weed Density: " + weedDensity;
			
			RaycastHit hit;
			if(Camera.main == null) return;
			Ray ray = Camera.main.ScreenPointToRay (Input.mousePosition);
			if (Physics.Raycast (ray, out hit, 4000f)) {
				UpdatePlaceMarker(hit.point);
			}
			
			if(Input.GetMouseButtonDown(0)) {

				if(PathMaker.Instance != null && PathMaker.Instance.selectedCrop) {

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
                                if(plotHelperPrefab != null) {
	                                plotHelper = Instantiate(plotHelperPrefab);
	                                if(plotHelper.GetComponent<PlotHelper>() != null) plotHelper.GetComponent<PlotHelper>().plant = PathMaker.Instance.selectedCrop;
	                                plotHelpers.Add(plotHelper);
                                }
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
				if(plotHelper.GetComponent<PlotHelper>() != null) {
					plotHelper.GetComponent<PlotHelper>().size = new Vector2(Mathf.Abs(plotTopLeft.x-plotBottomRight.x)/2, Mathf.Abs(plotTopLeft.y-plotBottomRight.y)/2);
				}
			
			}
			
			if(Input.GetKeyDown(KeyCode.Return)) {
				//Defunct
			}
			
		}
		else {
			if(PathMaker.Instance != null && PathMaker.Instance.placeMarker != null) PathMaker.Instance.placeMarker.SetActive(false);
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
				if(fullCamScreen != null) fullCamScreen.SetActive(false);
		}
		else {
			fullView = true;
			if(fullCamScreen != null) {
				fullCamScreen.SetActive(true);
				if(fullCamScreen.GetComponent<RectTransform>() != null) fullCamScreen.GetComponent<RectTransform>().sizeDelta = new Vector2(Screen.width,Screen.height);
			}
		}
	}
}
