using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using System.IO;
using TMPro;
using System.Globalization;

public class CoordSelector : MonoBehaviour
{
	
	public class CoordinateEntry {
		
		public string name;
		public Vector2 coords;
		
		public CoordinateEntry(string n, float x, float y) {
			name = n;
			coords = new Vector2(x,y);
		}
		
	}
	
	public Button b;
	public Button selectRobotButton;
	public TMP_InputField lat;
	public TMP_InputField lng;
	public Dropdown dropdown;
	private string dataPath;
	private string coordListPath;
	public List<CoordinateEntry> locations;
	public GameObject ContextMenu;
	public TMP_Text curRobotDisplay;
	public string currentLocation;
	
    // Start is called before the first frame update
    void Start()
    {
		
		if(dropdown != null) {
			dropdown.onValueChanged.AddListener(delegate {
	            DropdownValueChanged(dropdown);
	        });
		}
		
		locations = new List<CoordinateEntry>();
		
		dataPath = Application.dataPath;
		coordListPath = dataPath + "/Data/locations.csv";
		//Debug.Log(Application.dataPath);
        if(b != null) b.onClick.AddListener(LoadCoords);
		if(selectRobotButton != null) selectRobotButton.onClick.AddListener(SelectRobot);

		if(File.Exists(coordListPath)) {
			StreamReader reader = new StreamReader(coordListPath);
	        string fData = reader.ReadToEnd();
			int line = 0;
			foreach(string f in fData.Split('\n')) {
				if(line > 0 && !string.IsNullOrWhiteSpace(f)) {
					string[] fentries = f.Trim().Split(',');
					if(fentries.Length >= 3 &&
					   float.TryParse(fentries[1], NumberStyles.Float, CultureInfo.InvariantCulture, out float latValue) &&
					   float.TryParse(fentries[2], NumberStyles.Float, CultureInfo.InvariantCulture, out float lngValue)) {
						CoordinateEntry newC = new CoordinateEntry(fentries[0], latValue, lngValue);
						locations.Add(newC);
					}
				}
				line += 1;
			}
	        reader.Close();
		}
		else {
			Debug.LogWarning("CoordSelector: locations.csv not found at " + coordListPath);
		}

		if(dropdown != null) {
			dropdown.options.Clear();
			foreach(CoordinateEntry e in locations) {
				Dropdown.OptionData o = new Dropdown.OptionData();
				o.text = e.name;
				dropdown.options.Add(o);
			}
		}
		
		if(locations.Count > 0) {
			if(lat != null) lat.text = locations[0].coords.x.ToString(CultureInfo.InvariantCulture);
			if(lng != null) lng.text = locations[0].coords.y.ToString(CultureInfo.InvariantCulture);
			if(dropdown != null && dropdown.captionText != null) dropdown.captionText.text = locations[0].name;
			currentLocation = locations[0].name;
		}
		
		SelectRobot();
		
    }

    // Update is called once per frame
    void Update()
    {
		if(PathMaker.Instance == null || curRobotDisplay == null) return;

		if(PathMaker.Instance.humanoid) {
			curRobotDisplay.text = "Current Robot: Humanoid";
		}
		else if(PathMaker.Instance.selectedRobot != null &&
				PathMaker.Instance.selectedRobot.robot != null &&
				PathMaker.Instance.selectedRobot.robot.GetComponent<DebugRover>() != null) {
			curRobotDisplay.text = "Current Robot: " + PathMaker.Instance.selectedRobot.robot.GetComponent<DebugRover>().robotName;
		}
    }
	
	public void SelectRobot() {
		
		if(PathMaker.Instance == null || PathMaker.Instance.mainCam == null) return;

		MainCam mainCam = PathMaker.Instance.mainCam.GetComponent<MainCam>();
		if(mainCam == null) return;

		mainCam.mode = 3;
		mainCam.currentFocusPoint = Vector3.zero;
		//PathMaker.Instance.mainCam.GetComponent<MainCam>().camPos = new Vector3(3,5,0);
		if(PathMaker.Instance.selectMenuCameraPos != null) mainCam.camPos = PathMaker.Instance.selectMenuCameraPos.position;
		if(PathMaker.Instance.selectMenuCameraFocus != null) mainCam.currentFocusPoint = PathMaker.Instance.selectMenuCameraFocus.position;

		if(PathMaker.Instance.selectMenu != null && PathMaker.Instance.selectMenu.circle) {
			mainCam.camPos = new Vector3(4,2,0);
		}
		gameObject.SetActive(false);
		
	}
	
	public void LoadCoords() {
		
		if(PathMaker.Instance == null || lat == null || lng == null) return;
		if(string.IsNullOrWhiteSpace(currentLocation) && locations.Count > 0) currentLocation = locations[0].name;

		if(!float.TryParse(lat.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float _lat) ||
		   !float.TryParse(lng.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float _lng)) {
			Debug.LogWarning("CoordSelector: invalid latitude/longitude values.");
			return;
		}
		
		//PathMaker.Instance.map.SetPosition(_lng, _lat);
		PathMaker.Instance.mapCoords = new Vector2(_lng,_lat);
		PathMaker.Instance.LoadSubscene(currentLocation);
		PathMaker.Instance.mapReady = true;

		FindObjectOfType<RobotChat>(true)?.ShowChat();

		//first farm visit each session opens the manual
		TutorialBook.ShowFirstTime();
		
		if(PathMaker.Instance.posDisplay != null) {
			PathMaker.Instance.posDisplay.gameObject.SetActive(true);
			if(PathMaker.Instance.posDisplay.powText != null) PathMaker.Instance.posDisplay.powText.gameObject.SetActive(true);
			if(PathMaker.Instance.posDisplay.moistureText != null) PathMaker.Instance.posDisplay.moistureText.gameObject.SetActive(true);
		}
		if(PathMaker.Instance.manip != null && PathMaker.Instance.manip.smallCamScreen != null) {
			PathMaker.Instance.manip.smallCamScreen.SetActive(true);
		}
		
		gameObject.SetActive(false);
		if(ContextMenu != null) ContextMenu.SetActive(true);

		


	}
	
	void DropdownValueChanged(Dropdown d)
    {
        //Debug.Log(locations[d.value].name);
		if(d == null || d.value < 0 || d.value >= locations.Count) return;

		if(lat != null) lat.text = locations[d.value].coords.x.ToString(CultureInfo.InvariantCulture);
		if(lng != null) lng.text = locations[d.value].coords.y.ToString(CultureInfo.InvariantCulture);
		currentLocation = locations[d.value].name;
    }
}
