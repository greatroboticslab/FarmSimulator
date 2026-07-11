using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class RoverControls : MonoBehaviour
{
	
	public Toggle selfDriving;
	public Toggle scienceQA;
	public Button plantButton;
	public Button tractionButton;
	public Button recordButton;
	public Button toggleViewButton;
	public Button toggleViewButton2;
	public GameObject plantMenu;
	public GameObject tractionMenu;
	public GameObject recordingNotification;
	public Button mainMenuButton;
	public GameObject confirmMainMenuWindow;
	public Button yesMainMenu;
	public Button noMainMenu;
	public TMP_Text frictionDisplay;
	public TMP_Dropdown tractionDropdown;
	
	private float timeSinceRecordingStarted = 0f;
	
    // Start is called before the first frame update
    void Start()
    {
		if(selfDriving != null) selfDriving.isOn = false;
        if(plantButton != null) plantButton.onClick.AddListener(TogglePlanting);
		if(tractionButton != null) tractionButton.onClick.AddListener(ToggleTraction);
		if(recordButton != null) recordButton.onClick.AddListener(ToggleRecording);
		if(toggleViewButton != null) toggleViewButton.onClick.AddListener(ToggleView);
		if(toggleViewButton2 != null) toggleViewButton2.onClick.AddListener(ToggleView);
		if(mainMenuButton != null) mainMenuButton.onClick.AddListener(ShowMainMenuWindow);
		if(yesMainMenu != null) yesMainMenu.onClick.AddListener(GoToMainMenu);
		if(noMainMenu != null) noMainMenu.onClick.AddListener(HideMainMenuWindow);
		
		if(tractionDropdown != null) {
			tractionDropdown.onValueChanged.AddListener(delegate {
	            TractionDropdownValueChanged(tractionDropdown);
	        });
		}

		//Tutorial button joins the right-side panel, cloned so it matches the others
		TutorialBook.AttachHudButton(mainMenuButton);
    }

    // Update is called once per frame
    void Update()
    {
        if(PathMaker.Instance != null && PathMaker.Instance.rover != null) {
			if(frictionDisplay != null) frictionDisplay.text = "Friction: " + PathMaker.Instance.rover.currentFriction;
			if(selfDriving != null) PathMaker.Instance.rover.selfDriving = selfDriving.isOn;
			if(scienceQA != null) PathMaker.Instance.rover.scienceQA = scienceQA.isOn;
		}
		
		if(PathMaker.Instance != null && PathMaker.Instance.humanoidRobot) {
			if(frictionDisplay != null) frictionDisplay.text = "Friction: " + PathMaker.Instance.humanoidRobot.currentFriction;
			if(selfDriving) {			
				PathMaker.Instance.humanoidRobot.selfDriving = selfDriving.isOn;
			}
		}
		
		if(timeSinceRecordingStarted >= 5 && recordingNotification != null) {
			recordingNotification.SetActive(false);
		}
		//timeSinceRecordingStarted += Time.deltaTime;
		
    }
	
	public void TractionDropdownValueChanged(TMP_Dropdown d) {
		
		if(PathMaker.Instance != null && PathMaker.Instance.manip != null && d != null) {
			PathMaker.Instance.manip.UpdatePlacer(d.value);
		}
		
	}
	
	private void TogglePlanting() {
		if(PathMaker.Instance == null || PathMaker.Instance.manip == null || PathMaker.Instance.mainCam == null) return;

		if(tractionMenu != null && tractionMenu.activeSelf) {
			ToggleTraction();
		}

		if(plantMenu != null && plantMenu.activeSelf) {
			plantMenu.SetActive(false);
			PathMaker.Instance.manip.placingPlot = false;
			PathMaker.Instance.manip.EndEditMode();
		}
		else if(plantMenu != null) {
			plantMenu.SetActive(true);
			PathMaker.Instance.manip.placingPlot = true;
			PathMaker.Instance.mainCam.mode = 2;
			if(PathMaker.Instance.placeCamOrg != null && PathMaker.Instance.mainCam.focusedRobot != null) {
				PathMaker.Instance.placeCamOrg.transform.position = PathMaker.Instance.mainCam.focusedRobot.transform.position + new Vector3(0,15,0);
			}
		}
	}
	
	private void ToggleTraction() {
		if(PathMaker.Instance == null || PathMaker.Instance.manip == null || PathMaker.Instance.mainCam == null) return;

		if(plantMenu != null && plantMenu.activeSelf) {
			TogglePlanting();
		}
		
		if(tractionMenu != null && tractionMenu.activeSelf) {
			tractionMenu.SetActive(false);
			PathMaker.Instance.manip.EndEditMode();
		}
		else if(tractionMenu != null) {
			tractionMenu.SetActive(true);
			PathMaker.Instance.manip.UpdatePlacer(0);
			PathMaker.Instance.manip.placingTraction = true;
			PathMaker.Instance.mainCam.mode = 2;
			if(PathMaker.Instance.placeCamOrg != null && PathMaker.Instance.mainCam.focusedRobot != null) {
				PathMaker.Instance.placeCamOrg.transform.position = PathMaker.Instance.mainCam.focusedRobot.transform.position + new Vector3(0,15,0);
			}
		}
		
	}
	
	private void ToggleRecording() {
		if(PathMaker.Instance == null || PathMaker.Instance.rover == null) return;

		if(PathMaker.Instance.rover.recording) {
			PathMaker.Instance.rover.StopRecording();
			SetButtonText(recordButton, "Start Recording");
			if(recordingNotification != null) recordingNotification.SetActive(false);
		}
		else {
			
			
			PathMaker.Instance.rover.StartRecording();
			timeSinceRecordingStarted = 0f;
			if(recordingNotification != null) recordingNotification.SetActive(true);
			string s = "Recording to: " + Application.dataPath + "/Recordings/" + PathMaker.Instance.rover.curRecordDir;
			Debug.Log(s);
			if(recordingNotification != null && recordingNotification.GetComponent<TMP_Text>() != null) {
				recordingNotification.GetComponent<TMP_Text>().text = s;
			}
			SetButtonText(recordButton, "Stop Recording");
		}
	}
	
	private void ToggleView() {
		if(PathMaker.Instance != null && PathMaker.Instance.manip != null) {
			PathMaker.Instance.manip.ToggleView();
		}
	}
	
	private void GoToMainMenu() {
		
		if(confirmMainMenuWindow != null) confirmMainMenuWindow.SetActive(false);
		if(PathMaker.Instance != null) PathMaker.Instance.GoToMainMenu();
		
	}
	
	private void ShowMainMenuWindow() {
		
		if(confirmMainMenuWindow != null) confirmMainMenuWindow.SetActive(true);
		
	}
	private void HideMainMenuWindow() {
		
		if(confirmMainMenuWindow != null) confirmMainMenuWindow.SetActive(false);
		
	}

	private void SetButtonText(Button button, string text) {
		if(button == null || button.transform.childCount == 0) return;
		TMP_Text label = button.transform.GetChild(0).GetComponent<TMP_Text>();
		if(label != null) label.text = text;
	}
	
}
