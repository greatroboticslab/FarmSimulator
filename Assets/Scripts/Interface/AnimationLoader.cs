using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class AnimationLoader : MonoBehaviour
{

	public class AnimationEntry {

		public string name;
		public AnimationClip clip;

	}

	public GameObject animationWindow;

	public List<AnimationClip> firstClips;
	public List<AnimationEntry> animations;
	public List<AnimationEntry> animationQueue;
	public TMP_InputField searchBar;

	public void LoadAnimations() {

		// Folder under Resources (no leading slash, no extension)
		string path = "Animations/MoMask";
		GameObject[] fbxObjects = Resources.LoadAll<GameObject>(path);
		firstClips = new List<AnimationClip>();
		animations = new List<AnimationEntry>();

		foreach (GameObject fbx in fbxObjects)
		{
		    // The Resource path to this specific FBX
		    string fbxPath = path + "/" + fbx.name;

		    // Load all AnimationClip sub-assets in that FBX
		    AnimationClip[] clips = Resources.LoadAll<AnimationClip>(fbxPath);

		    if (clips != null && clips.Length > 0)
		    {
		        AnimationClip firstClip = clips[0];
		        firstClips.Add(firstClip);
		        //Debug.Log($"FBX: {fbx.name} | First Animation: {firstClip.name}");
			AnimationEntry newAnim = new AnimationEntry();
			newAnim.name = fbx.name;
			newAnim.clip = clips[0];
			animations.Add(newAnim);
			//Debug.Log("Length: " + newAnim.clip.length);
		    }
		    else
		    {
		        Debug.Log($"FBX: {fbx.name} has no animation clips.");
		    }
			
		}

		Debug.Log($"Total FBX files with animations: {firstClips.Count}");

	}

	public Transform buttonPanel;
	public GameObject buttonPrefab;
	public Transform queuePanel;
	public GameObject qButtonPrefab;
	public GameObject confirmPanel;

	public string currentStatus;
	public TextMeshProUGUI statusText;
	public bool playing;
	public int currentTask;

	public float playTime;

	//Reload all animation buttons in Kinematic menu
	public void RefreshAnimations() {
		if(buttonPanel == null || buttonPrefab == null || animations == null) return;

		//Delete existing buttons
		foreach (Transform child in buttonPanel.transform) {
			Destroy(child.gameObject);
		}

		//Add animations
		for(int i = 0; i < animations.Count; i++) {
			if(ShowAnimation(animations[i])) {
				GameObject newButton = Instantiate(buttonPrefab, buttonPanel);
				if(newButton.transform.childCount > 0 && newButton.transform.GetChild(0).gameObject.GetComponent<TextMeshProUGUI>() != null) {
					newButton.transform.GetChild(0).gameObject.GetComponent<TextMeshProUGUI>().text = animations[i].name;
				}
				if(newButton.GetComponent<AnimationButton>() != null) newButton.GetComponent<AnimationButton>().animation = animations[i];
			}
		}

	}

	//Reload animation buttons in the Show Tasks menu (the queue)
	public void RefreshCurrentTasks() {
		if(queuePanel == null || buttonPrefab == null || animationQueue == null) return;

		//Delete existing buttons
		foreach (Transform child in queuePanel.transform) {
			Destroy(child.gameObject);
		}

		//Add animations
		for(int i = 0; i < animationQueue.Count; i++) {
			GameObject newButton = Instantiate(buttonPrefab, queuePanel);
			if(newButton.transform.childCount > 0 && newButton.transform.GetChild(0).gameObject.GetComponent<TextMeshProUGUI>() != null) {
				newButton.transform.GetChild(0).gameObject.GetComponent<TextMeshProUGUI>().text = "" + (i + 1) + ". " + animationQueue[i].name;
			}
			if(newButton.GetComponent<AnimationButton>() != null) newButton.GetComponent<AnimationButton>().animId = i;
		}

	}

	//Function to determine if an animation entry should be shown or not
	private bool ShowAnimation(AnimationEntry a) {
		if(a == null || string.IsNullOrEmpty(a.name)) return false;
		if(searchBar == null || string.IsNullOrEmpty(searchBar.text)) return true;
		for(int i = 0; i < searchBar.text.Length; i++) {
			if(i < a.name.Length) {
				if(char.ToLower(searchBar.text[i]) == char.ToLower(a.name[i])) {
					//Keep going
				}
				else {
					
					return false;
				}
			}
		}
		
		return true;
	}

	public void AddTask(AnimationEntry a) {
		if(a == null) return;
		if(animationQueue == null) animationQueue = new List<AnimationEntry>();
		animationQueue.Add(a);
	}

	public void RemoveTask(int i) {
		if(animationQueue == null || i < 0 || i >= animationQueue.Count) return;
		animationQueue.RemoveAt(i);
	}

	public void ToggleAnimationWindow() {
		
		if(animationWindow == null) return;
		animationWindow.SetActive(!animationWindow.activeInHierarchy);
		if(PathMaker.Instance != null && PathMaker.Instance.manip != null && PathMaker.Instance.manip.smallCamScreen != null) {
			PathMaker.Instance.manip.smallCamScreen.SetActive(!animationWindow.activeInHierarchy);
		}

	}

	public void TogglePlaceMenu() {
		if(PathMaker.Instance == null || PathMaker.Instance.posDisplay == null || PathMaker.Instance.manip == null || animationWindow == null || confirmPanel == null) return;

		if(PathMaker.Instance.posDisplay.cropSelect != null && PathMaker.Instance.posDisplay.cropSelect.activeInHierarchy) {
			PathMaker.Instance.manip.placingPlot = false;
			PathMaker.Instance.posDisplay.cropSelect.SetActive(false);
			animationWindow.SetActive(true);
			confirmPanel.SetActive(false);
			
		}
		else {
			PathMaker.Instance.manip.placingPlot = true;
			if(PathMaker.Instance.posDisplay.cropSelect != null) PathMaker.Instance.posDisplay.cropSelect.SetActive(true);
			animationWindow.SetActive(false);
			confirmPanel.SetActive(true);
		}

	}

	public void EndPlacement() {

		if(confirmPanel != null) confirmPanel.SetActive(false);
		if(PathMaker.Instance != null && PathMaker.Instance.manip != null) PathMaker.Instance.manip.placingPlot = false;
		if(animationWindow != null) animationWindow.SetActive(true);

	}

	public void PlayAll() {
		
		if(animationQueue != null && animationQueue.Count > 0) {
			if(PathMaker.Instance != null && PathMaker.Instance.humanoidRobot != null) {
				currentStatus = "Starting playback.";
				if(PathMaker.Instance.humanoidRobot.actor != null) PathMaker.Instance.humanoidRobot.actor.PlayClip(animationQueue[0].clip);
				playing = true;
				currentTask = 0;
				
			}
			else {
				currentStatus = "Cannot play: No humanoid robot.";
			}
		}
		else {
			currentStatus = "Cannot play: No animations selected.";
		}

	}

    void Start()
    {
	currentStatus = "No animations selected.";
        LoadAnimations();
	animationQueue = new List<AnimationEntry>();
	RefreshAnimations();
    }
	void Update() {

		if(animationQueue != null && animationQueue.Count > 0) {
			if(playing) {
				currentStatus = "Playing: " + animationQueue[currentTask].name;

				if(playTime >= animationQueue[currentTask].clip.length) {
					currentTask += 1;
					playTime = 0;
					if(currentTask >= animationQueue.Count) {
						currentStatus = "Animations finished.";
						currentTask = 0;
						playing = false;
					}
					else {
						if(PathMaker.Instance != null && PathMaker.Instance.humanoidRobot != null && PathMaker.Instance.humanoidRobot.actor != null) {
							PathMaker.Instance.humanoidRobot.actor.PlayClip(animationQueue[currentTask].clip);
						}
					}
				}

				playTime += Time.deltaTime;

			}
			else {
				currentStatus = "Standing by...";
			}
		}

		if(statusText != null) statusText.text = currentStatus;
	}
}
