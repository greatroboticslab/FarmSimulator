using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;
using System.Globalization;

public class RobotConfig : MonoBehaviour
{

	public bool sync; //Whether to set the DebugRover's parameters to this scripts.
	public DebugRover rover;
	public HumanoidRobot humanoid;
	public GameObject roverPanel;
	public GameObject humanoidPanel;

	public TMP_InputField moveSpeedInput;
	public TMP_InputField turnSpeedInput;

	public TMP_InputField minDistanceInput;
	public TMP_InputField checkDistanceInput;
	public TMP_InputField closeSpeedInput;
	public TMP_InputField cruiseSpeedInput;
	public TMP_InputField headingThresholdInput;


	public Slider frictionInput;
	public TMP_Text frictionDisplay;


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {

	if(sync) {

		if(rover) {

			rover.moveSpeed = ReadFloat(moveSpeedInput, rover.moveSpeed);
			rover.turnSpeed = ReadFloat(turnSpeedInput, rover.turnSpeed);
			rover.minDistance = ReadFloat(minDistanceInput, rover.minDistance);
			rover.checkDistance = ReadFloat(checkDistanceInput, rover.checkDistance);
			rover.closeSpeed = ReadFloat(closeSpeedInput, rover.closeSpeed);
			rover.cruiseSpeed = ReadFloat(cruiseSpeedInput, rover.cruiseSpeed);
			rover.headingThreshold = ReadFloat(headingThresholdInput, rover.headingThreshold);

			if(frictionInput != null) rover.frictionMultiplier = frictionInput.value;
			if(frictionDisplay != null && frictionInput != null) frictionDisplay.text = "Friction (" + frictionInput.value + ")";

		}
	}
    }

	public void InitializeValues() {

		if(rover) {

			if(roverPanel != null) roverPanel.SetActive(true);
			if(humanoidPanel != null) humanoidPanel.SetActive(false);

			SetText(moveSpeedInput, rover.moveSpeed);
			SetText(turnSpeedInput, rover.turnSpeed);
			SetText(minDistanceInput, rover.minDistance);
			SetText(checkDistanceInput, rover.checkDistance);
			SetText(closeSpeedInput, rover.closeSpeed);
			SetText(cruiseSpeedInput, rover.cruiseSpeed);
			SetText(headingThresholdInput, rover.headingThreshold);

			if(frictionInput != null) frictionInput.value = rover.frictionMultiplier;
			if(frictionDisplay != null && frictionInput != null) frictionDisplay.text = "Friction (" + frictionInput.value + ")";

		}

	}

	private float ReadFloat(TMP_InputField input, float fallback) {
		if(input == null) return fallback;
		if(float.TryParse(input.text, NumberStyles.Float, CultureInfo.InvariantCulture, out float parsed)) return parsed;
		return fallback;
	}

	private void SetText(TMP_InputField input, float value) {
		if(input != null) input.text = value.ToString(CultureInfo.InvariantCulture);
	}

}
