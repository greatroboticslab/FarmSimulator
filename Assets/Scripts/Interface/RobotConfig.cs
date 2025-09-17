using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

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

			rover.moveSpeed = float.Parse(moveSpeedInput.text);
			rover.turnSpeed = float.Parse(turnSpeedInput.text);
			rover.minDistance = float.Parse(minDistanceInput.text);
			rover.checkDistance = float.Parse(checkDistanceInput.text);
			rover.closeSpeed = float.Parse(closeSpeedInput.text);
			rover.cruiseSpeed = float.Parse(cruiseSpeedInput.text);
			rover.headingThreshold = float.Parse(headingThresholdInput.text);

			rover.frictionMultiplier = frictionInput.value;
			frictionDisplay.text = "Friction (" + frictionInput.value + ")";

		}
	}
    }

	public void InitializeValues() {

		if(rover) {

			roverPanel.SetActive(true);
			humanoidPanel.SetActive(false);

			moveSpeedInput.text = rover.moveSpeed.ToString();
			turnSpeedInput.text = rover.turnSpeed.ToString();
			minDistanceInput.text = rover.minDistance.ToString();
			checkDistanceInput.text = rover.checkDistance.ToString();
			closeSpeedInput.text = rover.closeSpeed.ToString();
			cruiseSpeedInput.text = rover.cruiseSpeed.ToString();
			headingThresholdInput.text = rover.headingThreshold.ToString();

			frictionInput.value = rover.frictionMultiplier;
			frictionDisplay.text = "Friction (" + frictionInput.value + ")";

		}

	}

}
