using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class PosDisplay : MonoBehaviour
{

	public DebugRover rover;
	private Text t;
	public Text recText;
	public Text powText;
	public Text wgtText;
	public Text moistureText;
	
	public GameObject lightIcon;
	
	public TMP_Text weedDensityDisplay;
	public TMP_Text xDensityDisplay;
	public TMP_Text yDensityDisplay;
	public GameObject plotPanel;
	public Slider weedSlider;
	public Slider xSlider;
	public Slider ySlider;
	public RectTransform cropSelectRect;
	public GameObject cropSelect;
	public GameObject robotConfig;
	
	public TMP_Text wristText1;
	public TMP_Text wristPowerLevel;
	public GameObject wristLightDisplay;

	public Button selectCropButton;
	public Button configRobotButton;
	
	private int lastPlant;
	
	

    // Start is called before the first frame update
    void Start()
    {

	selectCropButton.onClick.AddListener(() => CropButtonPressed());
	configRobotButton.onClick.AddListener(() => RobotConfigButtonPressed());

        t = GetComponent<Text>();
		moistureText.color = new Color(1,0,0,0);
		lastPlant = 0;
    }

    // Update is called once per frame
    void Update()
    {
		
		if(plotPanel.active == true) {

			PathMaker.Instance.weedDensity = weedSlider.value;
			PathMaker.Instance.xDensity = xSlider.value;
			PathMaker.Instance.yDensity = ySlider.value;
			xDensityDisplay.text = "Row Density: " + PathMaker.Instance.xDensity.ToString("0.00");
			yDensityDisplay.text = "Column Density: " + PathMaker.Instance.yDensity.ToString("0.00");
			weedDensityDisplay.text = "Weed Density: " + PathMaker.Instance.weedDensity.ToString("0.00");
			
			//PathMaker.Instance.selectedCropId = cropSelect.value;
			//PathMaker.Instance.selectedVGGTCropId = vggtCropSelect.value;
			/*
			if(cropSelect.value != lastPlant) {
				if(cropSelect.value == 1) {
					//Strawberry
					xSlider.value = 2;
					ySlider.value = 0.6f;
				}
			}
			lastPlant = cropSelect.value;
			*/
			
		}
		
		if(rover != null) {
			
			moistureText.text = "Moisture: " + rover.measuredWater.ToString("0.00") + "%";
			moistureText.color = new Color(1,0,0,1-(rover.timeSinceMeasure/10));
			
			if(rover.basket) {
				wgtText.gameObject.SetActive(true);
				wgtText.text = "Basket Load: " + rover.basket.weight + "kg";
			}

			if(rover.recording) {
				recText.text = "*REC";
			}
			else {
				recText.text = "";
			}
			
			if(rover.lightOn) {
				lightIcon.SetActive(true);
				wristLightDisplay.SetActive(true);
			}
			else {
				lightIcon.SetActive(false);
				wristLightDisplay.SetActive(false);
			}
			
			powText.text = "Power: " + rover.charge.ToString("0.00") + "%";
			
        	t.text = "Lat: " + rover.latitude + "\nLng: " + rover.longitude + "\nHdg: " + rover.heading.ToString("0.00") + "\nVel: " + rover.vel.ToString("0.00");
			wristText1.text = "Lat: " + rover.latitude + "\nLng: " + rover.longitude + "\nHdg: " + rover.heading + "\nVel: " + rover.vel;
			wristPowerLevel.text = "Power: " + rover.charge + "%";
			
			
			
		}
		else {
			if(PathMaker.Instance.basketRover) {
				wgtText.gameObject.SetActive(true);
				wgtText.text = "Basket Load: " + PathMaker.Instance.basketRover.payloadWeight + "kg";
			}
		}
    }

public void CropButtonPressed() {

	cropSelect.SetActive(!cropSelect.activeInHierarchy);

}

void RobotConfigButtonPressed() {

	robotConfig.SetActive(!robotConfig.activeInHierarchy);

}

}



