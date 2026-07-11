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

	if(selectCropButton != null) selectCropButton.onClick.AddListener(() => CropButtonPressed());
	if(configRobotButton != null) configRobotButton.onClick.AddListener(() => RobotConfigButtonPressed());

        t = GetComponent<Text>();
		if(moistureText != null) moistureText.color = new Color(1,0,0,0);
		lastPlant = 0;

		//denser default planting so plots read as real crop rows, not a sparse grid
		if(xSlider != null) { xSlider.maxValue = Mathf.Max(xSlider.maxValue, 2.5f); xSlider.value = 1.4f; }
		if(ySlider != null) { ySlider.maxValue = Mathf.Max(ySlider.maxValue, 2.5f); ySlider.value = 0.55f; }
		if(weedSlider != null) { weedSlider.value = Mathf.Min(weedSlider.maxValue, 0.12f); }
    }

    // Update is called once per frame
    void Update()
    {
		
		if(plotPanel != null && plotPanel.activeSelf) {

			if(PathMaker.Instance != null) {
				if(weedSlider != null) PathMaker.Instance.weedDensity = weedSlider.value;
				if(xSlider != null) PathMaker.Instance.xDensity = xSlider.value;
				if(ySlider != null) PathMaker.Instance.yDensity = ySlider.value;
				if(xDensityDisplay != null) xDensityDisplay.text = "Row Density: " + PathMaker.Instance.xDensity.ToString("0.00");
				if(yDensityDisplay != null) yDensityDisplay.text = "Column Density: " + PathMaker.Instance.yDensity.ToString("0.00");
				if(weedDensityDisplay != null) weedDensityDisplay.text = "Weed Density: " + PathMaker.Instance.weedDensity.ToString("0.00");
			}
			
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
			
			if(moistureText != null) {
				moistureText.text = "Moisture: " + rover.measuredWater.ToString("0.00") + "%";
				moistureText.color = new Color(1,0,0,1-(rover.timeSinceMeasure/10));
			}
			
			if(rover.basket && wgtText != null) {
				wgtText.gameObject.SetActive(true);
				wgtText.text = "Basket Load: " + rover.basket.weight + "kg";
			}

			if(recText != null && rover.recording) {
				recText.text = "*REC";
			}
			else if(recText != null) {
				recText.text = "";
			}
			
			if(rover.lightOn) {
				if(lightIcon != null) lightIcon.SetActive(true);
				if(wristLightDisplay != null) wristLightDisplay.SetActive(true);
			}
			else {
				if(lightIcon != null) lightIcon.SetActive(false);
				if(wristLightDisplay != null) wristLightDisplay.SetActive(false);
			}
			
			if(powText != null) powText.text = "Power: " + rover.charge.ToString("0.00") + "%";
			
			if(t != null) t.text = "Lat: " + rover.latitude + "\nLng: " + rover.longitude + "\nHdg: " + rover.heading.ToString("0.00") + "\nVel: " + rover.vel.ToString("0.00");
			if(wristText1 != null) wristText1.text = "Lat: " + rover.latitude + "\nLng: " + rover.longitude + "\nHdg: " + rover.heading + "\nVel: " + rover.vel;
			if(wristPowerLevel != null) wristPowerLevel.text = "Power: " + rover.charge + "%";
			
			
			
		}
		else {
			if(PathMaker.Instance != null && PathMaker.Instance.basketRover && wgtText != null) {
				wgtText.gameObject.SetActive(true);
				wgtText.text = "Basket Load: " + PathMaker.Instance.basketRover.payloadWeight + "kg";
			}
		}
    }

public void CropButtonPressed() {

	if(cropSelect != null) cropSelect.SetActive(!cropSelect.activeInHierarchy);

}

void RobotConfigButtonPressed() {

	if(robotConfig != null) robotConfig.SetActive(!robotConfig.activeInHierarchy);

}

}



