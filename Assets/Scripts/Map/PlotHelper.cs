using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlotHelper : MonoBehaviour
{
	
	public GameObject[] stakes;
	public GameObject plantZone;
	public float weedDensity;
	
	public GameObject plant;
	public Vector2 size;
	
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
		if(stakes == null || stakes.Length < 4) return;
		for(int i = 0; i < 4; i++) {
			if(stakes[i] == null) return;
		}

        stakes[0].transform.position = transform.position + new Vector3(size.x,0,size.y);
		stakes[1].transform.position = transform.position + new Vector3(-size.x,0,size.y);
		stakes[2].transform.position = transform.position + new Vector3(-size.x,0,-size.y);
		stakes[3].transform.position = transform.position + new Vector3(size.x,0,-size.y);
    }
	
	public void PlacePlot() {
		if(plantZone == null || plant == null || PathMaker.Instance == null) return;

		//a single click (no drag) stakes a zero-area plot, which spawns a zone
		//that plants nothing; give clicks a sensible minimum plot instead
		size.x = Mathf.Max(size.x, 4f);
		size.y = Mathf.Max(size.y, 4f);
		
		GameObject newPlot = Instantiate(plantZone);
		PlantZone plantZoneComponent = newPlot.GetComponent<PlantZone>();
		Plant plantComponent = plant.GetComponent<Plant>();
		if(plantZoneComponent == null || plantComponent == null) return;

		plantZoneComponent.plant = plant;
		if(plantComponent.moldVulnerable) {
			plantZoneComponent.actionType = 0;
		}
		else {
			if(PathMaker.Instance.humanoid) {
				plantZoneComponent.actionType = 2;
			}
			else if(PathMaker.Instance.rover != null) {
				plantZoneComponent.actionType = PathMaker.Instance.rover.robotType;
			}
		}
		plantZoneComponent.weedDensity = weedDensity;
		newPlot.transform.position = transform.position;
		newPlot.transform.localScale = new Vector3(size.x,20,size.y);
		//PathMaker.Instance.roverControls.plantMenu.SetActive(false);
		if(PathMaker.Instance.humanoid && PathMaker.Instance.humanoidRobot != null) {
			PathMaker.Instance.humanoidRobot.wantsToTeleport = true;
		}
		else if(PathMaker.Instance.rover != null) {
			PathMaker.Instance.rover.wantsToTeleport = true;
		}



		//Destroy(gameObject);
	}
}
