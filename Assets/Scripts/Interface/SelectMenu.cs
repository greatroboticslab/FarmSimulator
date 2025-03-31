using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SelectMenu : MonoBehaviour
{

	//public List<GameObject> robotPrefabs;
	public List<GameObject> robots;
	public float radius;
	public bool circle;
	public GameObject selectMenuObj;
	
	[System.Serializable]
	public class RobotGroup {
		
		public string category;
		public List<GameObject> robotPrefabs;
		
	};
	
	public List<RobotGroup> robotGroups;
	
	public void HideSelections() {
		foreach(GameObject r in robots) {
			r.SetActive(false);
		}
	}
	
    // Start is called before the first frame update
    void Start()
    {
		
		
		
		if(circle) {
			
			int c = 0;
			for(int i = 0; i < robotGroups.Count; i++) {
				for(int j = 0; j < robotGroups[i].robotPrefabs.Count; j++) {
					c += 1;
				}
			}
			
			for(int i = 0; i < robotGroups.Count; i++) {
				for(int j = 0; j < robotGroups[i].robotPrefabs.Count; j++) {
					float m = Mathf.PI*2;
					float theta = m/(c);
					theta *= i+1;
					Vector3 newPos = new Vector3(Mathf.Sin(theta)*radius,0,Mathf.Cos(theta)*radius);
					GameObject newRobot = Instantiate(robotGroups[i].robotPrefabs[j]);
					newRobot.transform.position = newPos;
					newRobot.transform.SetParent(PathMaker.Instance.selectMenuObj.transform);
					robots.Add(newRobot);
				}
			}
			//Debug.Log(robotPrefabs[0]);
		}
		else {
			
			Vector3 offset = new Vector3(0,0,-3);
			
			for(int i = 0; i < robotGroups.Count; i++) {
				for(int j = 0; j < robotGroups[i].robotPrefabs.Count; j++) {
					
					Vector3 newPos = new Vector3((-i*2)+2,0,j*2) + offset;
					GameObject newRobot = Instantiate(robotGroups[i].robotPrefabs[j]);
					newRobot.transform.position = newPos;
					newRobot.transform.Rotate(0,90,0);
					newRobot.transform.SetParent(selectMenuObj.transform);
					robots.Add(newRobot);
					
				}
			}
			
			//PathMaker.Instance.placeCamOrg.transform.position += new Vector3(0,robotGroups.Count,0);
			
			
		}
		
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
