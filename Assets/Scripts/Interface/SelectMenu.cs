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
					newRobot.AddComponent<RobotShowcase>();
					robots.Add(newRobot);

				}
			}

			BuildShowroomFloor();

		}

    }

	//Dark grid floor under the select stage so the showroom is not a white void
	void BuildShowroomFloor() {

		GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
		floor.name = "ShowroomFloor";
		Object.Destroy(floor.GetComponent<Collider>());
		floor.transform.SetParent(selectMenuObj != null ? selectMenuObj.transform : transform, false);
		floor.transform.position = new Vector3(-1f, 0.01f, 2f);
		floor.transform.localScale = new Vector3(9f, 1f, 9f); //90x90 meters

		Material mat = new Material(Shader.Find("Standard"));
		mat.color = new Color(0.13f, 0.13f, 0.14f); //dark concrete, not water
		mat.SetFloat("_Metallic", 0.1f);
		mat.SetFloat("_Glossiness", 0.38f);
		mat.mainTexture = BuildGridTexture();
		mat.mainTextureScale = new Vector2(24f, 24f);
		floor.GetComponent<MeshRenderer>().material = mat;
	}

	static Texture2D BuildGridTexture() {

		const int size = 128;
		Texture2D tex = new Texture2D(size, size, TextureFormat.RGBA32, true);
		Color baseCol = new Color(0.72f, 0.76f, 0.82f);
		Color lineCol = new Color(1.4f, 1.7f, 1.9f); //brighter than base so lines catch light
		for(int y = 0; y < size; y++) {
			for(int x = 0; x < size; x++) {
				bool line = x < 2 || y < 2;
				tex.SetPixel(x, y, line ? lineCol : baseCol);
			}
		}
		tex.Apply();
		tex.wrapMode = TextureWrapMode.Repeat;
		return tex;
	}

    // Update is called once per frame
    void Update()
    {
        
    }
}
