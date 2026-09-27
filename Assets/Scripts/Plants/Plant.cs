using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Plant : MonoBehaviour
{
	//registry of live plants so robots can detect crops without physics queries
	public static readonly List<Plant> All = new List<Plant>();

	//plants currently inside a UV lamp's trigger; the only ones that need
	//per-frame work, ticked from one place instead of an Update per plant
	static readonly List<Plant> Sterilizing = new List<Plant>();

	void OnEnable() { All.Add(this); }
	void OnDisable() { All.Remove(this); Sterilizing.Remove(this); }

	public GameObject visibleMesh;
	private bool wayPointAdded;

	public GameObject healthyModel;
	public GameObject deadModel;

	public float health;
	public float mold;
	public float water;
	public bool healthy;
	private bool removedWaypoint;
	public bool killingMold;
	public bool moldVulnerable;
	public DebugRover rover;

	public GameObject[] growthStages;
	public GameObject fruitParent;
	public List<GameObject> fruits;
	public int growthStage;

	public bool desired;

    // Start is called before the first frame update
    void Start()
    {
		fruits = new List<GameObject>();
		if(fruitParent != null) {
			foreach(Transform f in fruitParent.transform) {
				fruits.Add(f.gameObject);
			}
		}
		if(growthStages.Length > 0) {
			growthStage = Random.Range(0,4);
			growthStages[growthStage].SetActive(true);
		}

		health = 40;
		water = Random.Range(0,100);
        //mold = Random.Range(0,100);
		mold = 20;
		if(!desired) {
			healthy = true;
			mold = 0;
		}

		EnsureDetectable();
    }

	//The UV lamp's trigger and the moisture probe's raycast both look for a
	//collider on layer 6 ("Plant") on the object holding this component. The
	//soybean, dandelion and strawberry prefabs have one (a trigger box about the
	//plant's size); tomato, pepper and corn have neither the layer nor a
	//collider, so the UV and Moisture robots passed them by. Give those the same
	//trigger box from their renderers' bounds. Runs from Start, after PlantZone
	//has finished the raycasts that place the plot. Only this object: the fruit
	//keep their own colliders and layer.
	void EnsureDetectable() {
		if(GetComponent<Collider>() != null) return;
		Renderer[] rs = GetComponentsInChildren<Renderer>();
		if(rs.Length == 0) return;
		Bounds b = rs[0].bounds;
		for(int i = 1; i < rs.Length; i++) b.Encapsulate(rs[i].bounds);
		BoxCollider box = gameObject.AddComponent<BoxCollider>();
		box.isTrigger = true;
		Vector3 s = transform.lossyScale;
		box.center = transform.InverseTransformPoint(b.center);
		box.size = new Vector3(b.size.x / Mathf.Max(Mathf.Abs(s.x), 1e-4f), b.size.y / Mathf.Max(Mathf.Abs(s.y), 1e-4f), b.size.z / Mathf.Max(Mathf.Abs(s.z), 1e-4f));
		gameObject.layer = 6;
	}

	//Called by UVCollider when a lamp rover's trigger covers or leaves this plant
	public void SetSterilizing(DebugRover r, bool on) {
		killingMold = on;
		if(on) {
			rover = r;
			if(!Sterilizing.Contains(this)) Sterilizing.Add(this);
		}
		else {
			Sterilizing.Remove(this);
		}
	}

	//Advances every plant under a lamp; called once per frame by GraphicsSettings
	public static void TickAll(float dt) {
		for(int i = Sterilizing.Count - 1; i >= 0; i--) {
			Plant p = Sterilizing[i];
			if(p == null) { Sterilizing.RemoveAt(i); continue; }
			p.Tick(dt);
		}
	}

	void Tick(float dt)
    {

		if(rover != null) {
			if(!rover.lightOn) {
				killingMold = false;
			}
		}

		if(killingMold && rover != null) {
			float dist = Vector3.Distance(transform.position, rover.transform.position);
			float killAmount = (10f * dt)/(dist*dist);
			mold -= killAmount;
			if(mold <= 0) {
				mold = 0;
				health -= killAmount;
				healthy = true;
			}
		}

		if(health <= 0) {

			if(healthy) {
				if(deadModel != null) deadModel.SetActive(true);
				if(healthyModel != null) healthyModel.SetActive(false);
				healthy = false;
			}

		}

		if(healthy) {
			if(desired) {
				if(!removedWaypoint) {
					removedWaypoint = true;
					//rover.currentWaypoint = PathMaker.Instance.GetNextWaypoint(rover.transform.position);
				}
			}
		}
		else {
			if(!desired) {
				if(!removedWaypoint) {
					removedWaypoint = true;
					//rover.currentWaypoint = PathMaker.Instance.GetNextWaypoint(rover.transform.position);
				}
			}
		}

		/*
        if(Vector3.Distance(transform.position, Camera.main.transform.position) > 50f) {
        	visibleMesh.SetActive(false);
        }
        else {
        	visibleMesh.SetActive(true);
        }
		*/

		if(!wayPointAdded) {
			if(PathMaker.Instance != null) {
				//wayPointAdded = true;
				//PathMaker.Instance.waypoints.Add(new Vector2(transform.position.x, transform.position.z));
				//PathMaker.Instance.loaded = true;
			}
		}
    }
}
