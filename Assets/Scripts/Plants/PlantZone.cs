using System.Collections;
using System.Collections.Generic;
using UnityEngine;

//Actually places the plants, weeds, and waypoints down

public class PlantZone : MonoBehaviour
{

	public GameObject plant;
	public GameObject weed;
	
	public int actionType = 0; //0 = Measure Moisture, 1 = Kill weeds, 2 = Snip fruit

	public float xDensity;
	public float yDensity;
	public float weedDensity;
	
	public bool makeWaypoints = true;
	
	public Transform prefabOutput;

	private Vector2 rOffset;

	private int iter;

	//random yaw and scale so fields stop looking like a grid of identical clones
	public static void ApplyNaturalVariation(GameObject plantObj) {
		if(plantObj == null) return;
		//crop menu entries can reference inactive scene objects; clones of those
		//spawn inactive and the whole field looks blank unless forced on
		if(!plantObj.activeSelf) plantObj.SetActive(true);
		plantObj.transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
		plantObj.transform.localScale *= Random.Range(0.8f, 1.3f);
	}

    // Start is called before the first frame update
    void Start()
    {
		if(PathMaker.Instance == null || plant == null || prefabOutput == null) {
			Debug.LogWarning("PlantZone: missing PathMaker, plant, or prefab output.");
			enabled = false;
			return;
		}
		
		xDensity = PathMaker.Instance.xDensity;
		yDensity = PathMaker.Instance.yDensity;
		if(xDensity <= 0 || yDensity <= 0) {
			Debug.LogWarning("PlantZone: row and column density must be greater than zero.");
			enabled = false;
			return;
		}
		
		rOffset = new Vector2(0,(1f/yDensity)/2);
		
    	prefabOutput.position = Vector3.zero;
        float x = transform.localScale.x;
        float y = transform.localScale.y;
        float z = transform.localScale.z;
        
        //some crop models are hundreds of thousands of triangles each; thin
        //the grid so one plot stays inside the graphics tier's budget
        ApplyTriangleBudget(x, z);

        float max_x = xDensity*x;
        float max_y = yDensity*z;
		
		int weedAmount = (int)(x*z*weedDensity);


		//Place human
		RaycastHit h_hit;
        Vector3 h_startPos = transform.position + new Vector3(x*1.1f,0,z*-1.1f);
        if (Physics.Raycast(h_startPos, transform.TransformDirection(-Vector3.up), out h_hit, RayLength))
        {

			if(PathMaker.Instance.plotHuman != null) {
	            GameObject newHuman = Instantiate(PathMaker.Instance.plotHuman);
			    newHuman.transform.position = h_hit.point;
				//a bystander standing in a field does not need its skeleton
				//evaluated while it is off screen
				foreach(Animator a in newHuman.GetComponentsInChildren<Animator>(true)) {
					a.cullingMode = AnimatorCullingMode.CullCompletely;
				}
			}

			//one companion is enough: this spawned a fresh 30-body ragdoll for
			//every plot, and replaced the player's own humanoid reference when
			//the player was controlling one
			if(PathMaker.Instance.humanoidRobotPrefab != null && !PathMaker.Instance.humanoid
			   && PathMaker.Instance.humanoidRobot == null) {
				GameObject newHumanoidRobot = Instantiate(PathMaker.Instance.humanoidRobotPrefab);
				newHumanoidRobot.transform.position = h_hit.point + new Vector3(1.5f, 0, 0);
				HumanoidRobot spawnedHR = newHumanoidRobot.GetComponent<HumanoidRobot>();
				if(spawnedHR != null) {
					spawnedHR.actor = PathMaker.Instance.actor;
					spawnedHR.playerControlled = false;
					PathMaker.Instance.humanoidRobot = spawnedHR;
				}
			}

        }



		for(int w = 0; w < weedAmount; w++) {
			
			//newWeed.transform.position = 
			
			RaycastHit hit;
			Vector3 startPos = new Vector3(
			Random.Range(transform.position.x-x,transform.position.x+(x)),
			y/2 + transform.position.y,
			Random.Range(transform.position.z-z,transform.position.z+(z))
			);
			if (Physics.Raycast(startPos, transform.TransformDirection(-Vector3.up), out hit, RayLength))
			{
				if(weed != null) {
				    GameObject newWeed = Instantiate(weed);
					newWeed.transform.parent = prefabOutput;
				    newWeed.transform.position = hit.point;
					ApplyNaturalVariation(newWeed);
				}
			    
			}
			
		}
        
		
		
        for(int _y = (int)-max_y; _y < max_y; _y++) {
			
			PathMaker.Waypoint wp;
			
			PathMaker.Instance.loaded = true;
			
			Plant plantComponent = plant.GetComponent<Plant>();
			if(plantComponent != null && plantComponent.moldVulnerable) {
				
        	}
			//else {
			if(actionType == 1) {
				
				for(int _x = (int)-max_x; _x < max_x; _x += 1) {
					RaycastHit hit;
					Vector3 startPos = new Vector3(_x/xDensity, y/2,(_y/yDensity));
					
					//Place Plant
					if (Physics.Raycast(startPos+transform.position, transform.TransformDirection(-Vector3.up), out hit, RayLength))
					{
						GameObject newPlant = Instantiate(plant);
						newPlant.transform.parent = prefabOutput;
						newPlant.transform.position = hit.point;
					ApplyNaturalVariation(newPlant);
					}
				}
			
				Vector2 wayPointPos;
				Vector2 wayPointPos2;
				
				wayPointPos = new Vector2(-max_x/xDensity, _y/yDensity);
				wayPointPos += new Vector2(transform.position.x, transform.position.z);
				//PathMaker.Instance.waypoints.Add(wayPointPos);
				wayPointPos2 = new Vector2(max_x/xDensity, _y/yDensity);
				wayPointPos2 += new Vector2(transform.position.x, transform.position.z);
				//PathMaker.Instance.waypoints.Add(wayPointPos);
				
				
				wayPointPos += rOffset;
				wayPointPos2 += rOffset;
				
				Vector2 midPoint = wayPointPos + wayPointPos2;
				midPoint /= 2;
				
				if(iter % 2 == 0) {
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos;
					wp.light = false;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = midPoint;
					wp.light = true;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos2;
					wp.light = true;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos2 + new Vector2((3f/xDensity),(1f/yDensity)*0.8f);
					wp.light = true;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos2 + new Vector2((2.6f/xDensity),(1f/yDensity)*1f);
					wp.light = false;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
				}
				else {
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos2;
					wp.light = false;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = midPoint;
					wp.light = true;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos;
					wp.light = true;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos + new Vector2(-(3f/xDensity),(1f/yDensity)*0.8f);
					wp.light = false;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
					wp = new PathMaker.Waypoint();
					wp.pos = wayPointPos + new Vector2(-(2.6f/xDensity),(1f/yDensity)*1f);
					wp.light = false;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
				}
			}
			
			
			int startX = (int)-max_x;
			int endX = (int)max_x;
			int _inc = 1;
			
			if(iter % 2 == 0) {
				//startX *= -1;
				//endX *= -1;
				_inc = -1;
			}
			
			//Moisture Robot Path
			
			//if(plant.GetComponent<Plant>().moldVulnerable) {
			
			if(actionType == 0) {
			
				for(int _x = startX; _x < endX; _x += 1) {
					RaycastHit hit;
					Vector3 startPos = new Vector3(_inc*_x/xDensity, y/2,(_y/yDensity));
					
					//MoveWaypoint
					if (Physics.Raycast(startPos+transform.position + new Vector3(0,0,1), transform.TransformDirection(-Vector3.up), out hit, RayLength))
					{
						Vector2 plantPoint = new Vector2(hit.point.x, hit.point.z);
						wp = new PathMaker.Waypoint();
						wp.pos = plantPoint;
						wp.light = false;
						wp.checkWater = false;
						PathMaker.Instance.waypoints.Add(wp);
					}
					
					//AimWaypoint
					if (Physics.Raycast(startPos+transform.position, transform.TransformDirection(-Vector3.up), out hit, RayLength))
					{
						GameObject newPlant = Instantiate(plant);
						newPlant.transform.parent = prefabOutput;
						newPlant.transform.position = hit.point;
					ApplyNaturalVariation(newPlant);
						Vector2 plantPoint = new Vector2(hit.point.x, hit.point.z);
						wp = new PathMaker.Waypoint();
						wp.pos = plantPoint;
						wp.light = false;
						wp.checkWater = true;
						wp.aimOnly = true;
						PathMaker.Instance.waypoints.Add(wp);
					}
				}
			}
			
			//Pick fruit
			if(actionType == 2) {
				
				for(int _x = startX; _x < endX; _x += 1) {
					RaycastHit hit;
					Vector3 startPos = new Vector3(_inc*_x/xDensity, y/2,(_y/yDensity));
					
					//MoveWaypoint
					if (Physics.Raycast(startPos+transform.position + new Vector3(0,0,0.8f), transform.TransformDirection(-Vector3.up), out hit, RayLength))
					{
						Vector2 plantPoint = new Vector2(hit.point.x, hit.point.z);
						wp = new PathMaker.Waypoint();
						wp.pos = plantPoint;
						wp.light = false;
						wp.checkWater = false;
						PathMaker.Instance.waypoints.Add(wp);
					}
					
					//AimWaypoint
					if (Physics.Raycast(startPos+transform.position, transform.TransformDirection(-Vector3.up), out hit, RayLength))
					{
						GameObject newPlant = Instantiate(plant);
						newPlant.transform.parent = prefabOutput;
						newPlant.transform.position = hit.point;
					ApplyNaturalVariation(newPlant);
						Vector2 plantPoint = new Vector2(hit.point.x, hit.point.z);
						wp = new PathMaker.Waypoint();
						wp.pos = plantPoint;
						wp.light = false;
						wp.checkWater = false;
						wp.pickFruit = true;
						wp.plant = newPlant;
						PathMaker.Instance.waypoints.Add(wp);
					}
				}
				
				RaycastHit _hit;
				Vector3 tPos = new Vector3((_inc*endX/xDensity) + _inc*3f, y/2,((_y+0.5f)/yDensity));
				if (Physics.Raycast(tPos+transform.position + new Vector3(0,0,0.8f), transform.TransformDirection(-Vector3.up), out _hit, RayLength))
				{
					Vector2 plantPoint = new Vector2(_hit.point.x, _hit.point.z);
					wp = new PathMaker.Waypoint();
					wp.pos = plantPoint;
					wp.light = false;
					wp.checkWater = false;
					PathMaker.Instance.waypoints.Add(wp);
				}
				
			}
			
			iter++;
        }
        
    }


	//---- cost control -------------------------------------------------------

	//The plot is placed 500 m above the ground, so rays never need to be infinite
	const float RayLength = 2000f;

	//Triangles one plot may add, indexed by quality level; 0 means no limit
	static readonly int[] PlotTriangleBudget = { 1500000, 3000000, 6000000, 20000000, 60000000, 0 };
	const int MinPlants = 16;

	//Reduces row and column density until the plot fits the tier's budget. The
	//Reduces row and column density until the plot fits the tier's budget. The
	//plot keeps its size and shape; the plants in it are just spaced further apart.
	void ApplyTriangleBudget(float halfX, float halfZ) {
		int level = Mathf.Clamp(QualitySettings.GetQualityLevel(), 0, PlotTriangleBudget.Length - 1);
		int budget = PlotTriangleBudget[level];
		if(budget <= 0) return;

		int perPlant = EstimateTriangles(plant);
		if(perPlant <= 0) return;

		int planned = PlannedPlants(halfX, halfZ);
		float allowed = Mathf.Max(MinPlants, budget / (float)perPlant);
		if(planned <= allowed) return;

		//the grid steps in whole rows and columns, so it rounds up: take the
		//arithmetic scale first, then tighten until the plot really does fit
		float startX = xDensity;
		float startY = yDensity;
		float scale = Mathf.Sqrt(allowed / planned);
		for(int attempt = 0; attempt < 12; attempt++) {
			xDensity = startX * scale;
			yDensity = startY * scale;
			int now = PlannedPlants(halfX, halfZ);
			if(now < 1) {
				//never thin a plot away completely; step back and stop
				scale /= 0.94f;
				xDensity = startX * scale;
				yDensity = startY * scale;
				break;
			}
			if(now <= allowed) break;
			scale *= 0.94f;
		}

		rOffset = new Vector2(0,(1f/yDensity)/2);
		Debug.LogWarning("PlantZone: " + plant.name + " is " + perPlant + " triangles per plant, so this plot was thinned from "
			+ planned + " to " + PlannedPlants(halfX, halfZ) + " plants for the "
			+ QualitySettings.names[level] + " graphics setting. Raise the Graphics setting for a denser field.");
	}

	//How many plants the grid loops below will actually place. They step in whole
	//rows and columns, so it is not simply the plot area times the densities.
	int PlannedPlants(float halfX, float halfZ) {
		float maxX = xDensity*halfX;
		float maxY = yDensity*halfZ;
		int rows = 0;
		for(int _y = (int)-maxY; _y < maxY; _y++) rows++;
		int cols = 0;
		for(int _x = (int)-maxX; _x < maxX; _x++) cols++;
		return rows * cols;
	}

	//Triangles one instance draws: everything active, plus the largest growth
	//stage (a plant enables exactly one of them at random when it starts)
	public static int EstimateTriangles(GameObject prefab) {
		if(prefab == null) return 0;

		List<Transform> stages = new List<Transform>();
		int biggestStage = 0;
		Plant p = prefab.GetComponent<Plant>();
		if(p != null && p.growthStages != null) {
			foreach(GameObject g in p.growthStages) {
				if(g == null) continue;
				stages.Add(g.transform);
				biggestStage = Mathf.Max(biggestStage, CountTriangles(g.transform, null));
			}
		}
		return CountTriangles(prefab.transform, stages) + biggestStage;
	}

	static int CountTriangles(Transform root, List<Transform> skip) {
		int total = 0;
		foreach(MeshFilter mf in root.GetComponentsInChildren<MeshFilter>(true)) {
			if(Skipped(mf.transform, root, skip)) continue;
			total += MeshTriangles(mf.sharedMesh);
		}
		foreach(SkinnedMeshRenderer smr in root.GetComponentsInChildren<SkinnedMeshRenderer>(true)) {
			if(Skipped(smr.transform, root, skip)) continue;
			total += MeshTriangles(smr.sharedMesh);
		}
		return total;
	}

	//The prefab's own root may be inactive (crop menu entries can be inactive
	//scene objects), but an inactive child really does stay hidden
	static bool Skipped(Transform t, Transform root, List<Transform> skip) {
		Transform cur = t;
		while(cur != null) {
			if(skip != null && skip.Contains(cur)) return true;
			if(cur == root) return false;
			if(!cur.gameObject.activeSelf) return true;
			cur = cur.parent;
		}
		return false;
	}

	static int MeshTriangles(Mesh m) {
		if(m == null) return 0;
		int tris = 0;
		for(int i = 0; i < m.subMeshCount; i++) tris += (int)(m.GetIndexCount(i) / 3);
		return tris;
	}

}
