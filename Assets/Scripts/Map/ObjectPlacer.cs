using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class PropEntry {

	public GameObject prefab;
	public float weight;

}

//Scatters decorative props (trees, rocks, people) over the loaded farm.
//The prop count follows the graphics tier, animated people are capped,
//spawning is spread across frames so the farm does not freeze while it
//fills in, and off-screen people stop animating.
public class ObjectPlacer : MonoBehaviour
{

	public int propAmount;

	public int seed;

	public Vector2 mapSize;

	public List<PropEntry> props;

	//instantiations per frame while the map fills in
	public int spawnPerFrame = 120;

	private float waitTime = 2f;
	private float t;
	private bool loaded;

	//true once every prop for this map has been placed
	public bool finished;

	private int animatedBudget;
	private int animatedSpawned;
	private bool[] animatedEntry;

	public void SpawnProp(GameObject prop) {
		if(prop == null) return;

		GameObject newProp = Instantiate(prop);
		newProp.transform.parent = transform;

		Vector3 rayPos = new Vector3(Random.Range(0,mapSize.x), 300, Random.Range(0,mapSize.y));

		RaycastHit hit;

		float alt = 0;

		//bounded ray instead of Mathf.Infinity: the terrain is never more than a
		//few hundred meters below the cast origin
		if (Physics.Raycast(rayPos, Vector3.down, out hit, 1000f, ~0, QueryTriggerInteraction.Ignore))
		{
			alt = hit.point.y;
		}

		newProp.transform.position = new Vector3(rayPos.x, alt, rayPos.z);

		//people that are not on screen do not need their skeleton evaluated
		foreach (Animator a in newProp.GetComponentsInChildren<Animator>(true))
		{
			a.cullingMode = AnimatorCullingMode.CullCompletely;
		}

	}

	//skinned, animated props cost far more than a tree; remember which entries are animated
	static bool IsAnimated(GameObject prefab) {
		if(prefab == null) return false;
		return prefab.GetComponentInChildren<Animator>(true) != null
			|| prefab.GetComponentInChildren<SkinnedMeshRenderer>(true) != null;
	}

    // Start is called before the first frame update
    void Start()
    {



    }

    // Update is called once per frame
    void Update()
    {
        if(t >= waitTime && !loaded) {
			loaded = true;

			GrassSpawner grassSpawner = GetComponent<GrassSpawner>();
			if(grassSpawner != null) {
				grassSpawner.StartGrass();
			}

			StartCoroutine(Populate());
		}
		else if(!loaded) {
			t += Time.deltaTime;
		}
    }

	IEnumerator Populate() {
		Random.InitState(seed);
		float maxVal = 0;

		animatedEntry = new bool[props.Count];
		for(int i = 0; i < props.Count; i++) {
			if(props[i] != null && props[i].prefab != null) {
				maxVal += props[i].weight;
				animatedEntry[i] = IsAnimated(props[i].prefab);
			}
		}

		if(maxVal <= 0) { finished = true; yield break; }

		//graphics tier decides how much decoration this machine can afford
		float fraction = GraphicsSettings.PropFraction();
		animatedBudget = GraphicsSettings.AnimatedPropBudget();
		int total = Mathf.RoundToInt(propAmount * fraction);
		Debug.Log("[ObjectPlacer] placing " + total + " of " + propAmount + " props (animated cap " + animatedBudget + ") at quality " + QualitySettings.names[QualitySettings.GetQualityLevel()]);

		int perFrame = Mathf.Max(1, spawnPerFrame);
		int sinceYield = 0;

		for(int i = 0; i < total; i++) {

			float rVal = Random.Range(0,maxVal);
			float rIt  = 0;

			for(int j = 0; j < props.Count; j++) {
				if(props[j] == null || props[j].prefab == null) continue;

				if(rVal >= rIt && rVal < rIt + props[j].weight) {
					if(animatedEntry[j]) {
						//over budget: skip the person entirely rather than substitute,
						//so the seeded layout of everything else stays identical
						if(animatedSpawned >= animatedBudget) break;
						animatedSpawned++;
					}
					SpawnProp(props[j].prefab);
					sinceYield++;
					break;
				}
				rIt += props[j].weight;

			}

			if(sinceYield >= perFrame) {
				sinceYield = 0;
				yield return null;
			}

		}

		finished = true;
	}
}
