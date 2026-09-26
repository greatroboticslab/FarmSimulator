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
//fills in, and off-screen people stop animating. A prop entry carrying
//PropVariants spawns one of several models with a random heading and size.
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

	//props (layer "Obstacle") must not land on each other: a rock used to be able
	//to come to rest on top of a tree's trunk collider
	const int PropLayer = 8;

	public void SpawnProp(GameObject prop) {
		SpawnProp(prop, new System.Random(Random.Range(int.MinValue, int.MaxValue)));
	}

	//Every random number comes from rnd, which Populate seeds per prop index, so a
	//lower tier (fewer props) shows a subset of the same layout, not a reshuffle.
	void SpawnProp(GameObject prop, System.Random rnd) {
		if(prop == null) return;

		//one list entry can stand for several models; see PropVariants
		PropVariants variants = prop.GetComponent<PropVariants>();
		GameObject model = variants != null ? variants.Pick(rnd) : null;
		if(model == null) model = prop;

		Vector3 rayPos = new Vector3((float)rnd.NextDouble() * mapSize.x, 300, (float)rnd.NextDouble() * mapSize.y);
		float yaw = (float)rnd.NextDouble() * 360f;
		float size = 1f;
		if(variants != null) size = Mathf.Lerp(variants.scaleRange.x, variants.scaleRange.y, (float)rnd.NextDouble());
		float tiltX = ((float)rnd.NextDouble() - 0.5f) * 20f;
		float tiltZ = ((float)rnd.NextDouble() - 0.5f) * 20f;

		RaycastHit hit;

		float alt = 0;
		Vector3 groundNormal = Vector3.up;

		//bounded ray instead of Mathf.Infinity: the terrain is never more than a
		//few hundred meters below the cast origin
		if (Physics.Raycast(rayPos, Vector3.down, out hit, 1000f, ~(1 << PropLayer), QueryTriggerInteraction.Ignore))
		{
			alt = hit.point.y;
			groundNormal = hit.normal;
		}

		GameObject newProp = Instantiate(model);

		Quaternion rot = model.transform.rotation;
		if(variants == null || variants.randomYaw) rot = Quaternion.Euler(0f, yaw, 0f) * rot;
		if(variants != null && variants.alignToGround)
			rot = Quaternion.FromToRotation(Vector3.up, groundNormal) * Quaternion.Euler(tiltX, 0f, tiltZ) * rot;
		newProp.transform.SetPositionAndRotation(new Vector3(rayPos.x, alt, rayPos.z), rot);
		newProp.transform.localScale = model.transform.localScale * size;

		if(variants != null && variants.sinkFraction > 0f) {
			Renderer r = newProp.GetComponentInChildren<Renderer>();
			if(r != null) newProp.transform.position -= groundNormal * (r.bounds.size.y * variants.sinkFraction);
		}

		newProp.transform.SetParent(transform, true);

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

			//each prop gets its own seeded generator, so skipping one (animated
			//budget) or placing fewer (lower tier) leaves every other prop where it is
			System.Random rnd = new System.Random(unchecked(seed * 486187739 + i * 16777619 + 12345));
			float rVal = (float)(rnd.NextDouble() * maxVal);
			float rIt  = 0;

			for(int j = 0; j < props.Count; j++) {
				if(props[j] == null || props[j].prefab == null) continue;

				if(rVal >= rIt && rVal < rIt + props[j].weight) {
					if(animatedEntry[j]) {
						//over budget: skip the person entirely rather than substitute
						if(animatedSpawned >= animatedBudget) break;
						animatedSpawned++;
					}
					SpawnProp(props[j].prefab, rnd);
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
