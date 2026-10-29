using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[System.Serializable]
public class PropEntry {

	public GameObject prefab;
	public float weight;

}

//Scatters decorative props (trees and rocks; no people, see Populate) over the loaded farm.
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

		float yaw = (float)rnd.NextDouble() * 360f;
		float size = 1f;
		if(variants != null) size = Mathf.Lerp(variants.scaleRange.x, variants.scaleRange.y, (float)rnd.NextDouble());
		float tiltX = ((float)rnd.NextDouble() - 0.5f) * 20f;
		float tiltZ = ((float)rnd.NextDouble() - 0.5f) * 20f;

		//Find a spot. Plain props take the first one; variant props try a few and
		//keep the first their placement rules accept, and are left out if none is
		//(open ground stays open).
		Vector3 pos = Vector3.zero;
		Vector3 groundNormal = Vector3.up;
		bool placed = false;
		int attempts = variants != null ? 6 : 1;
		for(int a = 0; a < attempts && !placed; a++) {
			Vector3 rayPos = new Vector3((float)rnd.NextDouble() * mapSize.x, 300, (float)rnd.NextDouble() * mapSize.y);
			float roll = (float)rnd.NextDouble();

			RaycastHit hit;
			float alt = 0;
			Vector3 n = Vector3.up;
			Terrain terrain = null;

			//bounded ray instead of Mathf.Infinity: the terrain is never more than a
			//few hundred meters below the cast origin
			bool ground = Physics.Raycast(rayPos, Vector3.down, out hit, 1000f, ~(1 << PropLayer), QueryTriggerInteraction.Ignore);
			if (ground)
			{
				alt = hit.point.y;
				n = hit.normal;
				terrain = hit.collider.GetComponent<Terrain>();
			}

			pos = new Vector3(rayPos.x, alt, rayPos.z);
			groundNormal = n;
			//Nothing under this spot (the map area is larger than the terrain):
			//these used to go in at y = 0 anyway, and people stood 7-8 m up in
			//the air along the edge of the farm.
			placed = ground && (variants == null || roll < Acceptance(variants, terrain, pos));
		}
		if(!placed) return;

		GameObject newProp = Instantiate(model);

		Quaternion rot = model.transform.rotation;
		if(variants == null || variants.randomYaw) rot = Quaternion.Euler(0f, yaw, 0f) * rot;
		if(variants != null && variants.alignToGround)
			rot = Quaternion.FromToRotation(Vector3.up, groundNormal) * Quaternion.Euler(tiltX, 0f, tiltZ) * rot;
		newProp.transform.SetPositionAndRotation(pos, rot);
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

	//terrain layers as FarmTerrainBuilder paints them
	const int DirtLayer = 2, SoilLayer = 3;
	//fence rows: this far out from plowed ground a tree is always accepted
	const float HedgerowStrip = 9f;
	//woodlot patches are about this big
	const float WoodlotScale = 180f;

	readonly Dictionary<TerrainData, float[,,]> splats = new Dictionary<TerrainData, float[,,]>();

	//Chance that a candidate spot is kept, from PropVariants' placement rules.
	float Acceptance(PropVariants v, Terrain terrain, Vector3 pos) {
		if(terrain != null) {
			Vector3 centre = terrain.transform.position + terrain.terrainData.size * 0.5f;
			float fromSpawn = new Vector2(pos.x - centre.x, pos.z - centre.z).magnitude;
			//the maps are built around the spawn pad, where the player starts and
			//lays out the first plots; keep that ground clear
			if(fromSpawn < v.clearRadius) return 0f;

			if(v.avoidFields && LayerWeight(terrain, pos, DirtLayer) + LayerWeight(terrain, pos, SoilLayer) > 0.3f) return 0f;

			if(v.hedgerows) {
				if(LayerWeight(terrain, pos + new Vector3(HedgerowStrip, 0, 0), SoilLayer) > 0.5f
					|| LayerWeight(terrain, pos - new Vector3(HedgerowStrip, 0, 0), SoilLayer) > 0.5f
					|| LayerWeight(terrain, pos + new Vector3(0, 0, HedgerowStrip), SoilLayer) > 0.5f
					|| LayerWeight(terrain, pos - new Vector3(0, 0, HedgerowStrip), SoilLayer) > 0.5f)
					return 1f;
			}
		}
		if(v.clustering <= 0f) return 1f;
		float noise = Mathf.PerlinNoise(pos.x / WoodlotScale + seed * 0.37f + 11.3f, pos.z / WoodlotScale + seed * 0.61f + 7.9f);
		float woods = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0.38f, 0.62f, noise));
		return Mathf.Lerp(1f, woods, v.clustering);
	}

	//Weight of one terrain layer at a world position, from the terrain's splat
	//map (read once per terrain and cached; about 4 MB for a 512 map).
	float LayerWeight(Terrain terrain, Vector3 pos, int layer) {
		TerrainData td = terrain.terrainData;
		float[,,] maps;
		if(!splats.TryGetValue(td, out maps)) {
			maps = td.GetAlphamaps(0, 0, td.alphamapWidth, td.alphamapHeight);
			splats[td] = maps;
		}
		if(layer >= maps.GetLength(2)) return 0f;
		Vector3 local = pos - terrain.transform.position;
		float u = local.x / td.size.x, v = local.z / td.size.z;
		if(u < 0f || u > 1f || v < 0f || v > 1f) return 0f;
		int x = Mathf.Clamp(Mathf.RoundToInt(u * (td.alphamapWidth - 1)), 0, td.alphamapWidth - 1);
		int y = Mathf.Clamp(Mathf.RoundToInt(v * (td.alphamapHeight - 1)), 0, td.alphamapHeight - 1);
		return maps[y, x, layer];
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
		//No people on the farm. The scattered bystanders (a doctor in a white coat
		//and three others) landed at random, some partly in the ground on slopes,
		//and walked into recorded training video; they are left out. Trees and rocks
		//are unchanged and stay where they were (each prop has its own seed).
		animatedBudget = 0;
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
