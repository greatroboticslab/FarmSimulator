using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;

//Short grass blades growing out of the lawn around the camera, leaning in the
//wind and pushed over by the robots as they drive through, with a trail that
//springs back behind them.
//
//The terrain's own detail grass is tufts (crossed quads up to a metre wide):
//fine at a distance, but up close the ground between them is a flat texture.
//These are real blades, clumps of six tapered strips, drawn with
//DrawMeshInstancedIndirect in 8 m chunks, only where the splat map says grass
//or dry grass, so paths, soil and plots stay bare. Chunks are built as the
//camera reaches them (a few per frame, from a seed per chunk, so the same
//ground always grows the same grass) and released behind it.
//
//The quality tier sets the radius and density (GraphicsSettings.BladeRadius,
//BladeDensity); radius 0 turns them off. Drawn for the main camera only, no
//shadow casting (they receive the sun's and the clouds' shadows).
public class GrassBlades : MonoBehaviour
{
	const float ChunkSize = 8f;
	const int BladesPerClump = 6;
	const int SegmentsPerBlade = 3;
	const int ChunkBuildsPerFrame = 3;
	//beyond this a chunk draws three blades per clump instead of six
	const float LodDistance = 7f;
	const int Benders = 8;
	//how long the grass behind a robot takes to stand back up
	const float TrailSeconds = 3f;

	static GrassBlades instance;
	static float radius;
	static float density;

	//set by the quality tier: drawn radius in metres, clumps per square metre
	public static void Configure(float drawRadius, float clumpsPerSquareMetre)
	{
		//"-grass-blades off" for comparisons
		string[] cmd = System.Environment.GetCommandLineArgs();
		int flag = System.Array.IndexOf(cmd, "-grass-blades");
		if (flag >= 0 && flag + 1 < cmd.Length && cmd[flag + 1] == "off") drawRadius = 0f;
		if (Mathf.Approximately(drawRadius, radius) && Mathf.Approximately(clumpsPerSquareMetre, density)) return;
		radius = drawRadius;
		density = clumpsPerSquareMetre;
		if (instance != null) instance.ReleaseChunks();
	}

	//clumps drawn in the last frame, for the benchmark log
	public static int DrawnClumps { get; private set; }

	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
	static void Create()
	{
		if (instance != null) return;
		GameObject go = new GameObject("GrassBlades");
		DontDestroyOnLoad(go);
		instance = go.AddComponent<GrassBlades>();
	}

	class Chunk
	{
		public ComputeBuffer data;
		public ComputeBuffer args;
		public ComputeBuffer argsLod;
		public MaterialPropertyBlock props;
		public Bounds bounds;
		public int clumps;
	}

	static readonly int DataId = Shader.PropertyToID("_GrassBladeData");
	static readonly int FadeId = Shader.PropertyToID("_GrassFade");
	static readonly int BendersId = Shader.PropertyToID("_FarmGrassBenders");

	readonly Dictionary<Vector2Int, Chunk> chunks = new Dictionary<Vector2Int, Chunk>();
	readonly List<Vector2Int> wanted = new List<Vector2Int>();
	readonly List<Vector2Int> dropped = new List<Vector2Int>();
	readonly Plane[] planes = new Plane[6];
	readonly Vector4[] benders = new Vector4[Benders];
	//x, time dropped, z, radius
	readonly List<Vector4> trail = new List<Vector4>();
	readonly List<Vector4> buildData = new List<Vector4>();
	Vector3 lastTrailPoint = new Vector3(1e9f, 0f, 1e9f);

	Mesh clump;
	Mesh clumpLod;
	Material material;
	Terrain terrain;
	//which splat layers are grass and dry grass on this terrain (-1: none); the
	//weights are read per chunk, a patch of the splat map at a time, since the
	//whole map is tens of millions of floats
	int grassLayer = -1, dryLayer = -1;
	bool prepared;

	void OnDestroy()
	{
		ReleaseChunks();
		if (instance == this) instance = null;
	}

	void ReleaseChunks()
	{
		foreach (Chunk c in chunks.Values)
		{
			if (c.data != null) c.data.Release();
			if (c.args != null) c.args.Release();
			if (c.argsLod != null) c.argsLod.Release();
		}
		chunks.Clear();
	}

	void Update()
	{
		DrawnClumps = 0;
		Camera cam = Camera.main;
		Terrain t = cam != null ? TerrainUnder(cam.transform.position) : null;
		if (radius <= 0f || density <= 0f || cam == null || t == null || t.terrainData == null
			|| !SystemInfo.supportsComputeShaders || !SystemInfo.supportsInstancing)
		{
			if (chunks.Count > 0) ReleaseChunks();
			return;
		}
		if (t != terrain)
		{
			ReleaseChunks();
			terrain = t;
			prepared = false;
		}
		if (!Prepare()) return;

		UpdateBenders();
		Shader.SetGlobalVector(FadeId, new Vector4(radius, Mathf.Max(2f, radius * 0.25f), 0f, 0f));

		//chunks the circle around the camera touches, nearest first
		Vector3 c = cam.transform.position;
		int r = Mathf.CeilToInt(radius / ChunkSize) + 1;
		Vector2Int centre = new Vector2Int(Mathf.FloorToInt(c.x / ChunkSize), Mathf.FloorToInt(c.z / ChunkSize));
		float reach = radius + ChunkSize * 0.75f;
		wanted.Clear();
		for (int dz = -r; dz <= r; dz++)
			for (int dx = -r; dx <= r; dx++)
			{
				Vector2Int k = new Vector2Int(centre.x + dx, centre.y + dz);
				if (ChunkDistance(k, c) <= reach && !chunks.ContainsKey(k)) wanted.Add(k);
			}
		wanted.Sort((a, b) => ChunkDistance(a, c).CompareTo(ChunkDistance(b, c)));
		for (int i = 0; i < wanted.Count && i < ChunkBuildsPerFrame; i++) chunks[wanted[i]] = Build(wanted[i]);

		//release what the camera has left behind
		dropped.Clear();
		foreach (KeyValuePair<Vector2Int, Chunk> kv in chunks)
			if (ChunkDistance(kv.Key, c) > reach + ChunkSize) dropped.Add(kv.Key);
		foreach (Vector2Int k in dropped)
		{
			Chunk ch = chunks[k];
			if (ch.data != null) ch.data.Release();
			if (ch.args != null) ch.args.Release();
			if (ch.argsLod != null) ch.argsLod.Release();
			chunks.Remove(k);
		}

		GeometryUtility.CalculateFrustumPlanes(cam, planes);
		foreach (KeyValuePair<Vector2Int, Chunk> kv in chunks)
		{
			Chunk ch = kv.Value;
			bool near = ChunkDistance(kv.Key, c) < LodDistance;
			if (ch.clumps == 0 || !GeometryUtility.TestPlanesAABB(planes, ch.bounds)) continue;
			Graphics.DrawMeshInstancedIndirect(near ? clump : clumpLod, 0, material, ch.bounds, near ? ch.args : ch.argsLod, 0, ch.props,
				ShadowCastingMode.Off, true, 0, cam, LightProbeUsage.Off);
			DrawnClumps += ch.clumps;
		}
	}

	//the terrain the camera is over; a farm can be made of several
	static Terrain TerrainUnder(Vector3 p)
	{
		foreach (Terrain t in Terrain.activeTerrains)
		{
			if (t == null || t.terrainData == null) continue;
			Vector3 o = t.transform.position, s = t.terrainData.size;
			if (p.x >= o.x && p.x <= o.x + s.x && p.z >= o.z && p.z <= o.z + s.z) return t;
		}
		return Terrain.activeTerrain;
	}

	static float ChunkDistance(Vector2Int k, Vector3 p)
	{
		float x = Mathf.Clamp(p.x, k.x * ChunkSize, (k.x + 1) * ChunkSize);
		float z = Mathf.Clamp(p.z, k.y * ChunkSize, (k.y + 1) * ChunkSize);
		return Mathf.Sqrt((x - p.x) * (x - p.x) + (z - p.z) * (z - p.z));
	}

	//mesh, material, the ground textures and the splat weights of this terrain
	bool Prepare()
	{
		if (clump == null) clump = BuildClump(BladesPerClump);
		if (clumpLod == null) clumpLod = BuildClump(BladesPerClump / 2);
		if (material == null)
		{
			Shader s = Resources.Load<Shader>("FarmGrassBlades");
			if (s == null || !s.isSupported) { radius = 0f; return false; }
			material = new Material(s) { enableInstancing = true };
			//the ground texture's average includes the shade between its blades;
			//a blade in the open is lighter (as with the tufts' 1.12, a bit more
			//here since these are seen against that same ground up close)
			material.SetColor("_Tint", new Color(1.25f, 1.25f, 1.25f, 1f));
		}
		if (prepared) return true;
		prepared = true;

		TerrainLayer[] layers = terrain.terrainData.terrainLayers;
		grassLayer = dryLayer = -1;
		for (int i = 0; i < layers.Length; i++)
		{
			if (layers[i] == null) continue;
			string n = layers[i].name.ToLowerInvariant();
			if (n.Contains("dry")) { if (dryLayer < 0) dryLayer = i; }
			else if (n.Contains("grass") && grassLayer < 0) grassLayer = i;
		}
		if (grassLayer < 0) return true;
		SetGround("_GroundTex", "_GroundST", layers[grassLayer]);
		SetGround("_DryTex", "_DryST", layers[dryLayer >= 0 ? dryLayer : grassLayer]);
		return true;
	}

	//the terrain samples a layer at (world - terrain position + offset) / tile size
	void SetGround(string tex, string st, TerrainLayer layer)
	{
		material.SetTexture(tex, layer.diffuseTexture);
		Vector3 tp = terrain.transform.position;
		Vector2 size = new Vector2(Mathf.Max(layer.tileSize.x, 0.01f), Mathf.Max(layer.tileSize.y, 0.01f));
		material.SetVector(st, new Vector4(1f / size.x, 1f / size.y, (layer.tileOffset.x - tp.x) / size.x, (layer.tileOffset.y - tp.z) / size.y));
	}

	Chunk Build(Vector2Int key)
	{
		Chunk ch = new Chunk();
		if (grassLayer < 0) return ch;
		System.Random rnd = new System.Random(unchecked(key.x * 73856093 ^ key.y * 19349663 ^ 0x2545F491));
		TerrainData td = terrain.terrainData;
		Vector3 tp = terrain.transform.position;
		Vector3 size = td.size;
		int splatW = td.alphamapWidth, splatH = td.alphamapHeight;

		//this chunk's patch of the splat map
		int x0 = Mathf.Clamp(Mathf.FloorToInt((key.x * ChunkSize - tp.x) / size.x * splatW), 0, splatW - 1);
		int y0 = Mathf.Clamp(Mathf.FloorToInt((key.y * ChunkSize - tp.z) / size.z * splatH), 0, splatH - 1);
		int x1 = Mathf.Clamp(Mathf.CeilToInt(((key.x + 1) * ChunkSize - tp.x) / size.x * splatW), 0, splatW - 1);
		int y1 = Mathf.Clamp(Mathf.CeilToInt(((key.y + 1) * ChunkSize - tp.z) / size.z * splatH), 0, splatH - 1);
		float[,,] splat = td.GetAlphamaps(x0, y0, x1 - x0 + 1, y1 - y0 + 1);

		int tries = Mathf.RoundToInt(density * ChunkSize * ChunkSize);
		buildData.Clear();
		float minY = float.MaxValue, maxY = float.MinValue;
		for (int i = 0; i < tries; i++)
		{
			float x = (key.x + (float)rnd.NextDouble()) * ChunkSize;
			float z = (key.y + (float)rnd.NextDouble()) * ChunkSize;
			float keep = (float)rnd.NextDouble();
			float u = (x - tp.x) / size.x, v = (z - tp.z) / size.z;
			if (u < 0f || u > 1f || v < 0f || v > 1f) continue;
			int sx = Mathf.Clamp((int)(u * splatW), x0, x1) - x0;
			int sy = Mathf.Clamp((int)(v * splatH), y0, y1) - y0;
			float g = splat[sy, sx, grassLayer];
			float d = dryLayer >= 0 ? splat[sy, sx, dryLayer] : 0f;
			float w = g + d;
			//thinning out toward the edges of paths and fields, none on them
			if (w < 0.3f || keep > w) continue;
			float y = terrain.SampleHeight(new Vector3(x, 0f, z)) + tp.y;
			buildData.Add(new Vector4(x, y, z, (float)rnd.NextDouble() * Mathf.PI * 2f));
			buildData.Add(new Vector4(0.75f + 0.55f * (float)rnd.NextDouble(), d / w, (float)rnd.NextDouble(), 0f));
			minY = Mathf.Min(minY, y);
			maxY = Mathf.Max(maxY, y);
		}
		ch.clumps = buildData.Count / 2;
		if (ch.clumps == 0) return ch;

		ch.data = new ComputeBuffer(buildData.Count, 16);
		ch.data.SetData(buildData);
		ch.args = new ComputeBuffer(1, 5 * sizeof(uint), ComputeBufferType.IndirectArguments);
		ch.args.SetData(new uint[] { clump.GetIndexCount(0), (uint)ch.clumps, clump.GetIndexStart(0), clump.GetBaseVertex(0), 0 });
		ch.argsLod = new ComputeBuffer(1, 5 * sizeof(uint), ComputeBufferType.IndirectArguments);
		ch.argsLod.SetData(new uint[] { clumpLod.GetIndexCount(0), (uint)ch.clumps, clumpLod.GetIndexStart(0), clumpLod.GetBaseVertex(0), 0 });
		ch.props = new MaterialPropertyBlock();
		ch.props.SetBuffer(DataId, ch.data);
		ch.bounds = new Bounds(
			new Vector3((key.x + 0.5f) * ChunkSize, (minY + maxY) * 0.5f + 0.2f, (key.y + 0.5f) * ChunkSize),
			new Vector3(ChunkSize + 1f, maxY - minY + 1.2f, ChunkSize + 1f));
		return ch;
	}

	//The robots push the grass over where they are, and for a few seconds where
	//they have just been: a point is dropped along each track every 0.25 m or so
	//and fades out over TrailSeconds.
	void UpdateBenders()
	{
		float now = Time.time;
		int n = 0;
		PathMaker pm = PathMaker.Instance;
		if (pm != null)
		{
			if (pm.rover != null) AddBender(pm.rover.transform.position, 0.9f, ref n, true);
			if (pm.basketRover != null) AddBender(pm.basketRover.transform.position, 0.8f, ref n, false);
			if (pm.humanoidRobot != null) AddBender(pm.humanoidRobot.transform.position, 0.45f, ref n, false);
		}
		for (int i = trail.Count - 1; i >= 0; i--)
			if (now - trail[i].y > TrailSeconds) trail.RemoveAt(i);
		for (int i = trail.Count - 1; i >= 0 && n < Benders; i--)
		{
			Vector4 p = trail[i];
			float strength = 1f - (now - p.y) / TrailSeconds;
			benders[n++] = new Vector4(p.x, strength * strength, p.z, p.w);
		}
		while (n < Benders) benders[n++] = Vector4.zero;
		Shader.SetGlobalVectorArray(BendersId, benders);
	}

	void AddBender(Vector3 p, float r, ref int n, bool track)
	{
		if (n >= Benders) return;
		benders[n++] = new Vector4(p.x, 1f, p.z, r);
		if (!track) return;
		if ((p - lastTrailPoint).sqrMagnitude > 0.25f * 0.25f)
		{
			lastTrailPoint = p;
			trail.Add(new Vector4(p.x, Time.time, p.z, r * 0.85f));
			if (trail.Count > Benders) trail.RemoveAt(0);
		}
	}

	//Tapered blades (six, or three far off) 12-22 cm tall around a 6 cm circle, each three segments
	//curving over to a point. Normals mostly up, like the ground, so a lawn of
	//them lights like the terrain it grows from. Vertex colour: R a per-blade
	//random, G height along the blade, B a per-blade phase.
	//The far version is the first half of the same blades, so a chunk switching
	//over loses blades rather than moving them.
	static Mesh BuildClump(int blades)
	{
		System.Random rnd = new System.Random(7);
		List<Vector3> verts = new List<Vector3>();
		List<Vector3> normals = new List<Vector3>();
		List<Color> colours = new List<Color>();
		List<int> tris = new List<int>();
		for (int b = 0; b < blades; b++)
		{
			float a = (float)rnd.NextDouble() * Mathf.PI * 2f;
			float rr = 0.06f * Mathf.Sqrt((float)rnd.NextDouble());
			Vector3 root = new Vector3(Mathf.Cos(a) * rr, 0f, Mathf.Sin(a) * rr);
			float h = 0.12f + 0.10f * (float)rnd.NextDouble();
			float w = 0.008f + 0.005f * (float)rnd.NextDouble();
			float face = (float)rnd.NextDouble() * Mathf.PI * 2f;
			Vector3 side = new Vector3(Mathf.Cos(face), 0f, Mathf.Sin(face));
			Vector3 facing = new Vector3(-side.z, 0f, side.x);
			Vector3 lean = facing * ((0.2f + 0.35f * (float)rnd.NextDouble()) * h);
			Vector3 normal = (Vector3.up * 0.8f + facing * 0.2f).normalized;
			float random = (float)rnd.NextDouble();
			float phase = (float)rnd.NextDouble();
			int first = verts.Count;
			for (int s = 0; s <= SegmentsPerBlade; s++)
			{
				float t = s / (float)SegmentsPerBlade;
				Vector3 centre = root + Vector3.up * (h * t) + lean * (t * t);
				if (s < SegmentsPerBlade)
				{
					float half = 0.5f * w * (1f - t * 0.85f);
					verts.Add(centre - side * half);
					verts.Add(centre + side * half);
					normals.Add(normal);
					normals.Add(normal);
					colours.Add(new Color(random, t, phase, 1f));
					colours.Add(new Color(random, t, phase, 1f));
				}
				else
				{
					verts.Add(centre);
					normals.Add(normal);
					colours.Add(new Color(random, 1f, phase, 1f));
				}
			}
			for (int s = 0; s < SegmentsPerBlade - 1; s++)
			{
				int i0 = first + s * 2;
				tris.Add(i0); tris.Add(i0 + 2); tris.Add(i0 + 1);
				tris.Add(i0 + 1); tris.Add(i0 + 2); tris.Add(i0 + 3);
			}
			int last = first + (SegmentsPerBlade - 1) * 2;
			tris.Add(last); tris.Add(last + 2); tris.Add(last + 1);
		}
		Mesh m = new Mesh { name = "GrassClump" };
		m.SetVertices(verts);
		m.SetNormals(normals);
		m.SetColors(colours);
		m.SetTriangles(tris, 0);
		//the shader moves vertices by up to about a blade length
		m.bounds = new Bounds(new Vector3(0f, 0.1f, 0f), new Vector3(0.6f, 0.4f, 0.6f));
		return m;
	}
}
