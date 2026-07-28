using UnityEngine;
using UnityEditor;
using System.IO;

//Rebuilds the farm ground from scratch. The original RealWorldTerrain TerrainData
//assets were never committed to the repo, so the farm prefabs reference missing
//data: no visual terrain and no collision. This generates our own farmland
//terrain (heightmap, splat-painted ground textures, collider) and installs it
//into the MTSU Farm and StartMap prefabs, centered on each map's spawn point.
public static class FarmTerrainBuilder
{
	const float TerrainSize = 1024f;
	const float MaxHeight = 60f;
	const float PlateauFraction = 0.35f; //base ground height as fraction of MaxHeight
	const int HeightRes = 513;
	const int AlphaRes = 512;

	[MenuItem("Tools/Build Farm Terrain")]
	public static void Build()
	{
		Directory.CreateDirectory("Assets/Terrain");

		TerrainLayer[] layers = BuildLayers();

		BuildForPrefab("Assets/Maps/MTSU Farm.prefab", "Assets/Terrain/MTSUFarm_Terrain.asset", layers, 12345);
		BuildForPrefab("Assets/Maps/StartMap.prefab", "Assets/Terrain/StartMap_Terrain.asset", layers, 54321);

		AssetDatabase.SaveAssets();
		AssetDatabase.Refresh();
		Debug.Log("FarmTerrainBuilder: done.");
	}

	static void EnsureNormalMapImport(string assetPath)
	{
		TextureImporter imp = AssetImporter.GetAtPath(assetPath) as TextureImporter;
		if (imp != null && imp.textureType != TextureImporterType.NormalMap)
		{
			imp.textureType = TextureImporterType.NormalMap;
			imp.SaveAndReimport();
		}
	}

	static TerrainLayer[] BuildLayers()
	{
		foreach (string n in new[] { "GrassNormal", "DryGrassNormal", "DirtNormal", "SoilNormal" })
		{
			EnsureNormalMapImport("Assets/Textures/Ground/" + n + ".png");
		}
		TerrainLayer grass = MakeLayer("Grass", "GrassAlbedo", "GrassNormal", 12f);
		TerrainLayer dry = MakeLayer("DryGrass", "DryGrassAlbedo", "DryGrassNormal", 14f);
		TerrainLayer dirt = MakeLayer("Dirt", "DirtAlbedo", "DirtNormal", 9f);
		TerrainLayer soil = MakeLayer("Soil", "SoilAlbedo", "SoilNormal", 7f);
		return new TerrainLayer[] { grass, dry, dirt, soil };
	}

	static TerrainLayer MakeLayer(string name, string albedo, string normal, float tile)
	{
		string path = "Assets/Terrain/" + name + ".terrainlayer";
		TerrainLayer layer = AssetDatabase.LoadAssetAtPath<TerrainLayer>(path);
		if (layer == null)
		{
			layer = new TerrainLayer();
			AssetDatabase.CreateAsset(layer, path);
		}
		layer.diffuseTexture = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Textures/Ground/" + albedo + ".png");
		layer.normalMapTexture = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Textures/Ground/" + normal + ".png");
		layer.normalScale = 0.7f;
		layer.tileSize = new Vector2(tile, tile);
		EditorUtility.SetDirty(layer);
		return layer;
	}

	static void BuildForPrefab(string prefabPath, string terrainDataPath, TerrainLayer[] layers, int seed)
	{
		TerrainData data = BuildTerrainData(terrainDataPath, layers, seed);

		GameObject root = PrefabUtility.LoadPrefabContents(prefabPath);
		try
		{
			MapInfo info = root.GetComponentInChildren<MapInfo>(true);
			if (info == null || info.spawn == null)
			{
				Debug.LogError("FarmTerrainBuilder: " + prefabPath + " has no MapInfo/spawn, skipped.");
				return;
			}

			Terrain terrain = root.GetComponentInChildren<Terrain>(true);
			GameObject terrainGO;
			if (terrain == null)
			{
				terrainGO = new GameObject("FarmTerrain");
				terrainGO.transform.SetParent(root.transform, false);
				terrain = terrainGO.AddComponent<Terrain>();
				terrainGO.AddComponent<TerrainCollider>();
			}
			else
			{
				terrainGO = terrain.gameObject;
			}

			terrain.terrainData = data;
			terrain.materialTemplate = AssetDatabase.GetBuiltinExtraResource<Material>("Default-Terrain-Standard.mat");
			//instanced rendering is fine with mesh-based details (only texture
			//billboards were incompatible) and much cheaper on the CPU
			terrain.drawInstanced = true;
			terrain.heightmapPixelError = 12f;
			terrain.basemapDistance = 300f;
			terrain.detailObjectDistance = 70f;
			terrain.detailObjectDensity = 0.7f;
			terrain.drawTreesAndFoliage = true;

			TerrainCollider col = terrainGO.GetComponent<TerrainCollider>();
			if (col == null) col = terrainGO.AddComponent<TerrainCollider>();
			col.terrainData = data;

			//Center the terrain under the spawn point, with the plateau surface at spawn height
			Vector3 spawnLocal = root.transform.InverseTransformPoint(info.spawn.position);
			float plateauHeight = PlateauFraction * MaxHeight;
			terrainGO.transform.localPosition = new Vector3(
				spawnLocal.x - TerrainSize * 0.5f,
				spawnLocal.y - plateauHeight,
				spawnLocal.z - TerrainSize * 0.5f);

			PrefabUtility.SaveAsPrefabAsset(root, prefabPath);
			Debug.Log("FarmTerrainBuilder: installed terrain into " + prefabPath + " at " + terrainGO.transform.localPosition);
		}
		finally
		{
			PrefabUtility.UnloadPrefabContents(root);
		}
	}

	static TerrainData BuildTerrainData(string path, TerrainLayer[] layers, int seed)
	{
		TerrainData data = AssetDatabase.LoadAssetAtPath<TerrainData>(path);
		bool created = false;
		if (data == null)
		{
			data = new TerrainData();
			created = true;
		}

		data.heightmapResolution = HeightRes;
		data.size = new Vector3(TerrainSize, MaxHeight, TerrainSize);
		data.alphamapResolution = AlphaRes;

		System.Random prng = new System.Random(seed);
		float ox = (float)prng.NextDouble() * 1000f;
		float oy = (float)prng.NextDouble() * 1000f;

		//---- heights: gentle rolling farmland with a flat pad in the middle ----
		float[,] heights = new float[HeightRes, HeightRes];
		for (int y = 0; y < HeightRes; y++)
		{
			for (int x = 0; x < HeightRes; x++)
			{
				float u = x / (float)(HeightRes - 1);
				float v = y / (float)(HeightRes - 1);

				float hills = 0f;
				float amp = 1f;
				float freq = 3f;
				float ampSum = 0f;
				for (int o = 0; o < 4; o++)
				{
					hills += Mathf.PerlinNoise(ox + u * freq, oy + v * freq) * amp;
					ampSum += amp;
					amp *= 0.5f;
					freq *= 2f;
				}
				hills /= ampSum; //0..1
				float hillOffset = (hills - 0.5f) * 0.12f; //about +-3.6m

				//distance from center in uv, plateau blend
				float d = Vector2.Distance(new Vector2(u, v), new Vector2(0.5f, 0.5f));
				float blend = Mathf.SmoothStep(0f, 1f, Mathf.InverseLerp(0.10f, 0.30f, d));

				heights[y, x] = PlateauFraction + hillOffset * blend;
			}
		}
		data.SetHeights(0, 0, heights);

		//---- splats ----
		data.terrainLayers = layers;
		float[,,] alpha = new float[AlphaRes, AlphaRes, 4];
		const int GRASS = 0, DRY = 1, DIRT = 2, SOIL = 3;
		float metersPerCell = TerrainSize / AlphaRes;

		for (int y = 0; y < AlphaRes; y++)
		{
			for (int x = 0; x < AlphaRes; x++)
			{
				float u = x / (float)(AlphaRes - 1);
				float v = y / (float)(AlphaRes - 1);
				//world offset from terrain center in meters (alphamap x maps to world z!)
				float wx = (u - 0.5f) * TerrainSize;
				float wz = (v - 0.5f) * TerrainSize;

				float n = Mathf.PerlinNoise(ox + u * 6f, oy + v * 6f);
				float g = 1f, dr = 0f, di = 0f, so = 0f;

				//dry meadow patches
				dr = Mathf.SmoothStep(0f, 0.85f, Mathf.InverseLerp(0.55f, 0.8f, n));

				//dirt road: runs north from the spawn pad along wx=0 (alpha x == world z axis)
				float roadDist = Mathf.Abs(wz);
				if (wx > -6f)
				{
					float road = 1f - Mathf.InverseLerp(2.5f, 5.5f, roadDist);
					di = Mathf.Max(di, Mathf.Clamp01(road));
				}
				//spawn pad: packed dirt circle
				float centerDist = Mathf.Sqrt(wx * wx + wz * wz);
				di = Mathf.Max(di, 1f - Mathf.InverseLerp(8f, 16f, centerDist));

				//plowed soil plots flanking the pad
				if (IsInPlot(wx, wz, 25f, -70f, 85f, -10f)) so = 1f;   //plot west of road
				if (IsInPlot(wx, wz, 25f, 12f, 85f, 72f)) so = 1f;    //plot east of road
				//soften plot edges with noise
				if (so > 0f) so = Mathf.Clamp01(so - Mathf.InverseLerp(0.75f, 0.95f, Mathf.PerlinNoise(u * 40f, v * 40f)) * 0.4f);

				di *= (1f - so);
				dr *= (1f - so) * (1f - di);
				g = Mathf.Max(0f, 1f - dr - di - so);

				alpha[y, x, GRASS] = g;
				alpha[y, x, DRY] = dr;
				alpha[y, x, DIRT] = di;
				alpha[y, x, SOIL] = so;
			}
		}
		data.SetAlphamaps(0, 0, alpha);

		//---- waving detail grass (instanced mesh tufts; texture billboards do not
		//render reliably on 2022 terrains) ----
		GameObject greenTuft = EnsureTuftPrefab("GrassTuft", "GrassBlades");
		GameObject dryTuftGO = EnsureTuftPrefab("DryTuft", "DryBlades");
		if (greenTuft != null && dryTuftGO != null)
		{
			DetailPrototype green = new DetailPrototype
			{
				usePrototypeMesh = true,
				prototype = greenTuft,
				renderMode = DetailRenderMode.VertexLit,
				useInstancing = true,
				minWidth = 0.9f, maxWidth = 1.8f,
				minHeight = 0.6f, maxHeight = 1.2f,
				noiseSpread = 0.25f
			};
			DetailPrototype dryTuft = new DetailPrototype
			{
				usePrototypeMesh = true,
				prototype = dryTuftGO,
				renderMode = DetailRenderMode.VertexLit,
				useInstancing = true,
				minWidth = 0.8f, maxWidth = 1.5f,
				minHeight = 0.45f, maxHeight = 0.9f,
				noiseSpread = 0.35f
			};
			data.detailPrototypes = new DetailPrototype[] { green, dryTuft };

			const int DetailRes = 512;
			data.SetDetailResolution(DetailRes, 32);
			//2022+ defaults to coverage-based scatter where small values mean near-zero
			//coverage; instance count mode treats values as tufts per cell
			data.SetDetailScatterMode(DetailScatterMode.InstanceCountMode);
			int[,] greenMap = new int[DetailRes, DetailRes];
			int[,] dryMap = new int[DetailRes, DetailRes];
			for (int y = 0; y < DetailRes; y++)
			{
				for (int x = 0; x < DetailRes; x++)
				{
					float u = x / (float)(DetailRes - 1);
					float v = y / (float)(DetailRes - 1);
					float wx = (u - 0.5f) * TerrainSize;
					float wz = (v - 0.5f) * TerrainSize;

					//keep the road, spawn pad, and soil plots clear
					float centerDist = Mathf.Sqrt(wx * wx + wz * wz);
					bool onRoad = wx > -6f && Mathf.Abs(wz) < 6f;
					bool onPad = centerDist < 17f;
					bool onPlot = IsInPlot(wx, wz, 25f, -70f, 85f, -10f) || IsInPlot(wx, wz, 25f, 12f, 85f, 72f);
					if (onRoad || onPad || onPlot) continue;

					float n = Mathf.PerlinNoise(ox + u * 9f, oy + v * 9f);
					float n2 = Mathf.PerlinNoise(ox + 40f + u * 25f, oy + 40f + v * 25f);
					//kept moderate: dense grass is the single biggest perf cost on weak GPUs
					if (n > 0.36f) greenMap[y, x] = Mathf.Clamp(1 + Mathf.RoundToInt((n - 0.36f) * 18f * n2), 0, 6);
					if (n < 0.42f) dryMap[y, x] = Mathf.Clamp(Mathf.RoundToInt((0.42f - n) * 12f * n2), 0, 4);
				}
			}
			data.SetDetailLayer(0, 0, 0, greenMap);
			data.SetDetailLayer(0, 0, 1, dryMap);

			data.wavingGrassSpeed = 0.45f;
			data.wavingGrassAmount = 0.35f;
			data.wavingGrassStrength = 0.5f;
			data.wavingGrassTint = new Color(0.85f, 0.85f, 0.75f);
		}

		if (created) AssetDatabase.CreateAsset(data, path);
		EditorUtility.SetDirty(data);
		return data;
	}

	//plot given in meters from terrain center: x0..x1 along north axis (wx), z0..z1 across (wz)
	static bool IsInPlot(float wx, float wz, float x0, float z0, float x1, float z1)
	{
		return wx >= x0 && wx <= x1 && wz >= z0 && wz <= z1;
	}

	//Creates (once) a crossed-quads tuft mesh + cutout material + prefab for terrain details
	static GameObject EnsureTuftPrefab(string name, string texture)
	{
		string prefabPath = "Assets/Terrain/" + name + ".prefab";
		GameObject existing = AssetDatabase.LoadAssetAtPath<GameObject>(prefabPath);
		if (existing != null)
		{
			//instanced terrain details silently draw nothing without this flag
			Material m = AssetDatabase.LoadAssetAtPath<Material>("Assets/Terrain/" + name + "Mat.mat");
			if (m != null && !m.enableInstancing) { m.enableInstancing = true; EditorUtility.SetDirty(m); }
			return existing;
		}

		Texture2D tex = AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Textures/Ground/" + texture + ".png");
		if (tex == null) return null;

		//two crossed quads, 1m tall, pivot at ground
		Mesh mesh = new Mesh { name = name + "Mesh" };
		Vector3[] verts = new Vector3[8];
		Vector2[] uvs = new Vector2[8];
		int[] tris = new int[24];
		for (int q = 0; q < 2; q++)
		{
			float a = q * Mathf.PI * 0.5f;
			Vector3 dir = new Vector3(Mathf.Cos(a), 0, Mathf.Sin(a)) * 0.5f;
			int b = q * 4;
			verts[b + 0] = -dir; verts[b + 1] = dir;
			verts[b + 2] = dir + Vector3.up; verts[b + 3] = -dir + Vector3.up;
			uvs[b + 0] = new Vector2(0, 0); uvs[b + 1] = new Vector2(1, 0);
			uvs[b + 2] = new Vector2(1, 1); uvs[b + 3] = new Vector2(0, 1);
			int t = q * 12;
			tris[t + 0] = b; tris[t + 1] = b + 2; tris[t + 2] = b + 1;
			tris[t + 3] = b; tris[t + 4] = b + 3; tris[t + 5] = b + 2;
			//back faces
			tris[t + 6] = b; tris[t + 7] = b + 1; tris[t + 8] = b + 2;
			tris[t + 9] = b; tris[t + 10] = b + 2; tris[t + 11] = b + 3;
		}
		mesh.vertices = verts;
		mesh.uv = uvs;
		mesh.triangles = tris;
		//up-facing normals so the tufts take the terrain's soft lighting instead
		//of going black from side-facing quad normals
		Vector3[] normals = new Vector3[8];
		for (int i = 0; i < 8; i++) normals[i] = Vector3.up;
		mesh.normals = normals;
		mesh.RecalculateBounds();
		AssetDatabase.CreateAsset(mesh, "Assets/Terrain/" + name + "Mesh.asset");

		Material mat = new Material(Shader.Find("Legacy Shaders/Transparent/Cutout/Diffuse"));
		mat.mainTexture = tex;
		mat.SetFloat("_Cutoff", 0.45f);
		mat.enableInstancing = true;
		AssetDatabase.CreateAsset(mat, "Assets/Terrain/" + name + "Mat.mat");

		GameObject go = new GameObject(name);
		go.AddComponent<MeshFilter>().sharedMesh = mesh;
		go.AddComponent<MeshRenderer>().sharedMaterial = mat;
		GameObject prefab = PrefabUtility.SaveAsPrefabAsset(go, prefabPath);
		Object.DestroyImmediate(go);
		return prefab;
	}
}
