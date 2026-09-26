using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

//Turns the generated trees (tools/tree_gen.py) into prefabs the farm uses.
//
// - import settings: leaf atlases and billboards keep their alpha coverage in
//   the mip chain (otherwise alpha-tested foliage thins out to nothing with
//   distance), normal maps as normal maps, bark tiling
// - materials: FarmSim/Bark and FarmSim/Foliage with GPU instancing, remapped
//   onto the FBX material slots by name
// - one prefab per tree in Assets/Plants/Trees: LODGroup with cross-fade, a
//   trunk collider on the Obstacle layer like the old tree1, no shadows from
//   the billboard LOD
// - tree1.prefab gets a PropVariants listing them, so the maps' ObjectPlacer
//   lists (LFS-tracked map prefabs) do not change
//
//Tools > Build Vegetation Prefabs, or -executeMethod VegetationSetup.Build
public static class VegetationSetup
{
	const string ModelDir = "Assets/Models/Vegetation";
	const string TexDir = "Assets/Textures/Vegetation";
	const string MatDir = "Assets/Materials/Vegetation";
	const string PrefabDir = "Assets/Plants/Trees";
	const string LegacyTree = "Assets/Plants/tree1.prefab";
	const int ObstacleLayer = 8;

	class Species
	{
		public string name, leaves, bark;
		public float translucency, trunkRadius, weight;
	}

	static readonly Species[] All =
	{
		new Species { name = "Oak", leaves = "OakLeaves", bark = "OakBark", translucency = 0.55f, trunkRadius = 0.42f, weight = 1.4f },
		new Species { name = "Elm", leaves = "ElmLeaves", bark = "OakBark", translucency = 0.55f, trunkRadius = 0.38f, weight = 1.0f },
		new Species { name = "Cedar", leaves = "CedarFoliage", bark = "CedarBark", translucency = 0.3f, trunkRadius = 0.3f, weight = 0.9f },
	};

	//fraction of screen height at which each LOD hands over (times the tier's
	//LOD bias): full detail up close, reduced cards to mid distance, billboard
	//beyond that and effectively never culled inside the far clip
	static readonly float[] LodHeights = { 0.3f, 0.1f, 0.004f };

	[MenuItem("Tools/Build Vegetation Prefabs")]
	public static void Build()
	{
		Directory.CreateDirectory(MatDir);
		Directory.CreateDirectory(PrefabDir);
		AssetDatabase.Refresh();

		List<GameObject> prefabs = new List<GameObject>();
		List<float> weights = new List<float>();
		int triangles0 = 0, trees = 0;

		foreach (Species sp in All)
		{
			ConfigureLeafTexture(TexDir + "/" + sp.leaves + "_Albedo.png", 0.35f);
			ConfigureNormal(TexDir + "/" + sp.leaves + "_Normal.png", TextureWrapMode.Clamp);
			ConfigureBarkTexture(TexDir + "/" + sp.bark + "_Albedo.png");
			ConfigureNormal(TexDir + "/" + sp.bark + "_Normal.png", TextureWrapMode.Repeat);

			Material bark = BarkMaterial(sp.name + "Bark", sp.bark);
			Material leaves = LeafMaterial(sp.name + "Leaves", sp.leaves, sp.translucency);

			foreach (string fbx in Directory.GetFiles(ModelDir, sp.name + "_*.fbx"))
			{
				string modelPath = fbx.Replace('\\', '/');
				string treeName = Path.GetFileNameWithoutExtension(modelPath);
				string impostorTex = TexDir + "/" + treeName + "_Impostor.png";
				ConfigureLeafTexture(impostorTex, 0.4f);
				Material impostor = ImpostorMaterial(treeName + "_Impostor", impostorTex);

				ModelImporter mi = (ModelImporter)AssetImporter.GetAtPath(modelPath);
				mi.importAnimation = false;
				mi.animationType = ModelImporterAnimationType.None;
				mi.importBlendShapes = false;
				mi.importCameras = false;
				mi.importLights = false;
				mi.importNormals = ModelImporterNormals.Import;
				mi.importTangents = ModelImporterTangents.CalculateMikk;
				mi.meshCompression = ModelImporterMeshCompression.Off;
				mi.isReadable = false;
				mi.addCollider = false;
				mi.materialImportMode = ModelImporterMaterialImportMode.ImportStandard;
				mi.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Material), sp.name + "Bark"), bark);
				mi.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Material), sp.name + "Leaves"), leaves);
				mi.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Material), treeName + "_Impostor"), impostor);
				mi.SaveAndReimport();

				GameObject model = AssetDatabase.LoadAssetAtPath<GameObject>(modelPath);
				GameObject prefab = MakePrefab(model, treeName, sp, out int tris);
				if (prefab == null) continue;
				prefabs.Add(prefab);
				weights.Add(sp.weight);
				triangles0 += tris;
				trees++;
			}
		}

		WireLegacyTree(prefabs, weights);
		AssetDatabase.SaveAssets();
		Debug.Log("VegetationSetup: " + trees + " tree prefabs, LOD0 average " + (trees > 0 ? triangles0 / trees : 0)
			+ " triangles; " + LegacyTree + " now spawns them");
	}

	static GameObject MakePrefab(GameObject model, string treeName, Species sp, out int lod0Triangles)
	{
		lod0Triangles = 0;
		GameObject go = (GameObject)PrefabUtility.InstantiatePrefab(model);
		PrefabUtility.UnpackPrefabInstance(go, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
		go.name = treeName;

		Renderer[] byLod = new Renderer[3];
		foreach (Renderer r in go.GetComponentsInChildren<Renderer>(true))
		{
			for (int i = 0; i < 3; i++)
				if (r.name.EndsWith("_LOD" + i)) byLod[i] = r;
		}
		if (byLod[0] == null || byLod[1] == null || byLod[2] == null)
		{
			Debug.LogError("VegetationSetup: " + treeName + " is missing a LOD mesh");
			Object.DestroyImmediate(go);
			return null;
		}

		MeshFilter mf = byLod[0].GetComponent<MeshFilter>();
		if (mf != null && mf.sharedMesh != null) lod0Triangles = mf.sharedMesh.triangles.Length / 3;

		byLod[2].shadowCastingMode = ShadowCastingMode.Off;
		foreach (Renderer r in byLod)
		{
			r.lightProbeUsage = LightProbeUsage.Off;
			r.reflectionProbeUsage = ReflectionProbeUsage.Off;
		}

		LODGroup group = go.GetComponent<LODGroup>();
		if (group == null) group = go.AddComponent<LODGroup>();
		LOD[] lods = new LOD[3];
		for (int i = 0; i < 3; i++) lods[i] = new LOD(LodHeights[i], new[] { byLod[i] });
		group.SetLODs(lods);
		group.fadeMode = LODFadeMode.CrossFade;
		group.animateCrossFading = true;
		group.RecalculateBounds();

		//the trunk, like the old tree1's capsule; branches are not obstacles
		CapsuleCollider cap = go.AddComponent<CapsuleCollider>();
		cap.radius = sp.trunkRadius;
		cap.height = 4.2f;
		cap.center = new Vector3(0f, 1.9f, 0f);
		cap.direction = 1;

		foreach (Transform t in go.GetComponentsInChildren<Transform>(true)) t.gameObject.layer = ObstacleLayer;
		go.isStatic = false;

		//the scale check: these should read as the heights tree_gen.py printed
		Bounds b = byLod[0].bounds;
		Debug.Log(string.Format(System.Globalization.CultureInfo.InvariantCulture,
			"VegetationSetup: {0} {1:0.0} m tall, {2:0.0} m wide, {3} LOD0 triangles, base at y {4:0.00}",
			treeName, b.size.y, Mathf.Max(b.size.x, b.size.z), lod0Triangles, b.min.y));

		string path = PrefabDir + "/" + treeName + ".prefab";
		GameObject saved = PrefabUtility.SaveAsPrefabAsset(go, path);
		Object.DestroyImmediate(go);
		return saved;
	}

	static void WireLegacyTree(List<GameObject> prefabs, List<float> weights)
	{
		GameObject root = PrefabUtility.LoadPrefabContents(LegacyTree);
		PropVariants pv = root.GetComponent<PropVariants>();
		if (pv == null) pv = root.AddComponent<PropVariants>();
		pv.variants = prefabs.ToArray();
		pv.weights = weights.ToArray();
		pv.scaleRange = new Vector2(0.8f, 1.2f);
		pv.randomYaw = true;
		pv.alignToGround = false;
		pv.sinkFraction = 0f;
		PrefabUtility.SaveAsPrefabAsset(root, LegacyTree);
		PrefabUtility.UnloadPrefabContents(root);
	}

	// ---- materials ------------------------------------------------------

	static Material Mat(string name, string shader)
	{
		string path = MatDir + "/" + name + ".mat";
		Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
		Shader s = Shader.Find(shader);
		if (m == null)
		{
			m = new Material(s);
			AssetDatabase.CreateAsset(m, path);
		}
		m.shader = s;
		m.enableInstancing = true;
		return m;
	}

	static Material BarkMaterial(string name, string tex)
	{
		Material m = Mat(name, "FarmSim/Bark");
		m.SetTexture("_MainTex", AssetDatabase.LoadAssetAtPath<Texture2D>(TexDir + "/" + tex + "_Albedo.png"));
		m.SetTexture("_BumpMap", AssetDatabase.LoadAssetAtPath<Texture2D>(TexDir + "/" + tex + "_Normal.png"));
		m.SetFloat("_Glossiness", 0.12f);
		m.SetFloat("_OcclusionFromVertex", 1f);
		m.SetFloat("_WindWeight", 1f);
		EditorUtility.SetDirty(m);
		return m;
	}

	static Material LeafMaterial(string name, string tex, float translucency)
	{
		Material m = Mat(name, "FarmSim/Foliage");
		m.SetTexture("_MainTex", AssetDatabase.LoadAssetAtPath<Texture2D>(TexDir + "/" + tex + "_Albedo.png"));
		m.SetTexture("_BumpMap", AssetDatabase.LoadAssetAtPath<Texture2D>(TexDir + "/" + tex + "_Normal.png"));
		m.SetFloat("_Cutoff", 0.35f);
		m.SetFloat("_Translucency", translucency);
		m.SetColor("_TranslucencyColor", new Color(0.78f, 0.86f, 0.40f));
		m.SetFloat("_OcclusionFromVertex", 1f);
		m.SetFloat("_WindWeight", 1f);
		EditorUtility.SetDirty(m);
		return m;
	}

	static Material ImpostorMaterial(string name, string texPath)
	{
		Material m = Mat(name, "FarmSim/Foliage");
		m.SetTexture("_MainTex", AssetDatabase.LoadAssetAtPath<Texture2D>(texPath));
		m.SetFloat("_Cutoff", 0.4f);
		//ambient occlusion is already in the render
		m.SetFloat("_OcclusionFromVertex", 0f);
		m.SetFloat("_Translucency", 0.25f);
		m.SetColor("_TranslucencyColor", new Color(0.78f, 0.86f, 0.40f));
		m.SetFloat("_WindWeight", 0.6f);
		EditorUtility.SetDirty(m);
		return m;
	}

	// ---- textures -------------------------------------------------------

	static void ConfigureLeafTexture(string path, float cutoff)
	{
		TextureImporter ti = AssetImporter.GetAtPath(path) as TextureImporter;
		if (ti == null) { Debug.LogWarning("VegetationSetup: no texture " + path); return; }
		ti.textureType = TextureImporterType.Default;
		ti.sRGBTexture = true;
		ti.alphaSource = TextureImporterAlphaSource.FromInput;
		ti.alphaIsTransparency = true;
		ti.mipmapEnabled = true;
		//keeps the same share of pixels above the cutoff in every mip level
		ti.mipMapsPreserveCoverage = true;
		ti.alphaTestReferenceValue = cutoff;
		ti.wrapMode = TextureWrapMode.Clamp;
		ti.maxTextureSize = 1024;
		ti.textureCompression = TextureImporterCompression.Compressed;
		ti.SaveAndReimport();
	}

	static void ConfigureBarkTexture(string path)
	{
		TextureImporter ti = AssetImporter.GetAtPath(path) as TextureImporter;
		if (ti == null) { Debug.LogWarning("VegetationSetup: no texture " + path); return; }
		ti.textureType = TextureImporterType.Default;
		ti.sRGBTexture = true;
		ti.mipmapEnabled = true;
		ti.wrapMode = TextureWrapMode.Repeat;
		ti.maxTextureSize = 1024;
		ti.textureCompression = TextureImporterCompression.Compressed;
		ti.SaveAndReimport();
	}

	static void ConfigureNormal(string path, TextureWrapMode wrap)
	{
		TextureImporter ti = AssetImporter.GetAtPath(path) as TextureImporter;
		if (ti == null) { Debug.LogWarning("VegetationSetup: no texture " + path); return; }
		ti.textureType = TextureImporterType.NormalMap;
		ti.wrapMode = wrap;
		ti.maxTextureSize = 1024;
		ti.SaveAndReimport();
	}
}
