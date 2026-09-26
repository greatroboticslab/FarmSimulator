using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

//Turns the decimated rock scans (tools/rock_gen.py) into prefabs and makes
//rock1, the single blockout the maps scatter over a thousand times, spawn them.
//
// - Standard material per rock: the scan's colour with baked occlusion, its
//   normal map, rough (smoothness 0.12), GPU instancing
// - LODGroup: 2400 / 600 / 150 triangles, culled once a rock is a few pixels
// - convex collider from the 150-triangle LOD, on the Obstacle layer like rock1's
// - rock1.prefab gains a PropVariants: tilted to the ground and sunk a quarter
//   of the way in, sizes 0.6 to 1.5, loosely clustered, piled along field edges
//   the way cleared field stones are
//
//Tools > Build Rock Prefabs, or -executeMethod RockSetup.Build
public static class RockSetup
{
	const string ModelDir = "Assets/Models/Rocks";
	const string TexDir = "Assets/Textures/Rocks";
	const string MatDir = "Assets/Materials/Rocks";
	const string PrefabDir = "Assets/Props/Rocks";
	const string LegacyRock = "Assets/Props/rock1.prefab";
	const int ObstacleLayer = 8;

	static readonly float[] LodHeights = { 0.08f, 0.025f, 0.004f };

	[MenuItem("Tools/Build Rock Prefabs")]
	public static void Build()
	{
		Directory.CreateDirectory(MatDir);
		Directory.CreateDirectory(PrefabDir);
		AssetDatabase.Refresh();

		List<GameObject> prefabs = new List<GameObject>();
		List<float> weights = new List<float>();
		foreach (string fbx in Directory.GetFiles(ModelDir, "*.fbx"))
		{
			string modelPath = fbx.Replace('\\', '/');
			string name = Path.GetFileNameWithoutExtension(modelPath);

			string albedo = TexDir + "/" + name + "_Albedo.png";
			string normal = TexDir + "/" + name + "_Normal.png";
			TextureImporter ta = AssetImporter.GetAtPath(albedo) as TextureImporter;
			if (ta != null) { ta.sRGBTexture = true; ta.maxTextureSize = 1024; ta.SaveAndReimport(); }
			TextureImporter tn = AssetImporter.GetAtPath(normal) as TextureImporter;
			if (tn != null) { tn.textureType = TextureImporterType.NormalMap; tn.maxTextureSize = 1024; tn.SaveAndReimport(); }

			string matPath = MatDir + "/" + name + ".mat";
			Material mat = AssetDatabase.LoadAssetAtPath<Material>(matPath);
			if (mat == null)
			{
				mat = new Material(Shader.Find("Standard"));
				AssetDatabase.CreateAsset(mat, matPath);
			}
			mat.SetTexture("_MainTex", AssetDatabase.LoadAssetAtPath<Texture2D>(albedo));
			mat.SetTexture("_BumpMap", AssetDatabase.LoadAssetAtPath<Texture2D>(normal));
			mat.EnableKeyword("_NORMALMAP");
			mat.SetFloat("_Glossiness", 0.12f);
			mat.SetFloat("_Metallic", 0f);
			mat.enableInstancing = true;
			EditorUtility.SetDirty(mat);

			ModelImporter mi = (ModelImporter)AssetImporter.GetAtPath(modelPath);
			mi.importAnimation = false;
			mi.animationType = ModelImporterAnimationType.None;
			mi.importNormals = ModelImporterNormals.Import;
			mi.importTangents = ModelImporterTangents.CalculateMikk;
			mi.isReadable = false;
			mi.addCollider = false;
			mi.materialImportMode = ModelImporterMaterialImportMode.ImportStandard;
			mi.AddRemap(new AssetImporter.SourceAssetIdentifier(typeof(Material), name + "Rock"), mat);
			mi.SaveAndReimport();

			GameObject model = AssetDatabase.LoadAssetAtPath<GameObject>(modelPath);
			GameObject prefab = MakePrefab(model, name);
			if (prefab == null) continue;
			prefabs.Add(prefab);
			//the boulder is the big one; fewer of those
			weights.Add(name.StartsWith("Boulder") ? 0.6f : 1f);
		}

		GameObject root = PrefabUtility.LoadPrefabContents(LegacyRock);
		PropVariants pv = root.GetComponent<PropVariants>();
		if (pv == null) pv = root.AddComponent<PropVariants>();
		pv.variants = prefabs.ToArray();
		pv.weights = weights.ToArray();
		pv.scaleRange = new Vector2(0.6f, 1.5f);
		pv.randomYaw = true;
		pv.alignToGround = true;
		pv.sinkFraction = 0.25f;
		pv.avoidFields = true;
		pv.clustering = 0.5f;
		pv.hedgerows = true;
		pv.clearRadius = 20f;
		PrefabUtility.SaveAsPrefabAsset(root, LegacyRock);
		PrefabUtility.UnloadPrefabContents(root);

		AssetDatabase.SaveAssets();
		Debug.Log("RockSetup: " + prefabs.Count + " rock prefabs; " + LegacyRock + " now spawns them");
	}

	static GameObject MakePrefab(GameObject model, string name)
	{
		GameObject go = (GameObject)PrefabUtility.InstantiatePrefab(model);
		PrefabUtility.UnpackPrefabInstance(go, PrefabUnpackMode.Completely, InteractionMode.AutomatedAction);
		go.name = name;

		Renderer[] byLod = new Renderer[3];
		foreach (Renderer r in go.GetComponentsInChildren<Renderer>(true))
			for (int i = 0; i < 3; i++)
				if (r.name.EndsWith("_LOD" + i)) byLod[i] = r;
		if (byLod[0] == null || byLod[1] == null || byLod[2] == null)
		{
			Debug.LogError("RockSetup: " + name + " is missing a LOD mesh");
			Object.DestroyImmediate(go);
			return null;
		}
		byLod[2].shadowCastingMode = ShadowCastingMode.Off;

		LODGroup group = go.GetComponent<LODGroup>();
		if (group == null) group = go.AddComponent<LODGroup>();
		LOD[] lods = new LOD[3];
		for (int i = 0; i < 3; i++) lods[i] = new LOD(LodHeights[i], new[] { byLod[i] });
		group.SetLODs(lods);
		group.fadeMode = LODFadeMode.None;
		group.RecalculateBounds();

		MeshCollider mc = go.AddComponent<MeshCollider>();
		mc.sharedMesh = byLod[2].GetComponent<MeshFilter>().sharedMesh;
		mc.convex = true;

		foreach (Transform t in go.GetComponentsInChildren<Transform>(true)) t.gameObject.layer = ObstacleLayer;

		Bounds b = byLod[0].bounds;
		Debug.Log(string.Format(System.Globalization.CultureInfo.InvariantCulture,
			"RockSetup: {0} {1:0.00} x {2:0.00} x {3:0.00} m", name, b.size.x, b.size.y, b.size.z));

		GameObject saved = PrefabUtility.SaveAsPrefabAsset(go, PrefabDir + "/" + name + ".prefab");
		Object.DestroyImmediate(go);
		return saved;
	}
}
