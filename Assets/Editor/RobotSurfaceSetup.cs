using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

//Step 3 of the robot pass: every polished robot part gets its baked detail mask
//(tools/robot_bake.py, robot_masks.py) through FarmSim/RobotSurface.
//
//For each renderer drawing a polished mesh, each material slot is replaced by a
//variant of the material it had: same paint colour, metallic, smoothness and
//world-projected detail, plus that part's mask. Variants are shared by every
//prefab that uses the same material on the same part, and live in
//Assets/Materials/RobotSurface. Rubber (dark, non-metallic) gets dust instead of
//bare-metal edges.
//
//Tools > Apply Robot Surface Detail, or -executeMethod RobotSurfaceSetup.Apply
public static class RobotSurfaceSetup
{
	const string PolishedDir = "Assets/Models/Robots";
	const string MaskDir = "Assets/Textures/Robots";
	const string MatDir = "Assets/Materials/RobotSurface";
	static readonly string[] PrefabDirs = { "Assets/Robots", "Assets/RobotSelects" };

	[MenuItem("Tools/Apply Robot Surface Detail")]
	public static void Apply()
	{
		Directory.CreateDirectory(MatDir);
		AssetDatabase.Refresh();

		foreach (string png in Directory.GetFiles(MaskDir, "*_Mask.png"))
		{
			TextureImporter ti = AssetImporter.GetAtPath(png.Replace('\\', '/')) as TextureImporter;
			if (ti == null) continue;
			//data, not colour
			if (ti.sRGBTexture || ti.maxTextureSize != 1024)
			{
				ti.sRGBTexture = false;
				ti.maxTextureSize = 1024;
				ti.SaveAndReimport();
			}
		}

		Shader shader = Shader.Find("FarmSim/RobotSurface");
		Dictionary<string, Material> variants = new Dictionary<string, Material>();
		int renderers = 0, prefabsChanged = 0, missing = 0;

		foreach (string dir in PrefabDirs)
		{
			foreach (string file in Directory.GetFiles(dir, "*.prefab"))
			{
				string path = file.Replace('\\', '/');
				GameObject root = PrefabUtility.LoadPrefabContents(path);
				int here = 0;
				foreach (MeshRenderer mr in root.GetComponentsInChildren<MeshRenderer>(true))
				{
					MeshFilter mf = mr.GetComponent<MeshFilter>();
					if (mf == null || mf.sharedMesh == null) continue;
					string meshPath = AssetDatabase.GetAssetPath(mf.sharedMesh);
					if (!meshPath.StartsWith(PolishedDir)) continue;

					Texture2D mask = FindMask(meshPath, mf.sharedMesh);
					if (mask == null)
					{
						Debug.LogWarning("RobotSurfaceSetup: no mask for " + mf.sharedMesh.name + " from " + meshPath);
						missing++;
						continue;
					}

					Material[] mats = mr.sharedMaterials;
					bool changed = false;
					for (int i = 0; i < mats.Length; i++)
					{
						Material m = mats[i];
						if (m == null || m.shader == shader) continue;
						string key = m.name + "__" + mask.name;
						Material v;
						if (!variants.TryGetValue(key, out v))
						{
							v = Variant(m, mask, shader, key);
							variants[key] = v;
						}
						mats[i] = v;
						changed = true;
					}
					if (changed)
					{
						mr.sharedMaterials = mats;
						here++;
					}
				}
				if (here > 0)
				{
					PrefabUtility.SaveAsPrefabAsset(root, path);
					prefabsChanged++;
					renderers += here;
				}
				PrefabUtility.UnloadPrefabContents(root);
			}
		}
		AssetDatabase.SaveAssets();
		Debug.Log("RobotSurfaceSetup: " + renderers + " renderers in " + prefabsChanged + " prefabs, "
			+ variants.Count + " material variants, " + missing + " parts without a mask");
	}

	//masks are named <file>_<ext>__<object>_Mask; an OBJ holds one object, whose
	//mesh Unity calls "default", so there the file's only mask is the one
	static Texture2D FindMask(string meshPath, Mesh mesh)
	{
		string file = Path.GetFileNameWithoutExtension(meshPath).Replace("_polished_single", "").Replace("_polished", "");
		string ext = Path.GetExtension(meshPath).TrimStart('.');
		string prefix = file + "_" + ext + "__";
		string[] candidates = Directory.GetFiles(MaskDir, prefix + "*_Mask.png").Select(p => p.Replace('\\', '/')).ToArray();
		string exact = MaskDir + "/" + prefix + mesh.name + "_Mask.png";
		string chosen = candidates.Contains(exact) ? exact : (candidates.Length == 1 ? candidates[0] : null);
		return chosen != null ? AssetDatabase.LoadAssetAtPath<Texture2D>(chosen) : null;
	}

	static Material Variant(Material source, Texture2D mask, Shader shader, string key)
	{
		string path = MatDir + "/" + key + ".mat";
		Material v = AssetDatabase.LoadAssetAtPath<Material>(path);
		if (v == null)
		{
			v = new Material(shader);
			AssetDatabase.CreateAsset(v, path);
		}
		v.shader = shader;
		Color c = source.HasProperty("_Color") ? source.GetColor("_Color") : Color.white;
		v.SetColor("_Color", c);
		if (source.HasProperty("_MainTex") && source.GetTexture("_MainTex") != null) v.SetTexture("_MainTex", source.GetTexture("_MainTex"));
		float metallic = source.HasProperty("_Metallic") ? source.GetFloat("_Metallic") : 0.3f;
		v.SetFloat("_Metallic", metallic);
		v.SetFloat("_Glossiness", source.HasProperty("_Glossiness") ? source.GetFloat("_Glossiness") : 0.5f);
		if (source.HasProperty("_DetailTex") && source.GetTexture("_DetailTex") != null)
		{
			v.SetTexture("_DetailTex", source.GetTexture("_DetailTex"));
			v.SetFloat("_DetailScale", source.GetFloat("_DetailScale"));
			v.SetFloat("_DetailStrength", source.GetFloat("_DetailStrength"));
		}
		v.SetTexture("_MaskTex", mask);

		//dark, non-metallic surfaces here are tyres: edges and treads gather dust,
		//there is no bare metal under rubber
		bool rubber = metallic < 0.1f && c.grayscale < 0.15f;
		if (rubber)
		{
			v.SetColor("_WearColor", new Color(0.33f, 0.30f, 0.26f));
			v.SetFloat("_WearMetallic", 0f);
			v.SetFloat("_WearSmoothness", 0.08f);
			v.SetFloat("_WearStrength", 0.55f);
			v.SetColor("_GrimeColor", new Color(0.36f, 0.31f, 0.24f));
			v.SetFloat("_GrimeStrength", 0.75f);
		}
		else
		{
			v.SetColor("_WearColor", new Color(0.78f, 0.78f, 0.76f));
			v.SetFloat("_WearMetallic", 1f);
			v.SetFloat("_WearSmoothness", 0.55f);
			v.SetFloat("_WearStrength", 0.8f);
			v.SetColor("_GrimeColor", new Color(0.24f, 0.21f, 0.17f));
			v.SetFloat("_GrimeStrength", 0.6f);
		}
		v.enableInstancing = true;
		EditorUtility.SetDirty(v);
		return v;
	}
}
