using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

//Swaps the robots' visible meshes for the bevelled, UV-mapped copies that
//tools/robot_polish.py writes to Assets/Models/Robots.
//
//Only MeshFilters change. Colliders keep the original meshes, so the physics of
//every robot is exactly what it was, and skinned parts (the snipper's blend
//shape) are not touched. A swap only happens when the polished mesh has the same
//name, the same submesh count (so materials stay on the right parts) and the same
//bounds as the one it replaces; anything else is logged and left alone, which is
//the guard against a coordinate-convention slip putting a wheel sideways.
//
//Tools > Swap In Polished Robot Meshes, or -executeMethod RobotPolishSetup.Apply
public static class RobotPolishSetup
{
	const string PolishedDir = "Assets/Models/Robots";
	static readonly string[] PrefabDirs = { "Assets/Robots", "Assets/RobotSelects" };

	[MenuItem("Tools/Swap In Polished Robot Meshes")]
	public static void Apply()
	{
		//source model path -> candidate files, each (mesh name -> polished mesh);
		//a multi-material OBJ has a second, single-material candidate
		Dictionary<string, List<Dictionary<string, Mesh>>> polished = new Dictionary<string, List<Dictionary<string, Mesh>>>();
		foreach (string file in Directory.GetFiles(PolishedDir))
		{
			if (file.EndsWith(".meta") || file.EndsWith(".mtl")) continue;
			string path = file.Replace('\\', '/');
			ModelImporter mi = AssetImporter.GetAtPath(path) as ModelImporter;
			if (mi == null) continue;
			bool dirty = mi.importNormals != ModelImporterNormals.Import
				|| mi.importTangents != ModelImporterTangents.CalculateMikk
				|| mi.materialImportMode != ModelImporterMaterialImportMode.None
				|| mi.importAnimation;
			mi.importNormals = ModelImporterNormals.Import;
			mi.importTangents = ModelImporterTangents.CalculateMikk;
			//the prefabs keep their own materials; nothing to import
			mi.materialImportMode = ModelImporterMaterialImportMode.None;
			mi.importAnimation = false;
			mi.addCollider = false;
			if (dirty) mi.SaveAndReimport();

			string name = Path.GetFileNameWithoutExtension(path);
			string ext = Path.GetExtension(path);
			string source = "Assets/Models/" + name.Replace("_polished_single", "").Replace("_polished", "") + ext;
			Dictionary<string, Mesh> byName = new Dictionary<string, Mesh>();
			foreach (Mesh m in AssetDatabase.LoadAllAssetsAtPath(path).OfType<Mesh>()) byName[m.name] = m;
			List<Dictionary<string, Mesh>> list;
			if (!polished.TryGetValue(source, out list)) polished[source] = list = new List<Dictionary<string, Mesh>>();
			list.Add(byName);
		}

		int swapped = 0, refused = 0, prefabsChanged = 0;
		foreach (string dir in PrefabDirs)
		{
			foreach (string file in Directory.GetFiles(dir, "*.prefab"))
			{
				string path = file.Replace('\\', '/');
				GameObject root = PrefabUtility.LoadPrefabContents(path);
				int here = 0;
				foreach (MeshFilter mf in root.GetComponentsInChildren<MeshFilter>(true))
				{
					Mesh old = mf.sharedMesh;
					if (old == null) continue;
					List<Dictionary<string, Mesh>> candidates;
					if (!polished.TryGetValue(AssetDatabase.GetAssetPath(old), out candidates)) continue;
					Mesh chosen = null;
					string why = "no polished mesh named " + old.name;
					foreach (Dictionary<string, Mesh> byName in candidates)
					{
						Mesh fresh;
						if (!byName.TryGetValue(old.name, out fresh)) continue;
						if (fresh == old) { chosen = old; break; }
						string mismatch = Mismatch(old, fresh);
						if (mismatch == null) { chosen = fresh; break; }
						why = mismatch;
					}
					if (chosen == old) continue;
					if (chosen == null)
					{
						Debug.LogWarning("RobotPolishSetup: kept " + old.name + " in " + path + " (" + mf.name + "): " + why);
						refused++;
						continue;
					}
					mf.sharedMesh = chosen;
					here++;
				}
				if (here > 0)
				{
					PrefabUtility.SaveAsPrefabAsset(root, path);
					prefabsChanged++;
					swapped += here;
					Debug.Log("RobotPolishSetup: " + path + ": " + here + " meshes swapped");
				}
				PrefabUtility.UnloadPrefabContents(root);
			}
		}
		AssetDatabase.SaveAssets();
		Debug.Log("RobotPolishSetup: " + swapped + " meshes swapped in " + prefabsChanged + " prefabs, " + refused + " refused");
	}

	//null when the polished mesh can stand in for the original
	static string Mismatch(Mesh old, Mesh fresh)
	{
		if (old.subMeshCount != fresh.subMeshCount)
			return "submeshes " + old.subMeshCount + " vs " + fresh.subMeshCount;
		Bounds a = old.bounds, b = fresh.bounds;
		//bevels cut corners inward; the box around the part stays put
		float tol = 0.02f * a.size.magnitude + 0.002f;
		if ((a.center - b.center).magnitude > tol) return "centre " + a.center.ToString("F3") + " vs " + b.center.ToString("F3");
		for (int i = 0; i < 3; i++)
			if (Mathf.Abs(a.size[i] - b.size[i]) > tol) return "size " + a.size.ToString("F3") + " vs " + b.size.ToString("F3");
		return null;
	}
}
