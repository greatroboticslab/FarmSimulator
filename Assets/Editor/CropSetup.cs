using System.Collections.Generic;
using System.Linq;
using UnityEditor;
using UnityEngine;

//Puts the generated crop plants (tools/crop_gen.py, textures from
//tools/build_crop_textures.py) into the crop prefabs.
//
//The prefabs keep their structure, scripts and colliders: only the MeshFilters
//that drew the old models get the new meshes, and the crop's existing materials
//(already FarmSim/Foliage) get the new atlas, normal map, translucency, vertex
//occlusion and a little wind. Nothing else references these models or materials.
//
//Tools > Build Crop Plants, or -executeMethod CropSetup.Build
public static class CropSetup
{
	class Crop
	{
		public string prefab, model;
		//old model file -> mesh name in the new model
		public Dictionary<string, string> meshes;
		//material path -> atlas name in Assets/Textures/Crops
		public Dictionary<string, string> materials;
	}

	static readonly Crop[] Crops =
	{
		new Crop
		{
			prefab = "Assets/Plants/soybean.prefab",
			model = "Assets/Models/Crops/Soybean.fbx",
			meshes = new Dictionary<string, string>
			{
				{ "Assets/Models/soybean.obj", "Soybean" },
				{ "Assets/Models/soybean_dead.obj", "SoybeanDead" },
			},
			materials = new Dictionary<string, string>
			{
				{ "Assets/Materials/Soybean.mat", "Soybean" },
				{ "Assets/Materials/Soybean_dead.mat", "SoybeanDead" },
			},
		},
		//the four ears are FruitBody objects the grabber harvests; only their
		//meshes change, their rigidbodies and colliders stay
		new Crop
		{
			prefab = "Assets/Plants/corn_stalk.prefab",
			model = "Assets/Models/Crops/Corn.fbx",
			meshes = new Dictionary<string, string>
			{
				{ "Assets/Models/corn_stalk.obj", "CornStalk" },
				{ "Assets/Models/corn_cob.obj", "CornEar" },
			},
			materials = new Dictionary<string, string>
			{
				{ "Assets/Materials/CornStalk.mat", "Corn" },
				{ "Assets/Materials/CornCob.mat", "Corn" },
			},
		},
	};

	[MenuItem("Tools/Build Crop Plants")]
	public static void Build()
	{
		foreach (Crop crop in Crops)
		{
			foreach (string atlas in crop.materials.Values.Distinct())
			{
				string albedo = "Assets/Textures/Crops/" + atlas + "_Albedo.png";
				TextureImporter ta = AssetImporter.GetAtPath(albedo) as TextureImporter;
				if (ta != null)
				{
					ta.alphaIsTransparency = true;
					ta.mipMapsPreserveCoverage = true;
					ta.alphaTestReferenceValue = 0.35f;
					ta.wrapMode = TextureWrapMode.Clamp;
					ta.maxTextureSize = 1024;
					ta.SaveAndReimport();
				}
				TextureImporter tn = AssetImporter.GetAtPath("Assets/Textures/Crops/" + atlas + "_Normal.png") as TextureImporter;
				if (tn != null)
				{
					tn.textureType = TextureImporterType.NormalMap;
					tn.wrapMode = TextureWrapMode.Clamp;
					tn.maxTextureSize = 1024;
					tn.SaveAndReimport();
				}
			}

			ModelImporter mi = (ModelImporter)AssetImporter.GetAtPath(crop.model);
			mi.importAnimation = false;
			mi.animationType = ModelImporterAnimationType.None;
			mi.importNormals = ModelImporterNormals.Import;
			mi.importTangents = ModelImporterTangents.CalculateMikk;
			mi.materialImportMode = ModelImporterMaterialImportMode.None;
			mi.addCollider = false;
			mi.isReadable = false;
			mi.SaveAndReimport();
			Dictionary<string, Mesh> fresh = AssetDatabase.LoadAllAssetsAtPath(crop.model).OfType<Mesh>().ToDictionary(m => m.name);

			foreach (KeyValuePair<string, string> kv in crop.materials)
			{
				Material mat = AssetDatabase.LoadAssetAtPath<Material>(kv.Key);
				if (mat == null) { Debug.LogWarning("CropSetup: no material " + kv.Key); continue; }
				mat.shader = Shader.Find("FarmSim/Foliage");
				mat.SetTexture("_MainTex", AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Textures/Crops/" + kv.Value + "_Albedo.png"));
				mat.SetTexture("_BumpMap", AssetDatabase.LoadAssetAtPath<Texture2D>("Assets/Textures/Crops/" + kv.Value + "_Normal.png"));
				mat.SetColor("_Color", Color.white);
				mat.SetFloat("_Cutoff", 0.35f);
				mat.SetFloat("_Translucency", 0.5f);
				mat.SetColor("_TranslucencyColor", new Color(0.78f, 0.86f, 0.40f));
				mat.SetFloat("_OcclusionFromVertex", 1f);
				//crops sway less than tree crowns: about 5 cm at the leaf tips
				mat.SetFloat("_WindWeight", 0.35f);
				mat.enableInstancing = true;
				EditorUtility.SetDirty(mat);
			}

			GameObject root = PrefabUtility.LoadPrefabContents(crop.prefab);
			int swapped = 0;
			foreach (MeshFilter mf in root.GetComponentsInChildren<MeshFilter>(true))
			{
				if (mf.sharedMesh == null) continue;
				string name;
				if (!crop.meshes.TryGetValue(AssetDatabase.GetAssetPath(mf.sharedMesh), out name)) continue;
				Mesh m;
				if (!fresh.TryGetValue(name, out m)) { Debug.LogWarning("CropSetup: no mesh " + name + " in " + crop.model); continue; }
				mf.sharedMesh = m;
				swapped++;
			}
			PrefabUtility.SaveAsPrefabAsset(root, crop.prefab);
			PrefabUtility.UnloadPrefabContents(root);
			Debug.Log("CropSetup: " + crop.prefab + ": " + swapped + " meshes swapped to " + crop.model
				+ " (" + string.Join(", ", fresh.Values.Select(m => m.name + " " + (m.triangles.Length / 3) + " tris").ToArray()) + ")");
		}
		AssetDatabase.SaveAssets();
	}
}
