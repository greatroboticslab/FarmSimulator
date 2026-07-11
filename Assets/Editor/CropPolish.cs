using UnityEngine;
using UnityEditor;

//Switches every crop/weed material to the two-sided FarmSim/Foliage shader
//so leaves render from both sides, keeping each material's texture and tint.
public static class CropPolish
{
	static readonly string[] CropMaterials =
	{
		"Assets/Materials/Soybean.mat",
		"Assets/Materials/Soybean_dead.mat",
		"Assets/Materials/Dandelion.mat",
		"Assets/Materials/Dandelion_dead.mat",
		"Assets/Materials/BellPepper.mat",
		"Assets/Materials/PepperPlant.mat",
		"Assets/Materials/CornCob.mat",
		"Assets/Materials/CornStalk.mat",
		"Assets/Materials/Tomato.mat",
		"Assets/Materials/TomatoPlant.mat",
		"Assets/Materials/VGGT/strawberryplant/Strawberry.mat",
		"Assets/Materials/VGGT/strawberryplant/StrawberryPlant.mat",
	};

	[MenuItem("Tools/Improve Crop Materials")]
	public static void Improve()
	{
		Shader foliage = Shader.Find("FarmSim/Foliage");
		if (foliage == null) { Debug.LogError("CropPolish: FarmSim/Foliage shader not found."); return; }

		int changed = 0;
		foreach (string path in CropMaterials)
		{
			Material m = AssetDatabase.LoadAssetAtPath<Material>(path);
			if (m == null) { Debug.LogWarning("CropPolish: missing " + path); continue; }

			Texture tex = m.mainTexture;
			Color col = m.HasProperty("_Color") ? m.color : Color.white;
			m.shader = foliage;
			m.mainTexture = tex;
			m.color = col;
			m.SetFloat("_Cutoff", 0.35f);
			EditorUtility.SetDirty(m);
			changed++;
		}
		AssetDatabase.SaveAssets();
		Debug.Log("CropPolish: switched " + changed + " materials to FarmSim/Foliage.");
	}
}
