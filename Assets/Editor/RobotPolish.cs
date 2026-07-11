using UnityEngine;
using UnityEditor;

//Gives the robot materials physically sensible metal/rubber/paint values so
//the machines read as built hardware instead of flat untextured plastic.
public static class RobotPolish
{
	struct Spec { public string path; public float metallic; public float smoothness; public Spec(string p, float m, float s) { path = p; metallic = m; smoothness = s; } }

	static readonly Spec[] Specs =
	{
		new Spec("Assets/Materials/Aluminium.mat", 0.85f, 0.60f),
		new Spec("Assets/Materials/JaguarMetal.mat", 0.80f, 0.55f),
		new Spec("Assets/Materials/JaguarShiny.mat", 0.90f, 0.75f),
		new Spec("Assets/Materials/JaguarBody.mat", 0.30f, 0.55f),
		new Spec("Assets/Materials/LampRover.mat", 0.25f, 0.50f),
		new Spec("Assets/Materials/LongRover.mat", 0.25f, 0.50f),
		new Spec("Assets/Materials/Test1.mat", 0.25f, 0.45f),
		new Spec("Assets/Materials/Wheel.mat", 0.00f, 0.22f),
		new Spec("Assets/Materials/HoseMesh.mat", 0.00f, 0.30f),
		new Spec("Assets/Materials/HumanoidBlue.mat", 0.45f, 0.65f),
		new Spec("Assets/Materials/HumanoidGrey.mat", 0.55f, 0.60f),
	};

	[MenuItem("Tools/Improve Robot Materials")]
	public static void Improve()
	{
		Shader standard = Shader.Find("Standard");
		int changed = 0;
		foreach (Spec s in Specs)
		{
			Material m = AssetDatabase.LoadAssetAtPath<Material>(s.path);
			if (m == null) { Debug.LogWarning("RobotPolish: missing " + s.path); continue; }

			if (m.shader != standard)
			{
				Texture tex = m.mainTexture;
				Color col = m.HasProperty("_Color") ? m.color : Color.white;
				m.shader = standard;
				m.mainTexture = tex;
				m.color = col;
			}
			if (m.HasProperty("_Metallic")) m.SetFloat("_Metallic", s.metallic);
			if (m.HasProperty("_Glossiness")) m.SetFloat("_Glossiness", s.smoothness);
			EditorUtility.SetDirty(m);
			changed++;
		}
		AssetDatabase.SaveAssets();
		Debug.Log("RobotPolish: tuned " + changed + " robot materials.");
	}
}
