using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

//One-shot lighting and atmosphere pass for the main scene: warm sun,
//distance fog, procedural sky, and richer ambient light so the farm
//reads as an outdoor scene instead of flat default lighting.
public static class ScenePolish
{
	[MenuItem("Tools/Polish Scene Lighting")]
	public static void Polish()
	{
		//sun
		Light sun = RenderSettings.sun;
		if (sun == null)
		{
			foreach (Light l in Object.FindObjectsOfType<Light>())
			{
				if (l.type == LightType.Directional) { sun = l; break; }
			}
		}
		if (sun != null)
		{
			sun.color = new Color(1f, 0.956f, 0.878f);
			sun.intensity = 1.15f;
			sun.shadows = LightShadows.Soft;
			sun.shadowStrength = 0.82f;
			sun.transform.rotation = Quaternion.Euler(52f, 205f, 0f);
			RenderSettings.sun = sun;
			EditorUtility.SetDirty(sun);
			EditorUtility.SetDirty(sun.transform);
		}

		//sky
		Material sky = AssetDatabase.LoadAssetAtPath<Material>("Assets/Terrain/FarmSkybox.mat");
		if (sky == null)
		{
			sky = new Material(Shader.Find("Skybox/Procedural"));
			AssetDatabase.CreateAsset(sky, "Assets/Terrain/FarmSkybox.mat");
		}
		sky.SetFloat("_SunSize", 0.04f);
		sky.SetFloat("_AtmosphereThickness", 0.95f);
		sky.SetColor("_SkyTint", new Color(0.45f, 0.6f, 0.82f));
		sky.SetColor("_GroundColor", new Color(0.4f, 0.42f, 0.44f));
		sky.SetFloat("_Exposure", 1.15f);
		EditorUtility.SetDirty(sky);
		RenderSettings.skybox = sky;

		//fog and ambient
		RenderSettings.fog = true;
		RenderSettings.fogMode = FogMode.Linear;
		RenderSettings.fogStartDistance = 160f;
		RenderSettings.fogEndDistance = 950f;
		RenderSettings.fogColor = new Color(0.75f, 0.81f, 0.88f);

		RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
		RenderSettings.ambientSkyColor = new Color(0.60f, 0.70f, 0.84f);
		RenderSettings.ambientEquatorColor = new Color(0.55f, 0.58f, 0.52f);
		RenderSettings.ambientGroundColor = new Color(0.34f, 0.32f, 0.27f);

		QualitySettings.shadowDistance = 170f;

		EditorSceneManager.MarkAllScenesDirty();
		EditorSceneManager.SaveOpenScenes();
		AssetDatabase.SaveAssets();
		Debug.Log("ScenePolish: lighting, fog, sky, and ambient applied and scene saved.");
	}
}
