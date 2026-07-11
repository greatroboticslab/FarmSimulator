using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.Rendering.PostProcessing;

//Adds bloom, subtle color grading, and FXAA to the main camera so lasers,
//UV light, and the sky glow properly instead of rendering flat.
public static class BloomSetup
{
	[MenuItem("Tools/Setup Post Processing")]
	public static void Setup()
	{
		//shared profile asset
		PostProcessProfile profile = AssetDatabase.LoadAssetAtPath<PostProcessProfile>("Assets/Terrain/FarmPostProfile.asset");
		if (profile == null)
		{
			profile = ScriptableObject.CreateInstance<PostProcessProfile>();
			AssetDatabase.CreateAsset(profile, "Assets/Terrain/FarmPostProfile.asset");
		}

		Bloom bloom = profile.HasSettings<Bloom>() ? profile.GetSetting<Bloom>() : profile.AddSettings<Bloom>();
		bloom.enabled.Override(true);
		bloom.intensity.Override(2.6f);
		bloom.threshold.Override(1.05f);
		bloom.softKnee.Override(0.55f);

		ColorGrading grading = profile.HasSettings<ColorGrading>() ? profile.GetSetting<ColorGrading>() : profile.AddSettings<ColorGrading>();
		grading.enabled.Override(true);
		grading.saturation.Override(8f);
		grading.contrast.Override(6f);
		grading.postExposure.Override(0.15f);

		EditorUtility.SetDirty(profile);

		//global volume
		PostProcessVolume volume = Object.FindObjectOfType<PostProcessVolume>();
		if (volume == null)
		{
			GameObject vgo = new GameObject("PostProcessVolume");
			volume = vgo.AddComponent<PostProcessVolume>();
		}
		volume.isGlobal = true;
		volume.profile = profile;
		volume.gameObject.layer = 0;

		//layer on every camera that renders the world
		int cameras = 0;
		foreach (Camera cam in Object.FindObjectsOfType<Camera>(true))
		{
			PostProcessLayer layer = cam.GetComponent<PostProcessLayer>();
			if (layer == null) layer = cam.gameObject.AddComponent<PostProcessLayer>();
			layer.volumeLayer = 1; //Default layer mask
			layer.volumeTrigger = cam.transform;
			layer.antialiasingMode = PostProcessLayer.Antialiasing.FastApproximateAntialiasing;
			EditorUtility.SetDirty(cam.gameObject);
			cameras++;
		}

		EditorSceneManager.MarkAllScenesDirty();
		EditorSceneManager.SaveOpenScenes();
		AssetDatabase.SaveAssets();
		Debug.Log("BloomSetup: profile applied, global volume set, post layer on " + cameras + " cameras.");
	}
}
