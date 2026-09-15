using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine.Rendering.PostProcessing;

//Adds bloom, subtle color grading, and FXAA to the main camera so lasers,
//UV light, and the sky glow properly instead of rendering flat.
//
//The effect settings are stored as sub-assets of the profile (the first
//version of this tool forgot that, so the profile shipped empty) and the
//volume references the shared profile. GraphicsSettings turns the layer,
//bloom and HDR off on the low quality tiers at runtime.
public static class BloomSetup
{
	[MenuItem("Tools/Setup Post Processing")]
	public static void Setup()
	{
		const string profilePath = "Assets/Terrain/FarmPostProfile.asset";

		//shared profile asset
		PostProcessProfile profile = AssetDatabase.LoadAssetAtPath<PostProcessProfile>(profilePath);
		if (profile == null)
		{
			profile = ScriptableObject.CreateInstance<PostProcessProfile>();
			AssetDatabase.CreateAsset(profile, profilePath);
		}
		//drop null slots left behind by the earlier version of this tool
		profile.settings.RemoveAll(x => x == null);

		Bloom bloom = profile.HasSettings<Bloom>() ? profile.GetSetting<Bloom>() : profile.AddSettings<Bloom>();
		bloom.enabled.Override(true);
		bloom.intensity.Override(2.6f);
		bloom.threshold.Override(1.05f);
		bloom.softKnee.Override(0.55f);
		Persist(bloom, profile);

		ColorGrading grading = profile.HasSettings<ColorGrading>() ? profile.GetSetting<ColorGrading>() : profile.AddSettings<ColorGrading>();
		grading.enabled.Override(true);
		grading.saturation.Override(8f);
		grading.contrast.Override(6f);
		grading.postExposure.Override(0.15f);
		Persist(grading, profile);

		EditorUtility.SetDirty(profile);

		//global volume
		PostProcessVolume volume = Object.FindObjectOfType<PostProcessVolume>();
		if (volume == null)
		{
			GameObject vgo = new GameObject("PostProcessVolume");
			volume = vgo.AddComponent<PostProcessVolume>();
		}
		volume.isGlobal = true;
		volume.sharedProfile = profile;
		volume.gameObject.layer = 0;
		EditorUtility.SetDirty(volume);

		//layer on every camera that renders to the screen; robot cameras draw
		//into small render textures and do not need post effects
		int cameras = 0;
		foreach (Camera cam in Object.FindObjectsOfType<Camera>(true))
		{
			if (cam.targetTexture != null) continue;
			PostProcessLayer layer = cam.GetComponent<PostProcessLayer>();
			if (layer == null) layer = cam.gameObject.AddComponent<PostProcessLayer>();
			layer.volumeLayer = 1; //Default layer mask
			layer.volumeTrigger = cam.transform;
			layer.antialiasingMode = PostProcessLayer.Antialiasing.FastApproximateAntialiasing;
			//NaN guard is a debugging aid that costs a full-screen pass every frame
			layer.stopNaNPropagation = false;
			EditorUtility.SetDirty(cam.gameObject);
			cameras++;
		}

		EditorSceneManager.MarkAllScenesDirty();
		EditorSceneManager.SaveOpenScenes();
		AssetDatabase.SaveAssets();
		Debug.Log("BloomSetup: profile applied, global volume set, post layer on " + cameras + " cameras.");
	}

	//Effect settings live inside the profile asset file; without this they
	//are lost the moment the asset database reloads
	static void Persist(PostProcessEffectSettings settings, PostProcessProfile profile)
	{
		if (settings == null || AssetDatabase.Contains(settings)) return;
		settings.hideFlags = HideFlags.HideInHierarchy;
		AssetDatabase.AddObjectToAsset(settings, profile);
	}
}
