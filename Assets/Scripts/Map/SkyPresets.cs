using System;
using UnityEngine;

//Time of day and weather: a few complete lighting setups built from HDRI skies
//(Tools > Install HDRI Sky fills them in), switched at runtime.
//
//Each preset is a sky plus the sun that sky implies: its direction (measured by
//rendering the sky), colour and strength, how hard its shadows are, and the fog
//colour at its horizon. Ambient light and reflections follow the sky
//(DynamicGI.UpdateEnvironment), so a switch relights everything consistently.
//
//The choice is saved (FarmSim_Sky); "-sky <name>" picks one for a single run,
//which is how the benchmark tour photographs each of them.
public class SkyPresets : MonoBehaviour
{
	[Serializable]
	public class Preset
	{
		public string name;
		public Material skybox;
		public Vector3 sunEuler;
		//as the Light component takes them: gamma colour and intensity
		public Color sunColor = Color.white;
		public float sunIntensity = 1f;
		public float shadowStrength = 1f;
		//gamma, like RenderSettings.fogColor
		public Color fogColor = Color.grey;
	}

	public Preset[] presets = new Preset[0];
	public Light sun;

	const string PrefKey = "FarmSim_Sky";

	public static SkyPresets Instance { get; private set; }
	public static int Current { get; private set; }

	public static string CurrentName
	{
		get
		{
			if (Instance == null || Instance.presets.Length == 0) return "Default";
			return Instance.presets[Mathf.Clamp(Current, 0, Instance.presets.Length - 1)].name;
		}
	}

	void Awake()
	{
		Instance = this;
		if (presets.Length == 0) return;

		int index = Mathf.Clamp(PlayerPrefs.GetInt(PrefKey, 0), 0, presets.Length - 1);
		string forced = CommandLineValue("-sky");
		if (!string.IsNullOrEmpty(forced))
		{
			for (int i = 0; i < presets.Length; i++)
				if (presets[i].name.IndexOf(forced, StringComparison.OrdinalIgnoreCase) >= 0) { index = i; break; }
		}
		Apply(index);
	}

	void OnDestroy()
	{
		if (Instance == this) Instance = null;
	}

	public void Apply(int index)
	{
		if (presets.Length == 0) return;
		index = Mathf.Clamp(index, 0, presets.Length - 1);
		Preset p = presets[index];
		Current = index;

		if (p.skybox != null) RenderSettings.skybox = p.skybox;
		Light light = sun != null ? sun : RenderSettings.sun;
		if (light != null)
		{
			light.transform.rotation = Quaternion.Euler(p.sunEuler);
			light.color = p.sunColor;
			light.intensity = p.sunIntensity;
			light.shadowStrength = p.shadowStrength;
			//an overcast sky has no sun to cast shadows with
			light.shadows = p.shadowStrength > 0.01f ? LightShadows.Soft : LightShadows.None;
		}
		RenderSettings.fogColor = p.fogColor;
		DynamicGI.UpdateEnvironment();
	}

	//Next preset, remembered for the next run
	public static void Cycle()
	{
		if (Instance == null || Instance.presets.Length == 0) return;
		int next = (Current + 1) % Instance.presets.Length;
		Instance.Apply(next);
		PlayerPrefs.SetInt(PrefKey, next);
		PlayerPrefs.Save();
	}

	static string CommandLineValue(string flag)
	{
		string[] args = Environment.GetCommandLineArgs();
		for (int i = 0; i + 1 < args.Length; i++)
			if (string.Equals(args[i], flag, StringComparison.OrdinalIgnoreCase)) return args[i + 1];
		return null;
	}
}
