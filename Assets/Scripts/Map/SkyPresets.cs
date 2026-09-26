using System;
using UnityEngine;
using UnityEngine.Rendering;

//Time of day and weather: a few complete lighting setups built from HDRI skies
//(Tools > Install HDRI Sky fills them in), switched at runtime.
//
//Each preset is a sky plus the sun that sky implies: its direction (measured by
//rendering the sky), colour and strength, how hard its shadows are, and the fog
//colour at its horizon. Ambient light and reflections follow the sky
//(DynamicGI.UpdateEnvironment), so a switch relights everything consistently.
//
//The choice is saved (FarmSim_Sky); "-sky <name>" picks one for a single run,
//which is how the benchmark tour photographs each of them. "-cloud-speed 0"
//holds the cloud shadows still and "-cloud-strength 0" takes them out, for
//screenshots that compare.
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
		//shadows of the clouds in this sky, drifting with the wind; none for a
		//clear or fully overcast sky
		public Texture cloudShadows;
		public float cloudShadowSize = 600f;
	}

	//cloud shadows drift at this speed (m/s) along FarmWind's direction
	public float cloudSpeed = 6f;
	//0 leaves the sun's shadow as Unity makes it, 1 applies the full pattern
	[Range(0f, 1f)] public float cloudStrength = 1f;

	public Preset[] presets = new Preset[0];
	public Light sun;
	//FarmScreenSpaceShadows.shader; SkySetup assigns it so builds include it
	public Shader cloudShadowShader;

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

		float s;
		if (TryCommandLineFloat("-cloud-speed", out s)) cloudSpeed = s;
		if (TryCommandLineFloat("-cloud-strength", out s)) cloudStrength = Mathf.Clamp01(s);

		int index = Mathf.Clamp(PlayerPrefs.GetInt(PrefKey, 0), 0, presets.Length - 1);
		string forced = CommandLineValue("-sky");
		if (!string.IsNullOrEmpty(forced))
		{
			for (int i = 0; i < presets.Length; i++)
				if (presets[i].name.IndexOf(forced, StringComparison.OrdinalIgnoreCase) >= 0) { index = i; break; }
		}
		Apply(index);
	}

	void OnEnable()
	{
		if (Instance == this && presets.Length > 0) UpdateCloudShadows();
	}

	void OnDisable()
	{
		//the shader swap is project-wide state (in the editor it would outlive
		//play mode), so it never stays behind this component
		RestoreShadowShader();
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
		UpdateCloudShadows();
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

	//======================================================================
	// cloud shadows
	//======================================================================
	//
	//Not a cookie on the sun: in the forward path a directional light with a
	//cookie cannot be the base pass's main light, so every lit object gets drawn
	//a second time (measured at High 1080p: +2.5 ms CPU, +1.2 ms GPU). Instead
	//FarmScreenSpaceShadows.shader, a copy of Unity's screen-space shadow
	//collector, multiplies the cloud pattern into the sun's shadow: one texture
	//read in a pass that runs anyway. Receivers fade that shadow out at the
	//shadow distance, so the clouds end there too, which is why the tier table
	//(GraphicsSettings) keeps them to the tiers with a long one.

	static readonly int CloudTexId = Shader.PropertyToID("_FarmCloudShadows");
	static readonly int CloudUId = Shader.PropertyToID("_FarmCloudU");
	static readonly int CloudVId = Shader.PropertyToID("_FarmCloudV");
	static readonly int CloudStrengthId = Shader.PropertyToID("_FarmCloudStrength");

	static bool cloudsAllowed = true;

	//set by the quality tier
	public static void AllowCloudShadows(bool allowed)
	{
		if (cloudsAllowed == allowed) return;
		cloudsAllowed = allowed;
		if (Instance != null) Instance.UpdateCloudShadows();
	}

	public static bool CloudShadowsOn { get { return Instance != null && Instance.customShadowShader; } }

	bool customShadowShader;
	BuiltinShaderMode savedMode;
	Shader savedShader;
	//drift so far (m) along the light's right and up axes, where the pattern is laid out
	Vector2 cloudOffset;
	float cloudSize = 600f;

	void UpdateCloudShadows()
	{
		Preset p = presets.Length > 0 ? presets[Mathf.Clamp(Current, 0, presets.Length - 1)] : null;
		Light light = sun != null ? sun : RenderSettings.sun;
		//(isActiveAndEnabled is still false during Awake)
		bool on = cloudsAllowed && enabled && gameObject.activeInHierarchy && p != null && p.cloudShadows != null
			&& cloudShadowShader != null && cloudShadowShader.isSupported
			&& light != null && light.shadows != LightShadows.None;
		if (on)
		{
			Shader.SetGlobalTexture(CloudTexId, p.cloudShadows);
			cloudSize = Mathf.Max(1f, p.cloudShadowSize);
			if (!customShadowShader)
			{
				savedMode = UnityEngine.Rendering.GraphicsSettings.GetShaderMode(BuiltinShaderType.ScreenSpaceShadows);
				savedShader = UnityEngine.Rendering.GraphicsSettings.GetCustomShader(BuiltinShaderType.ScreenSpaceShadows);
				UnityEngine.Rendering.GraphicsSettings.SetCustomShader(BuiltinShaderType.ScreenSpaceShadows, cloudShadowShader);
				UnityEngine.Rendering.GraphicsSettings.SetShaderMode(BuiltinShaderType.ScreenSpaceShadows, BuiltinShaderMode.UseCustom);
				customShadowShader = true;
			}
			UpdateCloudDrift(0f);
		}
		else RestoreShadowShader();
		Shader.SetGlobalFloat(CloudStrengthId, on ? cloudStrength : 0f);
	}

	void RestoreShadowShader()
	{
		if (!customShadowShader) return;
		UnityEngine.Rendering.GraphicsSettings.SetShaderMode(BuiltinShaderType.ScreenSpaceShadows, savedMode);
		UnityEngine.Rendering.GraphicsSettings.SetCustomShader(BuiltinShaderType.ScreenSpaceShadows, savedShader);
		customShadowShader = false;
		Shader.SetGlobalFloat(CloudStrengthId, 0f);
	}

	void Update()
	{
		if (customShadowShader) UpdateCloudDrift(Time.deltaTime);
	}

	void UpdateCloudDrift(float dt)
	{
		Light light = sun != null ? sun : RenderSettings.sun;
		if (light == null) return;
		Transform t = light.transform;
		Vector3 wind = FarmWind.direction;
		wind.y = 0f;
		if (dt > 0f && cloudSpeed != 0f && wind.sqrMagnitude > 1e-6f)
		{
			wind = wind.normalized * (cloudSpeed * dt);
			cloudOffset += new Vector2(Vector3.Dot(wind, t.right), Vector3.Dot(wind, t.up));
			//the pattern repeats every cloudSize along both axes, so wrapping
			//there is seamless and keeps the numbers small
			cloudOffset.x = Mathf.Repeat(cloudOffset.x, cloudSize);
			cloudOffset.y = Mathf.Repeat(cloudOffset.y, cloudSize);
		}
		//the shader's uv: position along each axis minus the drift, in pattern
		//repeats; projecting along the light is what a cookie would have done
		float k = 1f / cloudSize;
		Vector3 r = t.right * k, u = t.up * k;
		Shader.SetGlobalVector(CloudUId, new Vector4(r.x, r.y, r.z, -cloudOffset.x * k));
		Shader.SetGlobalVector(CloudVId, new Vector4(u.x, u.y, u.z, -cloudOffset.y * k));
	}

	static string CommandLineValue(string flag)
	{
		string[] args = Environment.GetCommandLineArgs();
		for (int i = 0; i + 1 < args.Length; i++)
			if (string.Equals(args[i], flag, StringComparison.OrdinalIgnoreCase)) return args[i + 1];
		return null;
	}

	static bool TryCommandLineFloat(string flag, out float value)
	{
		value = 0f;
		string text = CommandLineValue(flag);
		return !string.IsNullOrEmpty(text) && float.TryParse(text, System.Globalization.NumberStyles.Float,
			System.Globalization.CultureInfo.InvariantCulture, out value);
	}
}
