using System.Collections.Generic;
using System.Globalization;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

//Installs the HDRI skies in the main scene as time-of-day / weather presets
//(SkyPresets on the sun), and applies the first one:
// - imports each lat-long HDR as a cubemap and builds a Skybox/Cubemap material
// - finds where each sky's sun actually is by rendering the sky, and rotates the
//   sky so that sun sits behind the scene's light at the farm's usual compass
//   heading; the light takes the sky's elevation, colour and intensity
// - ambient light and reflections come from the sky; fog takes its horizon
//
//The photometric numbers come from tools/hdri_sun.py on each ORIGINAL sky file
//(the prepared files have the sun clamped out; see tools/hdri_prepare.py). The
//direction is measured here instead of derived from the image, so no assumption
//about how Unity wraps a lat-long image onto a cube can be wrong.
//
//Poly Haven skies are each exposed differently, so every preset has its own
//exposure. Midday stays at 1 (it matches the brightness the farm always had).
//Late afternoon: the 18 degree sun's normal irradiance is set to 1.5x midday's
//(it is lower and redder, but the camera adapts), which puts flat ground at
//about 45% of midday, long shadows and a warm key light. Overcast: about 55% of
//midday on flat ground, no direct sun at all.
//
//Tools > Install HDRI Sky, or batch mode with graphics: -executeMethod SkySetup.Install
public static class SkySetup
{
	const string ScenePath = "Assets/Scenes/SampleScene.unity";
	//the compass heading the farm's sun has always had; shadows keep falling
	//the same way in every preset
	const float SunYaw = 205f;

	class Sky
	{
		public string name, file, material;
		public float exposure;
		//E_sun / pi at sky exposure 1, and its normalised linear colour
		public float sunLinear;
		public Color sunColour;
		//mean of the 0-3 degree band above the horizon, linear
		public Color horizon;
		public bool hasSun;
		//a partly cloudy sky casts moving cloud shadows (tools/build_cloud_shadows.py)
		public bool clouds;
	}

	const string CloudTexture = "Assets/Textures/Sky/CloudShadows.png";
	const string CloudShader = "Assets/Shaders/FarmScreenSpaceShadows.shader";

	static readonly Sky[] Skies =
	{
		new Sky
		{
			name = "Midday", file = "Assets/Textures/Sky/kloofendal_48d_partly_cloudy_farm_4k.hdr",
			material = "Assets/Terrain/FarmSkyHDRI.mat", exposure = 1f,
			sunLinear = 1.447f, sunColour = new Color(0.992f, 1f, 0.912f),
			horizon = new Color(0.425f, 0.458f, 0.554f), hasSun = true, clouds = true,
		},
		new Sky
		{
			name = "Late afternoon", file = "Assets/Textures/Sky/syferfontein_18d_clear_farm_4k.hdr",
			material = "Assets/Terrain/FarmSkyAfternoon.mat", exposure = 0.45f,
			sunLinear = 4.920f, sunColour = new Color(1f, 0.866f, 0.537f),
			horizon = new Color(0.568f, 0.597f, 0.543f), hasSun = true,
		},
		new Sky
		{
			name = "Overcast", file = "Assets/Textures/Sky/overcast_soil_farm_4k.hdr",
			material = "Assets/Terrain/FarmSkyOvercast.mat", exposure = 0.55f,
			sunLinear = 0f, sunColour = Color.white,
			horizon = new Color(0.826f, 0.748f, 0.673f), hasSun = false,
		},
	};

	[MenuItem("Tools/Install HDRI Sky")]
	public static void Install()
	{
		if (EditorSceneManager.GetActiveScene().path != ScenePath) EditorSceneManager.OpenScene(ScenePath);
		Light sun = FindSun();
		if (sun == null) { Debug.LogError("SkySetup: no directional light in the scene"); return; }

		List<SkyPresets.Preset> presets = new List<SkyPresets.Preset>();
		Texture2D clouds = ImportCloudShadows(CloudTexture);
		foreach (Sky sky in Skies)
		{
			Cubemap cube = ImportSky(sky.file);
			if (cube == null) { Debug.LogError("SkySetup: no cubemap at " + sky.file); continue; }

			Material mat = AssetDatabase.LoadAssetAtPath<Material>(sky.material);
			if (mat == null)
			{
				mat = new Material(Shader.Find("Skybox/Cubemap"));
				AssetDatabase.CreateAsset(mat, sky.material);
			}
			mat.shader = Shader.Find("Skybox/Cubemap");
			mat.SetTexture("_Tex", cube);
			mat.SetFloat("_Exposure", sky.exposure);
			mat.SetFloat("_Rotation", 0f);
			RenderSettings.skybox = mat;

			SkyPresets.Preset p = new SkyPresets.Preset { name = sky.name, skybox = mat };
			float elevation = 55f, rotation = 0f, error = 0f;
			if (sky.hasSun)
			{
				Vector3 d0 = FindSunDirection();
				elevation = Mathf.Asin(Mathf.Clamp(d0.y, -1f, 1f)) * Mathf.Rad2Deg;
				Vector3 target = -(Quaternion.Euler(elevation, SunYaw, 0f) * Vector3.forward);

				//Skybox/Cubemap samples the cube in the direction rotated by _Rotation
				//about Y, which adds _Rotation to a direction's azimuth atan2(z, x)
				float phi0 = Mathf.Atan2(d0.z, d0.x) * Mathf.Rad2Deg;
				float phiT = Mathf.Atan2(target.z, target.x) * Mathf.Rad2Deg;
				rotation = Mathf.Repeat(phi0 - phiT, 360f);
				mat.SetFloat("_Rotation", rotation);
				error = Vector3.Angle(FindSunDirection(), target);
				if (error > 1f)
				{
					//the other sign convention; the measurement decides
					float alt = Mathf.Repeat(phiT - phi0, 360f);
					mat.SetFloat("_Rotation", alt);
					float altError = Vector3.Angle(FindSunDirection(), target);
					if (altError < error) { rotation = alt; error = altError; }
					else mat.SetFloat("_Rotation", rotation);
				}

				//Lights use gamma intensity in this project (m_LightsUseLinearIntensity 0):
				//Unity converts colour * intensity to linear per channel. Invert that.
				Color lin = sky.sunColour * (sky.sunLinear * sky.exposure);
				Color gamma = new Color(Mathf.LinearToGammaSpace(lin.r), Mathf.LinearToGammaSpace(lin.g), Mathf.LinearToGammaSpace(lin.b));
				float intensity = Mathf.Max(gamma.r, Mathf.Max(gamma.g, gamma.b));
				p.sunColor = new Color(gamma.r / intensity, gamma.g / intensity, gamma.b / intensity);
				p.sunIntensity = intensity;
				//the sky fills the shadows; letting the sun leak into them as well
				//would double-count it
				p.shadowStrength = 1f;
			}
			else
			{
				p.sunColor = Color.white;
				p.sunIntensity = 0f;
				p.shadowStrength = 0f;
			}
			p.sunEuler = new Vector3(elevation, SunYaw, 0f);
			p.cloudShadows = sky.clouds ? clouds : null;
			p.cloudShadowSize = 600f;
			Color h = sky.horizon * sky.exposure;
			p.fogColor = new Color(Mathf.LinearToGammaSpace(h.r), Mathf.LinearToGammaSpace(h.g), Mathf.LinearToGammaSpace(h.b));
			EditorUtility.SetDirty(mat);
			presets.Add(p);

			Debug.Log(string.Format(CultureInfo.InvariantCulture,
				"SkySetup: {0}: sun elevation {1:0.00} deg, sky rotation {2:0.00} deg, sun-to-light error {3:0.00} deg, exposure {4:0.00}, light {5:0.000} x ({6:0.000}, {7:0.000}, {8:0.000}) gamma, fog ({9:0.000}, {10:0.000}, {11:0.000})",
				sky.name, elevation, rotation, error, sky.exposure, p.sunIntensity, p.sunColor.r, p.sunColor.g, p.sunColor.b,
				p.fogColor.r, p.fogColor.g, p.fogColor.b));
		}

		SkyPresets holder = sun.GetComponent<SkyPresets>();
		if (holder == null) holder = sun.gameObject.AddComponent<SkyPresets>();
		holder.presets = presets.ToArray();
		holder.sun = sun;
		//referenced from the scene so builds include it
		holder.cloudShadowShader = AssetDatabase.LoadAssetAtPath<Shader>(CloudShader);
		if (holder.cloudShadowShader == null) Debug.LogWarning("SkySetup: no cloud shadow shader at " + CloudShader);
		EditorUtility.SetDirty(holder);

		//the scene's own settings are the first preset, so the editor view and a
		//run without the component both look the same
		SkyPresets.Preset first = presets[0];
		RenderSettings.skybox = first.skybox;
		sun.transform.rotation = Quaternion.Euler(first.sunEuler);
		sun.color = first.sunColor;
		sun.intensity = first.sunIntensity;
		sun.shadows = LightShadows.Soft;
		sun.shadowStrength = first.shadowStrength;
		//clouds go through the shadow pass (SkyPresets), never a cookie: a cookie
		//on the sun draws every lit object twice in the forward path
		sun.cookie = null;
		RenderSettings.sun = sun;
		RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Skybox;
		RenderSettings.ambientIntensity = 1f;
		RenderSettings.defaultReflectionMode = UnityEngine.Rendering.DefaultReflectionMode.Skybox;
		RenderSettings.reflectionIntensity = 1f;
		RenderSettings.fogColor = first.fogColor;
		DynamicGI.UpdateEnvironment();

		EditorUtility.SetDirty(sun);
		EditorUtility.SetDirty(sun.transform);
		EditorSceneManager.MarkSceneDirty(sun.gameObject.scene);
		EditorSceneManager.SaveScene(sun.gameObject.scene);
		AssetDatabase.SaveAssets();
		Debug.Log("SkySetup: " + presets.Count + " presets installed on " + sun.name);
	}

	static Cubemap ImportSky(string path)
	{
		TextureImporter imp = AssetImporter.GetAtPath(path) as TextureImporter;
		if (imp == null) return null;

		TextureImporterSettings s = new TextureImporterSettings();
		imp.ReadTextureSettings(s);
		bool changed = s.textureShape != TextureImporterShape.TextureCube
			|| s.generateCubemap != TextureImporterGenerateCubemap.Cylindrical
			|| !s.seamlessCubemap;
		s.textureShape = TextureImporterShape.TextureCube;
		s.generateCubemap = TextureImporterGenerateCubemap.Cylindrical;
		s.cubemapConvolution = TextureImporterCubemapConvolution.None;
		//mip edges of a converted lat-long otherwise show as faint lines in reflections
		s.seamlessCubemap = true;
		s.mipmapEnabled = true;
		imp.SetTextureSettings(s);
		if (imp.maxTextureSize != 1024 || imp.textureCompression != TextureImporterCompression.CompressedHQ) changed = true;
		//a 4096 lat-long holds 1024 pixels per 90 degrees, so a 1024 face keeps
		//what the image has; 2048 faces interpolated it and cost 32 MB of build
		imp.maxTextureSize = 1024;
		//BC6H on desktop: HDR, 1 byte per pixel
		imp.textureCompression = TextureImporterCompression.CompressedHQ;
		if (changed) imp.SaveAndReimport();
		return AssetDatabase.LoadAssetAtPath<Cubemap>(path);
	}

	//The cloud pattern FarmScreenSpaceShadows.shader reads: one linear channel
	//(R8, 256 KB at 512), repeating so the drifting pattern tiles, and no mips:
	//the shader samples it per pixel of a depth buffer, where a depth edge would
	//otherwise pull in a blurred mip and outline every silhouette
	static Texture2D ImportCloudShadows(string path)
	{
		TextureImporter imp = AssetImporter.GetAtPath(path) as TextureImporter;
		if (imp == null) { Debug.LogWarning("SkySetup: no cloud shadows at " + path); return null; }
		TextureImporterSettings s = new TextureImporterSettings();
		imp.ReadTextureSettings(s);
		if (imp.textureType != TextureImporterType.SingleChannel || s.singleChannelComponent != TextureImporterSingleChannelComponent.Red
			|| imp.wrapMode != TextureWrapMode.Repeat || imp.mipmapEnabled || imp.sRGBTexture)
		{
			s.textureType = TextureImporterType.SingleChannel;
			s.singleChannelComponent = TextureImporterSingleChannelComponent.Red;
			s.textureShape = TextureImporterShape.Texture2D;
			s.alphaSource = TextureImporterAlphaSource.None;
			s.sRGBTexture = false;
			s.wrapMode = TextureWrapMode.Repeat;
			s.filterMode = FilterMode.Bilinear;
			s.mipmapEnabled = false;
			imp.SetTextureSettings(s);
			imp.SaveAndReimport();
		}
		return AssetDatabase.LoadAssetAtPath<Texture2D>(path);
	}

	static Light FindSun()
	{
		if (RenderSettings.sun != null) return RenderSettings.sun;
		foreach (Light l in Object.FindObjectsOfType<Light>())
			if (l.type == LightType.Directional) return l;
		return null;
	}

	//Renders only the sky, in six 90 degree views and then a narrow one around
	//the brightest pixel, and returns the direction of that pixel. The sun is
	//far brighter than anything else in the sky, so the brightest pixel is it.
	static Vector3 FindSunDirection()
	{
		GameObject go = new GameObject("SkySunProbe");
		go.hideFlags = HideFlags.HideAndDontSave;
		Camera cam = go.AddComponent<Camera>();
		cam.enabled = false;
		cam.clearFlags = CameraClearFlags.Skybox;
		cam.cullingMask = 0;
		cam.allowHDR = true;
		cam.allowMSAA = false;
		cam.nearClipPlane = 0.1f;
		cam.farClipPlane = 10f;
		cam.aspect = 1f;

		const int N = 256;
		RenderTexture rt = new RenderTexture(N, N, 24, RenderTextureFormat.ARGBHalf);
		Texture2D read = new Texture2D(N, N, TextureFormat.RGBAHalf, false, true);
		cam.targetTexture = rt;

		Vector3[] axes = { Vector3.forward, Vector3.back, Vector3.left, Vector3.right, Vector3.up, Vector3.down };
		float best = -1f;
		Vector3 bestDir = Vector3.up;
		cam.fieldOfView = 90f;
		foreach (Vector3 axis in axes)
		{
			cam.transform.rotation = Quaternion.LookRotation(axis, Mathf.Abs(axis.y) > 0.5f ? Vector3.forward : Vector3.up);
			float lum;
			Vector3 dir = Brightest(cam, rt, read, N, out lum);
			if (lum > best) { best = lum; bestDir = dir; }
		}

		//refine: 6 degree view around the first estimate, 0.02 degrees a pixel
		cam.fieldOfView = 6f;
		cam.transform.rotation = Quaternion.LookRotation(bestDir, Mathf.Abs(bestDir.y) > 0.95f ? Vector3.forward : Vector3.up);
		float fine;
		Vector3 refined = Brightest(cam, rt, read, N, out fine);

		cam.targetTexture = null;
		Object.DestroyImmediate(read);
		rt.Release();
		Object.DestroyImmediate(rt);
		Object.DestroyImmediate(go);
		return refined.normalized;
	}

	static Vector3 Brightest(Camera cam, RenderTexture rt, Texture2D read, int n, out float lum)
	{
		cam.Render();
		RenderTexture prev = RenderTexture.active;
		RenderTexture.active = rt;
		read.ReadPixels(new Rect(0, 0, n, n), 0, 0);
		read.Apply();
		RenderTexture.active = prev;

		Color[] px = read.GetPixels();
		int best = 0;
		lum = -1f;
		for (int i = 0; i < px.Length; i++)
		{
			float l = 0.2126f * px[i].r + 0.7152f * px[i].g + 0.0722f * px[i].b;
			if (l > lum) { lum = l; best = i; }
		}
		//centroid of everything within 90% of the peak, for sub-pixel stability
		float sx = 0f, sy = 0f, sw = 0f;
		for (int i = 0; i < px.Length; i++)
		{
			float l = 0.2126f * px[i].r + 0.7152f * px[i].g + 0.0722f * px[i].b;
			if (l < lum * 0.9f) continue;
			sx += (i % n + 0.5f) * l;
			sy += (i / n + 0.5f) * l;
			sw += l;
		}
		Vector2 uv = sw > 0f ? new Vector2(sx / sw / n, sy / sw / n) : new Vector2((best % n + 0.5f) / n, (best / n + 0.5f) / n);
		return cam.ViewportPointToRay(new Vector3(uv.x, uv.y, 0f)).direction;
	}
}
