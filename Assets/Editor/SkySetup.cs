using System.Globalization;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

//Installs the HDRI sky in the main scene:
// - imports the lat-long HDR as a cubemap and builds a Skybox/Cubemap material
// - finds where the sky's sun actually is by rendering the sky, rotates the sky
//   so that sun sits exactly behind the scene's directional light, and sets the
//   light's elevation, colour and intensity from the HDRI
// - takes ambient light and reflections from the sky, and the fog colour from
//   the sky's horizon
//
//The photometric numbers come from tools/hdri_sun.py on the sky file (sun
//irradiance, colour, horizon). The direction is measured here instead of being
//derived from the image, so no assumption about how Unity wraps a lat-long image
//onto a cube can be wrong. Run from Tools > Install HDRI Sky, or in batch mode
//(with graphics, it renders): -executeMethod SkySetup.Install
public static class SkySetup
{
	const string SkyPath = "Assets/Textures/Sky/kloofendal_48d_partly_cloudy_farm_4k.hdr";
	const string MaterialPath = "Assets/Terrain/FarmSkyHDRI.mat";
	const string ScenePath = "Assets/Scenes/SampleScene.unity";

	//tools/hdri_sun.py: irradiance of the sun on a surface facing it is
	//4.510 / 4.547 / 4.148, so the matching light (E / pi) is 1.447 in the sky's
	//units with this colour, and the 0-3 degree band above the horizon averages
	//0.425 / 0.458 / 0.554
	const float SunIntensityLinear = 1.447f;
	static readonly Color SunColourLinear = new Color(0.992f, 1f, 0.912f);
	static readonly Color HorizonLinear = new Color(0.425f, 0.458f, 0.554f);

	//Exposure 1 keeps the HDRI's own balance. It also happens to fit: the median
	//blue sky displays at 0.23 / 0.34 / 0.65 and the 90th percentile cloud at
	//0.71 / 0.79 / 1.04, and the sun comes out at 1.45, close to the 1.36 the
	//procedural setup used, so the farm keeps its overall brightness.
	const float Exposure = 1f;

	[MenuItem("Tools/Install HDRI Sky")]
	public static void Install()
	{
		Cubemap cube = ImportSky();
		if (cube == null) { Debug.LogError("SkySetup: no cubemap at " + SkyPath); return; }

		if (EditorSceneManager.GetActiveScene().path != ScenePath) EditorSceneManager.OpenScene(ScenePath);

		Material sky = AssetDatabase.LoadAssetAtPath<Material>(MaterialPath);
		if (sky == null)
		{
			sky = new Material(Shader.Find("Skybox/Cubemap"));
			AssetDatabase.CreateAsset(sky, MaterialPath);
		}
		sky.shader = Shader.Find("Skybox/Cubemap");
		sky.SetTexture("_Tex", cube);
		sky.SetFloat("_Exposure", Exposure);
		sky.SetFloat("_Rotation", 0f);
		RenderSettings.skybox = sky;

		Light sun = FindSun();
		if (sun == null) { Debug.LogError("SkySetup: no directional light in the scene"); return; }

		//where the sky's sun is with no rotation, measured
		Vector3 d0 = FindSunDirection();
		float elevation = Mathf.Asin(Mathf.Clamp(d0.y, -1f, 1f)) * Mathf.Rad2Deg;

		//keep the light's compass direction (shadows fall where they always have)
		//and give it the sky's elevation
		float yaw = sun.transform.eulerAngles.y;
		sun.transform.rotation = Quaternion.Euler(elevation, yaw, 0f);
		Vector3 target = -sun.transform.forward;

		//Skybox/Cubemap samples the cube in the direction rotated by _Rotation
		//about Y, which adds _Rotation to a direction's azimuth atan2(z, x). The
		//sun shows up where rotating the view direction lands on d0.
		float phi0 = Mathf.Atan2(d0.z, d0.x) * Mathf.Rad2Deg;
		float phiT = Mathf.Atan2(target.z, target.x) * Mathf.Rad2Deg;
		float rotation = Mathf.Repeat(phi0 - phiT, 360f);
		sky.SetFloat("_Rotation", rotation);
		float error = Vector3.Angle(FindSunDirection(), target);
		if (error > 1f)
		{
			//the other sign convention; the measurement decides, not the comment above
			float alt = Mathf.Repeat(phiT - phi0, 360f);
			sky.SetFloat("_Rotation", alt);
			float altError = Vector3.Angle(FindSunDirection(), target);
			if (altError < error) { rotation = alt; error = altError; }
			else sky.SetFloat("_Rotation", rotation);
		}

		//Lights use gamma intensity in this project (m_LightsUseLinearIntensity 0):
		//Unity converts colour * intensity to linear per channel. Invert that so
		//the light's linear value is the HDRI's sun.
		Color lin = SunColourLinear * (SunIntensityLinear * Exposure);
		Color gamma = new Color(Mathf.LinearToGammaSpace(lin.r), Mathf.LinearToGammaSpace(lin.g), Mathf.LinearToGammaSpace(lin.b));
		float intensity = Mathf.Max(gamma.r, Mathf.Max(gamma.g, gamma.b));
		sun.color = new Color(gamma.r / intensity, gamma.g / intensity, gamma.b / intensity);
		sun.intensity = intensity;
		sun.shadows = LightShadows.Soft;
		//the sky now fills shadows the way it does outdoors; letting direct sun
		//leak into them as well would double-count it
		sun.shadowStrength = 1f;
		RenderSettings.sun = sun;

		RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Skybox;
		RenderSettings.ambientIntensity = 1f;
		RenderSettings.defaultReflectionMode = UnityEngine.Rendering.DefaultReflectionMode.Skybox;
		RenderSettings.reflectionIntensity = 1f;
		Color horizon = HorizonLinear * Exposure;
		RenderSettings.fogColor = new Color(Mathf.LinearToGammaSpace(horizon.r), Mathf.LinearToGammaSpace(horizon.g), Mathf.LinearToGammaSpace(horizon.b));
		DynamicGI.UpdateEnvironment();

		EditorUtility.SetDirty(sky);
		EditorUtility.SetDirty(sun);
		EditorUtility.SetDirty(sun.transform);
		EditorSceneManager.MarkSceneDirty(sun.gameObject.scene);
		EditorSceneManager.SaveScene(sun.gameObject.scene);
		AssetDatabase.SaveAssets();

		Debug.Log(string.Format(CultureInfo.InvariantCulture,
			"SkySetup: sky sun measured at elevation {0:0.00} deg; light yaw {1:0.0}, sky rotation {2:0.00} deg, sun-to-light error {3:0.00} deg; light {4:0.000} x ({5:0.000}, {6:0.000}, {7:0.000}) gamma, fog ({8:0.000}, {9:0.000}, {10:0.000})",
			elevation, yaw, rotation, error, sun.intensity, sun.color.r, sun.color.g, sun.color.b,
			RenderSettings.fogColor.r, RenderSettings.fogColor.g, RenderSettings.fogColor.b));
	}

	static Cubemap ImportSky()
	{
		TextureImporter imp = AssetImporter.GetAtPath(SkyPath) as TextureImporter;
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
		if (imp.maxTextureSize != 2048 || imp.textureCompression != TextureImporterCompression.CompressedHQ) changed = true;
		//a 4096 lat-long holds 1024 pixels per 90 degrees; a 2048 face keeps all of it
		imp.maxTextureSize = 2048;
		//BC6H on desktop: HDR, 1 byte per pixel
		imp.textureCompression = TextureImporterCompression.CompressedHQ;
		if (changed) imp.SaveAndReimport();
		return AssetDatabase.LoadAssetAtPath<Cubemap>(SkyPath);
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
	//four orders of magnitude brighter than anything else in the sky, so the
	//brightest pixel is it.
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
