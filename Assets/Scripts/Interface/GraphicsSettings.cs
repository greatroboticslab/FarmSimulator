using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;
using UnityEngine.Rendering.PostProcessing;
using TMPro;

//Player-facing graphics control plus the runtime half of the low-end
//performance work.
//
// - picks a quality tier for the machine on first run (integrated GPUs
//   start low, dedicated GPUs start high) and remembers the player's choice
// - scales the render resolution with the tier (the biggest single win on
//   integrated graphics), with a separate HUD button to override it
// - ties post-processing, HDR, shadows, terrain grass, terrain mesh detail,
//   view distance, decorative prop counts and the robot picture-in-picture
//   cameras to the tier
// - caps the frame rate on the lower tiers so laptops do not thermal-throttle
// - F3 shows a frame-time overlay; "-benchmark" runs an unattended measurement
//   pass and writes benchmark.csv (see BenchmarkRoutine)
public class GraphicsSettings : MonoBehaviour
{
	//deliberately not the key the old Graphics button used: the tiers mean
	//different things now, and a level saved by the previous build would stop
	//this one from ever detecting the machine
	const string PrefKey = "FarmSim_QualityTier";
	//0 = automatic for the tier, otherwise the render height in pixels
	const string HeightPrefKey = "FarmSim_RenderHeight";

	//---- per-tier table; index = quality level (Very Low, Low, Medium, High, Very High, Ultra)
	static readonly int[]   RenderHeight   = { 540, 720, 900, 0, 0, 0 };          //0 = native
	static readonly bool[]  PostFx         = { false, false, false, true, true, true };
	static readonly bool[]  BloomOn        = { false, false, false, true, true, true };
	static readonly bool[]  HdrOn          = { false, false, false, true, true, true };
	static readonly float[] ShadowDist     = { 0f, 40f, 70f, 120f, 140f, 170f };
	static readonly float[] DetailDist     = { 0f, 30f, 60f, 100f, 120f, 140f };
	static readonly float[] DetailDensity  = { 0f, 0.35f, 0.6f, 0.85f, 1f, 1f };
	static readonly float[] PixelError     = { 24f, 16f, 12f, 8f, 6f, 5f };
	//beyond this distance the terrain draws from one prebaked texture instead of
	//blending four layers with their normal maps, which is most of its pixel cost
	static readonly float[] BasemapDist    = { 40f, 60f, 150f, 400f, 800f, 1000f };
	static readonly bool[]  TerrainShadows = { false, false, true, true, true, true };
	static readonly float[] FarClip        = { 400f, 550f, 750f, 1000f, 1000f, 1000f };
	static readonly int[]   TargetFps      = { 60, 60, 60, 0, 0, 0 };             //0 = vsync, uncapped
	static readonly float[] PropFrac       = { 0.15f, 0.3f, 0.5f, 1f, 1f, 1f };
	static readonly int[]   AnimatedProps  = { 0, 0, 12, 40, 60, 80 };
	static readonly float[] PipFps         = { 5f, 8f, 12f, 30f, 60f, 0f };         //0 = every frame
	static readonly float[] PipShadowDist  = { 0f, 0f, 20f, 60f, 100f, 170f };
	static readonly int[]   MinLod         = { 1, 0, 0, 0, 0, 0 };             //QualitySettings.maximumLODLevel

	//render heights offered by the Resolution button; 0 = automatic, -1 = native
	static readonly int[] HeightChoices = { 0, 540, 720, 900, 1080, -1 };

	static GraphicsSettings instance;
	static Button hudButton;
	static Button resButton;
	static bool showOverlay;

	//-renderheight from the command line, int.MinValue when absent
	static int heightOverride = int.MinValue;

	//benchmark state
	static bool benchmark;
	static float benchmarkSeconds = 6f;
	static string benchmarkOut;
	//lift the 60 fps cap of the low tiers while measuring, so frame rate shows
	//the frame's real cost (used by the editor comparison, EditorBenchmark.cs)
	static bool benchmarkUncapped;
	//fixed-view screenshots after measuring, see BenchmarkTour
	static bool benchmarkTour;

	float cameraScanAt;
	readonly List<float> frameTimes = new List<float>(1024);
	float overlayFps, overlayCpu, overlayGpu;
	float overlayAccum; int overlayFrames;
	readonly FrameTiming[] timing = new FrameTiming[1];

	static int Levels { get { return QualitySettings.names.Length; } }
	static int Clamp(int level) { return Mathf.Clamp(level, 0, Levels - 1); }
	static T Pick<T>(T[] table, int level) { return table[Mathf.Clamp(level, 0, table.Length - 1)]; }

	//======================================================================
	// startup
	//======================================================================

	//Choose the tier before anything renders
	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
	static void ApplySavedQuality()
	{
		ParseCommandLine();

		//physics may not try to catch up with more than a few steps after a slow
		//frame, otherwise a low frame rate spirals into an even lower one
		Time.maximumDeltaTime = 0.1f;

		int level;
		string source;
		int forced = CommandLineInt("-quality", -1);
		if (forced >= 0) { level = Clamp(forced); source = "command line"; }
		else if (PlayerPrefs.HasKey(PrefKey)) { level = Clamp(PlayerPrefs.GetInt(PrefKey)); source = "saved"; }
		else { level = DetectTier(); source = "auto-detected"; PlayerPrefs.SetInt(PrefKey, level); PlayerPrefs.Save(); }

		//a command-line height applies to this run only; the saved choice stays
		heightOverride = CommandLineInt("-renderheight", int.MinValue);

		QualitySettings.SetQualityLevel(level, true);
		Debug.Log("[GraphicsSettings] " + SystemSummary() + "\n[GraphicsSettings] quality " + QualitySettings.names[level] + " (" + source + ")");

		GameObject go = new GameObject("GraphicsSettings");
		DontDestroyOnLoad(go);
		instance = go.AddComponent<GraphicsSettings>();
		SceneManager.sceneLoaded += OnSceneLoaded;
	}

	//The scene's cameras, terrain and post volume exist now
	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
	static void ApplyAfterLoad()
	{
		ApplyTerrainCost();
		if (benchmark && instance != null) instance.StartCoroutine(instance.BenchmarkRoutine());
	}

	static void OnSceneLoaded(Scene s, LoadSceneMode m) { ApplyTerrainCost(); }

	static void ParseCommandLine()
	{
		string[] args = Environment.GetCommandLineArgs();
		for (int i = 0; i < args.Length; i++)
		{
			string a = args[i].ToLowerInvariant();
			if (a == "-benchmark") benchmark = true;
			else if (a == "-benchmark-seconds" && i + 1 < args.Length) float.TryParse(args[i + 1], NumberStyles.Float, CultureInfo.InvariantCulture, out benchmarkSeconds);
			else if (a == "-benchmark-out" && i + 1 < args.Length) benchmarkOut = args[i + 1];
			else if (a == "-benchmark-uncapped") benchmarkUncapped = true;
			else if (a == "-benchmark-tour") benchmarkTour = true;
			else if (a == "-overlay") showOverlay = true;
		}
	}

	static int CommandLineInt(string flag, int fallback)
	{
		string[] args = Environment.GetCommandLineArgs();
		for (int i = 0; i + 1 < args.Length; i++)
		{
			if (string.Equals(args[i], flag, StringComparison.OrdinalIgnoreCase))
			{
				int v;
				if (int.TryParse(args[i + 1], NumberStyles.Integer, CultureInfo.InvariantCulture, out v)) return v;
			}
		}
		return fallback;
	}

	//Rough hardware classification. Wrong guesses cost one click on the
	//Graphics button; the guess is only used until the player picks a tier.
	public static int DetectTier()
	{
		string name = (SystemInfo.graphicsDeviceName ?? "").ToLowerInvariant();
		string vendor = (SystemInfo.graphicsDeviceVendor ?? "").ToLowerInvariant();
		int vram = SystemInfo.graphicsMemorySize;

		if (name.Contains("basic render") || name.Contains("warp") || name.Contains("llvmpipe") || name.Contains("swiftshader") || name.Contains("software"))
			return 0;

		//test NVIDIA first and match "ati" strictly: "NVIDIA Corporation" contains
		//the letters a-t-i, which used to send every NVIDIA card down the AMD path
		bool nvidia = vendor.Contains("nvidia") || name.Contains("nvidia") || name.Contains("geforce")
			|| name.Contains("quadro") || name.Contains("rtx") || name.Contains("gtx");
		bool intel = !nvidia && (vendor.Contains("intel") || name.Contains("intel"));
		bool amd = !nvidia && !intel && (vendor.Contains("amd") || vendor.Contains("advanced micro")
			|| vendor.StartsWith("ati") || name.Contains("radeon"));
		bool arm = vendor.Contains("qualcomm") || name.Contains("adreno") || name.Contains("mali") || vendor.Contains("apple");

		if (nvidia)
		{
			//MX laptop chips are barely faster than integrated graphics
			if (System.Text.RegularExpressions.Regex.IsMatch(name, @"\bmx\s?\d{3}")) return 2;
			return 3;
		}
		if (intel)
		{
			//discrete Arc cards carry a model number (A380, A770, B580); "Arc Graphics"
			//without one is the Meteor Lake integrated part
			if (System.Text.RegularExpressions.Regex.IsMatch(name, @"arc(\(tm\))?\s*[ab]\d{3}")) return 3;
			if (name.Contains("arc")) return 2;
			//"Iris Xe" on Windows, plain "Xe Graphics" under Mesa on Linux
			if (name.Contains("iris") || name.Contains("xe graphics")) return 1;
			return 0;                                    //UHD / HD Graphics integrated
		}
		if (amd)
		{
			//RDNA integrated parts are named by model (680M, 780M, 890M) and are far
			//faster than the Vega-era ones, which usually report no model number at all:
			//"AMD Radeon(TM) Graphics", "AMD Radeon(TM) Vega 8 Graphics", or under Mesa
			//"AMD Radeon Graphics (renoir, LLVM ...)"
			if (System.Text.RegularExpressions.Regex.IsMatch(name, @"radeon\s*\(?t?m?\)?\s*\d{3}m\b")) return 2;
			bool integratedName = name.Contains("vega") || name.Contains("radeon graphics")
				|| name.Contains("(tm) graphics") || name.EndsWith("graphics")
				|| System.Text.RegularExpressions.Regex.IsMatch(name, @"raven|picasso|renoir|lucienne|cezanne|barcelo|rembrandt|phoenix|mendocino|van gogh|stoney|carrizo|bristol");
			if (integratedName) return 1;
			return 3;
		}
		if (arm) return 1;

		if (vram > 0 && vram < 1500) return 1;
		return 2;
	}

	public static string SystemSummary()
	{
		return "GPU '" + SystemInfo.graphicsDeviceName + "' vendor '" + SystemInfo.graphicsDeviceVendor + "' " + SystemInfo.graphicsMemorySize + "MB "
			+ SystemInfo.graphicsDeviceType + ", CPU '" + SystemInfo.processorType + "' x" + SystemInfo.processorCount + ", RAM " + SystemInfo.systemMemorySize + "MB, display "
			+ Display.main.systemWidth + "x" + Display.main.systemHeight;
	}

	//======================================================================
	// HUD buttons
	//======================================================================

	//Clones an existing HUD button so Graphics matches its siblings
	public static void AttachHudButton(Button template)
	{
		if (template == null || template.transform.parent == null) return;

		RectTransform src = template.GetComponent<RectTransform>();
		float slot = src.sizeDelta.y + 4f;

		if (template.transform.parent.Find("GraphicsButton") == null)
		{
			hudButton = UnityEngine.Object.Instantiate(template, template.transform.parent);
			hudButton.name = "GraphicsButton";
			hudButton.onClick.RemoveAllListeners();
			hudButton.onClick.AddListener(CycleQuality);
			//sits below the Tutorial button, which is one slot below the template
			hudButton.GetComponent<RectTransform>().anchoredPosition = src.anchoredPosition + new Vector2(0f, -2f * slot);
		}

		if (template.transform.parent.Find("ResolutionButton") == null)
		{
			resButton = UnityEngine.Object.Instantiate(template, template.transform.parent);
			resButton.name = "ResolutionButton";
			resButton.onClick.RemoveAllListeners();
			resButton.onClick.AddListener(CycleRenderHeight);
			resButton.GetComponent<RectTransform>().anchoredPosition = src.anchoredPosition + new Vector2(0f, -3f * slot);
		}

		RefreshLabel();
	}

	public static void CycleQuality()
	{
		SetQuality((QualitySettings.GetQualityLevel() + 1) % Levels);
	}

	public static void SetQuality(int level)
	{
		level = Clamp(level);
		QualitySettings.SetQualityLevel(level, true);
		PlayerPrefs.SetInt(PrefKey, level);
		PlayerPrefs.Save();
		ApplyTerrainCost();
		Debug.Log("[GraphicsSettings] quality set to " + QualitySettings.names[level]);
	}

	public static void CycleRenderHeight()
	{
		heightOverride = int.MinValue;
		int current = PlayerPrefs.GetInt(HeightPrefKey, 0);
		int idx = Array.IndexOf(HeightChoices, current);
		int next = HeightChoices[(idx + 1 + HeightChoices.Length) % HeightChoices.Length];
		PlayerPrefs.SetInt(HeightPrefKey, next);
		PlayerPrefs.Save();
		ApplyResolution(QualitySettings.GetQualityLevel());
		RefreshLabel();
	}

	static void RefreshLabel()
	{
		SetLabel(hudButton, "Graphics: " + QualitySettings.names[QualitySettings.GetQualityLevel()]);

		int w, h;
		TargetResolution(QualitySettings.GetQualityLevel(), out w, out h);
		//short enough to fit the HUD button on one line
		bool automatic = heightOverride == int.MinValue && PlayerPrefs.GetInt(HeightPrefKey, 0) == 0;
		SetLabel(resButton, (automatic ? "Auto: " : "Render: ") + w + "x" + h);
	}

	static void SetLabel(Button b, string text)
	{
		if (b == null) return;
		TMP_Text label = b.GetComponentInChildren<TMP_Text>();
		if (label != null) { label.text = text; return; }
		Text legacy = b.GetComponentInChildren<Text>();
		if (legacy != null) legacy.text = text;
	}

	//======================================================================
	// applying a tier
	//======================================================================

	//Kept under its historical name: callers push the tier onto the farm once
	//its terrain exists. Applies everything the tier controls to the scene.
	public static void ApplyTerrainCost()
	{
		int level = QualitySettings.GetQualityLevel();

		QualitySettings.shadowDistance = Pick(ShadowDist, level);
		//the generated trees' full-detail LOD is several thousand triangles of
		//alpha-tested cards; Very Low starts at the reduced one
		QualitySettings.maximumLODLevel = Pick(MinLod, level);

		//frame pacing: capped and vsync-free on the low tiers so a laptop does
		//not run flat out and throttle; vsync on the high tiers
		int fps = Pick(TargetFps, level);
		QualitySettings.vSyncCount = fps > 0 ? 0 : 1;
		Application.targetFrameRate = fps > 0 ? fps : -1;

		foreach (Terrain t in Terrain.activeTerrains)
		{
			if (t == null) continue;
			//GPU-instanced terrain patches; the shipped prefabs still had the CPU path on
			t.drawInstanced = true;
			float dist = Pick(DetailDist, level);
			t.detailObjectDistance = dist;
			t.detailObjectDensity = Pick(DetailDensity, level);
			t.drawTreesAndFoliage = dist > 0f;
			t.heightmapPixelError = Pick(PixelError, level);
			t.basemapDistance = Pick(BasemapDist, level);
			t.shadowCastingMode = Pick(TerrainShadows, level)
				? UnityEngine.Rendering.ShadowCastingMode.On
				: UnityEngine.Rendering.ShadowCastingMode.Off;
		}

		//view distance, with the fog pulled in so the far plane never shows
		float far = Pick(FarClip, level);
		if (RenderSettings.fog && RenderSettings.fogMode == FogMode.Linear)
		{
			RenderSettings.fogEndDistance = Mathf.Min(950f, far - 30f);
			RenderSettings.fogStartDistance = Mathf.Min(160f, RenderSettings.fogEndDistance * 0.4f);
		}

		foreach (Camera cam in UnityEngine.Object.FindObjectsOfType<Camera>(true))
		{
			if (IsPictureInPicture(cam) || RenderScaler.IsPresenter(cam)) continue;
			cam.allowHDR = Pick(HdrOn, level);
			//MSAA only where the quality asset asks for it (Very High and Ultra)
			cam.allowMSAA = level >= 4;
			if (cam.farClipPlane > 100f) cam.farClipPlane = far;
		}

		ApplyPostProcessing(level);
		ApplySkybox(level);
		ApplyResolution(level);

		RobotCameraThrottle.fps = Pick(PipFps, level);
		RobotCameraThrottle.shadowDistance = Pick(PipShadowDist, level);
		RobotCameraThrottle.stripPostProcessing = level < 3;
		RobotCameraThrottle.allowHdr = Pick(HdrOn, level);
		RobotCameraThrottle.farClip = far;

		if (instance != null) instance.AdoptRobotCameras();
		RefreshLabel();
	}

	static void ApplyPostProcessing(int level)
	{
		bool fx = Pick(PostFx, level);
		foreach (PostProcessLayer layer in UnityEngine.Object.FindObjectsOfType<PostProcessLayer>(true))
		{
			Camera cam = layer.GetComponent<Camera>();
			if (cam != null && IsPictureInPicture(cam)) continue;
			layer.enabled = fx;
			//the NaN guard is a debugging aid that costs a full-screen pass
			layer.stopNaNPropagation = false;
			//cheaper FXAA variant on the middle tier
			layer.fastApproximateAntialiasing.fastMode = level < 3;
		}

		bool bloomOn = Pick(BloomOn, level);
		foreach (PostProcessVolume vol in UnityEngine.Object.FindObjectsOfType<PostProcessVolume>(true))
		{
			if (!vol.isGlobal) continue;
			//the shipped profile asset is empty, so the intended look is built here
			//on the volume's runtime instance; a proper asset from Tools > Setup
			//Post Processing is used as-is when present
			PostProcessProfile profile = vol.profile;
			Bloom bloom;
			if (!profile.TryGetSettings(out bloom))
			{
				bloom = profile.AddSettings<Bloom>();
				bloom.enabled.Override(true);
				bloom.intensity.Override(2.6f);
				bloom.threshold.Override(1.05f);
				bloom.softKnee.Override(0.55f);
			}
			bloom.active = bloomOn;

			ColorGrading grading;
			if (!profile.TryGetSettings(out grading))
			{
				grading = profile.AddSettings<ColorGrading>();
				grading.enabled.Override(true);
				grading.saturation.Override(8f);
				grading.contrast.Override(6f);
				grading.postExposure.Override(0.15f);
			}
			grading.active = fx;
		}
	}

	//Alt-tabbed players do not need 60 frames a second of a farm they cannot see
	void OnApplicationFocus(bool focused)
	{
		if (benchmark) return;
		if (focused) ApplyTerrainCost();
		else
		{
			//targetFrameRate is ignored while vsync is on, so drop both
			QualitySettings.vSyncCount = 0;
			Application.targetFrameRate = 10;
		}
	}

	static Material skyboxInstance;
	static Material environmentFrom;

	//Two kinds of sky. The farm's is an HDRI cubemap (Tools > Install HDRI Sky):
	//one texture read per pixel, the same on every tier, nothing to switch. The
	//procedural sky computes its sun disk per pixel behind everything else, and
	//its cheap variant is close enough on the tiers that need the frames.
	//Either way ambient light and reflections come from the sky, and the scene has
	//no baked lighting data to hold them, so they are derived from the sky here
	//whenever it changes.
	static void ApplySkybox(int level)
	{
		Material sky = RenderSettings.skybox;
		if (sky == null) return;

		if (sky.HasProperty("_SunDisk")) ApplyProceduralSunDisk(sky, level);

		if (environmentFrom != RenderSettings.skybox)
		{
			environmentFrom = RenderSettings.skybox;
			DynamicGI.UpdateEnvironment();
		}
	}

	static void ApplyProceduralSunDisk(Material sky, int level)
	{
		if (skyboxInstance == null || sky != skyboxInstance)
		{
			//work on a copy: editing the shared material would change the asset on disk
			skyboxInstance = new Material(sky);
			RenderSettings.skybox = skyboxInstance;
		}

		bool simple = level < 3;
		skyboxInstance.SetFloat("_SunDisk", simple ? 1f : 2f);
		if (simple)
		{
			skyboxInstance.EnableKeyword("_SUNDISK_SIMPLE");
			skyboxInstance.DisableKeyword("_SUNDISK_HIGH_QUALITY");
		}
		else
		{
			skyboxInstance.EnableKeyword("_SUNDISK_HIGH_QUALITY");
			skyboxInstance.DisableKeyword("_SUNDISK_SIMPLE");
		}
	}

	static void TargetResolution(int level, out int w, out int h)
	{
		//in the editor "native" is the Game view, not the desktop: scaling has to be
		//measured against the surface actually being filled, and Screen reports the
		//Game view there. In a player the two are the same thing.
		int nativeW, nativeH;
		if (Application.isEditor)
		{
			nativeW = Screen.width;
			nativeH = Screen.height;
		}
		else
		{
			nativeW = Display.main.systemWidth;
			nativeH = Display.main.systemHeight;
		}
		if (nativeW <= 0 || nativeH <= 0) { nativeW = Screen.width; nativeH = Screen.height; }

		int targetH = heightOverride != int.MinValue ? heightOverride : PlayerPrefs.GetInt(HeightPrefKey, 0);
		if (targetH == 0) targetH = Pick(RenderHeight, level);

		if (targetH <= 0 || targetH >= nativeH) { w = nativeW; h = nativeH; }
		else
		{
			h = targetH;
			w = Mathf.RoundToInt(nativeW * (targetH / (float)nativeH));
		}
		w &= ~1; h &= ~1;
	}

	//Render at a fraction of the screen and let the display scale it up.
	//Fill rate is what integrated GPUs run out of first, so this is the
	//single most effective control in the file.
	static void ApplyResolution(int level)
	{
		int w, h;
		TargetResolution(level, out w, out h);

		//Screen.SetResolution is a no-op in the editor, and the machine this work
		//targets runs the project in the editor, so scale through a render texture
		//there instead. See RenderScaler.
		if (Application.isEditor)
		{
			editorScaleWanted = w < Screen.width || h < Screen.height;
			if (!editorScaleWanted) RenderScaler.Disable();
			else RenderScaler.Apply(Camera.main, w, h);
			return;
		}

		if (Screen.width == w && Screen.height == h) return;
		Screen.SetResolution(w, h, Screen.fullScreenMode);
		Debug.Log("[GraphicsSettings] render resolution " + w + "x" + h + " (display " + Display.main.systemWidth + "x" + Display.main.systemHeight + ")");
	}

	//RenderScaler calls this when the editor Game view is resized, so the scene
	//texture keeps matching the surface it is stretched over.
	public static void RefreshRenderScale()
	{
		if (Application.isEditor) ApplyResolution(QualitySettings.GetQualityLevel());
	}

	//the tier wants the editor view scaled; retried from Update in case there was
	//no main camera yet when the tier was applied
	static bool editorScaleWanted;

	//what the scene is drawn at, and the surface it is shown on: the scaler's
	//texture and the Game view in the editor, the window and desktop in a player
	static string RenderSizeText()
	{
		return RenderScaler.RenderWidth + "x" + RenderScaler.RenderHeight;
	}

	static string DisplaySizeText()
	{
		return Application.isEditor
			? Screen.width + "x" + Screen.height
			: Display.main.systemWidth + "x" + Display.main.systemHeight;
	}

	//Decorative props (trees, rocks, people) scale with the tier; see ObjectPlacer
	public static float PropFraction() { return Pick(PropFrac, QualitySettings.GetQualityLevel()); }
	public static int AnimatedPropBudget() { return Pick(AnimatedProps, QualitySettings.GetQualityLevel()); }

	//Robot picture-in-picture cameras draw into a texture. So does the main camera
	//while RenderScaler scales it, and that one is the player's view: adopting it
	//would disable it (Camera.main goes null, clicks stop working, the listener
	//goes quiet) and redraw it at the picture-in-picture rate of 5 to 12 fps.
	static bool IsPictureInPicture(Camera cam)
	{
		return cam != null && cam.targetTexture != null && !RenderScaler.Owns(cam);
	}

	//Any enabled camera drawing into a render texture is a robot camera; it
	//must not render every frame on its own
	void AdoptRobotCameras()
	{
		foreach (Camera cam in Camera.allCameras)
		{
			if (!IsPictureInPicture(cam)) continue;
			if (cam.GetComponent<RobotCameraThrottle>() == null) cam.gameObject.AddComponent<RobotCameraThrottle>();
		}
	}

	//======================================================================
	// per-frame: robot camera adoption, plant ticking, overlay stats
	//======================================================================

	void Update()
	{
		if (Time.unscaledTime >= cameraScanAt)
		{
			cameraScanAt = Time.unscaledTime + 0.5f;
			AdoptRobotCameras();
			if (editorScaleWanted && !RenderScaler.Active && Camera.main != null) RefreshRenderScale();
		}

		//plants only need per-frame work while a UV lamp is over them
		Plant.TickAll(Time.deltaTime);

		if (Input.GetKeyDown(KeyCode.F3)) showOverlay = !showOverlay;

		if (showOverlay || benchmark)
		{
			FrameTimingManager.CaptureFrameTimings();
			uint n = FrameTimingManager.GetLatestTimings(1, timing);
			overlayAccum += Time.unscaledDeltaTime; overlayFrames++;
			if (n > 0)
			{
				overlayCpu = Mathf.Lerp(overlayCpu, (float)timing[0].cpuFrameTime, 0.1f);
				overlayGpu = Mathf.Lerp(overlayGpu, (float)timing[0].gpuFrameTime, 0.1f);
			}
			if (overlayAccum >= 0.5f)
			{
				overlayFps = overlayFrames / overlayAccum;
				overlayAccum = 0f; overlayFrames = 0;
			}
		}
	}

	void OnGUI()
	{
		if (!showOverlay) return;
		string text = string.Format(CultureInfo.InvariantCulture,
			"{0:0} fps   cpu {1:0.0} ms   gpu {2:0.0} ms\n{3}   render {4} on {5}\n{6}",
			overlayFps, overlayCpu, overlayGpu,
			QualitySettings.names[QualitySettings.GetQualityLevel()], RenderSizeText(), DisplaySizeText(),
			SystemInfo.graphicsDeviceName);
		GUI.color = Color.black;
		GUI.Label(new Rect(11, 11, 600, 70), text);
		GUI.color = Color.white;
		GUI.Label(new Rect(10, 10, 600, 70), text);
	}

	//======================================================================
	// unattended benchmark: FarmSimulator.exe -benchmark [-quality N] [-renderheight H]
	//   [-benchmark-seconds S] [-benchmark-out file]
	// Drives the menu to the farm with a rover, plants a plot, measures, quits.
	// Results append to <persistentDataPath>/benchmark.csv unless -benchmark-out is given.
	//======================================================================
	IEnumerator BenchmarkRoutine()
	{
		Debug.Log("[Benchmark] starting at quality " + QualitySettings.names[QualitySettings.GetQualityLevel()]);
		showOverlay = true;
		float deadline = Time.unscaledTime + 90f;

		//showroom: pick the first wheeled rover
		while (PathMaker.Instance == null && Time.unscaledTime < deadline) yield return null;
		yield return new WaitForSecondsRealtime(2f);

		//the showroom models may still be hidden; only their prefab reference and flags matter
		RobotInfo choice = null;
		List<RobotInfo> infos = new List<RobotInfo>(UnityEngine.Object.FindObjectsOfType<RobotInfo>(true));
		infos.Sort((a, b) => string.CompareOrdinal(a.name, b.name));
		foreach (RobotInfo ri in infos)
		{
			if (ri.humanoid || ri.training || ri.robot == null) continue;
			bool wheeled = ri.robot.GetComponent<DebugRover>() != null;
			if (choice == null || (wheeled && choice.robot.GetComponent<DebugRover>() == null)) choice = ri;
		}
		if (choice == null || PathMaker.Instance == null)
		{
			Debug.LogError("[Benchmark] no rover selectable in the showroom");
			WriteBenchmarkRow("FAILED: no rover", 0, 0, 0, 0, 0);
			Quit();
			yield break;
		}

		PathMaker.Instance.selectedRobot = choice;
		PathMaker.Instance.humanoid = choice.humanoid;
		PathMaker.Instance.useTractor = choice.includeTractor;
		PathMaker.Instance.training = choice.training;
		PathMaker.Instance.useTerrain = choice.terrain;
		if (PathMaker.Instance.mainCam != null)
		{
			PathMaker.Instance.mainCam.mode = 0;
			PathMaker.Instance.mainCam.transform.position = Vector3.zero;
		}
		if (PathMaker.Instance.coordMenu != null) PathMaker.Instance.coordMenu.SetActive(true);
		yield return null;
		yield return null;

		CoordSelector coords = UnityEngine.Object.FindObjectOfType<CoordSelector>(true);
		if (coords == null)
		{
			Debug.LogError("[Benchmark] no CoordSelector in scene");
			WriteBenchmarkRow("FAILED: no coord menu", 0, 0, 0, 0, 0);
			Quit();
			yield break;
		}
		TutorialBook.SuppressFirstTime();
		coords.LoadCoords();

		//farm: wait for the rover to spawn and the props to finish placing
		while ((PathMaker.Instance.rover == null) && Time.unscaledTime < deadline) yield return null;
		ObjectPlacer placer = UnityEngine.Object.FindObjectOfType<ObjectPlacer>();
		while (placer != null && !placer.finished && Time.unscaledTime < deadline) yield return null;
		yield return new WaitForSecondsRealtime(3f);

		//plot: a 20x20 m field just ahead of the rover
		if (PathMaker.Instance.rover != null && PathMaker.Instance.manip != null && PathMaker.Instance.manip.plotHelperPrefab != null)
		{
			GameObject crop = PathMaker.Instance.selectedCrop;
			if (crop == null && PathMaker.Instance.cropList != null && PathMaker.Instance.cropList.Count > 0) crop = PathMaker.Instance.cropList[0];
			if (crop != null)
			{
				PathMaker.Instance.selectedCrop = crop;
				//densities are normally copied out of the plot panel's sliders, and
				//that panel never opens in an unattended run; without this the zone
				//plants nothing and the benchmark measures an empty field
				if (PathMaker.Instance.xDensity <= 0f) PathMaker.Instance.xDensity = 1.4f;
				if (PathMaker.Instance.yDensity <= 0f) PathMaker.Instance.yDensity = 0.55f;
				if (PathMaker.Instance.weedDensity <= 0f) PathMaker.Instance.weedDensity = 0.12f;
				Transform rover = PathMaker.Instance.rover.transform;
				GameObject helperGo = UnityEngine.Object.Instantiate(PathMaker.Instance.manip.plotHelperPrefab);
				PlotHelper helper = helperGo.GetComponent<PlotHelper>();
				if (helper != null)
				{
					helper.plant = crop;
					helper.weedDensity = PathMaker.Instance.weedDensity;
					helper.size = new Vector2(10f, 10f);
					Vector3 flat = rover.forward; flat.y = 0f; flat.Normalize();
					helperGo.transform.position = new Vector3(rover.position.x, 500f, rover.position.z) + flat * 16f;
					helper.PlacePlot();
				}
				UnityEngine.Object.Destroy(helperGo);
			}
		}
		yield return new WaitForSecondsRealtime(4f);

		//a picture of what was measured, next to the CSV
		string shotDir = string.IsNullOrEmpty(benchmarkOut) ? Application.persistentDataPath : Path.GetDirectoryName(Path.GetFullPath(benchmarkOut));
		//the render size is in the name so the same tier at two scales keeps both pictures
		string shot = Path.Combine(shotDir, "benchmark_" + QualitySettings.names[QualitySettings.GetQualityLevel()].Replace(" ", "") + "_" + RenderSizeText() + ".png");
		ScreenCapture.CaptureScreenshot(shot);
		yield return new WaitForSecondsRealtime(0.5f);

		if (benchmarkUncapped)
		{
			QualitySettings.vSyncCount = 0;
			Application.targetFrameRate = -1;
		}
		if (RenderScaler.Active && Camera.main != null) Debug.Log("[Benchmark] " + ClickMappingReport(Camera.main));
		Debug.Log("[Benchmark] " + LightingReport());

		//measure
		frameTimes.Clear();
		double cpuSum = 0, gpuSum = 0; int timed = 0;
		float until = Time.unscaledTime + benchmarkSeconds;
		while (Time.unscaledTime < until)
		{
			yield return null;
			frameTimes.Add(Time.unscaledDeltaTime);
			FrameTimingManager.CaptureFrameTimings();
			if (FrameTimingManager.GetLatestTimings(1, timing) > 0 && timing[0].gpuFrameTime > 0)
			{
				cpuSum += timing[0].cpuFrameTime; gpuSum += timing[0].gpuFrameTime; timed++;
			}
		}

		float total = 0f; foreach (float f in frameTimes) total += f;
		float avgFps = frameTimes.Count / Mathf.Max(total, 0.0001f);
		frameTimes.Sort();
		float p99 = frameTimes[Mathf.Clamp(Mathf.FloorToInt(frameTimes.Count * 0.99f), 0, frameTimes.Count - 1)];
		float lowFps = 1f / Mathf.Max(p99, 0.0001f);
		float cpu = timed > 0 ? (float)(cpuSum / timed) : 0f;
		float gpu = timed > 0 ? (float)(gpuSum / timed) : 0f;

		WriteBenchmarkRow("ok", avgFps, lowFps, cpu, gpu, frameTimes.Count);

		//after measuring, so the fixed views never touch the numbers
		if (benchmarkTour)
			yield return StartCoroutine(BenchmarkTour.Run(shotDir,
				"tour_" + QualitySettings.names[QualitySettings.GetQualityLevel()].Replace(" ", "") + "_" + RenderSizeText()));
		Quit();
	}

	void WriteBenchmarkRow(string status, float avgFps, float lowFps, float cpuMs, float gpuMs, int frames)
	{
		string path = string.IsNullOrEmpty(benchmarkOut) ? Path.Combine(Application.persistentDataPath, "benchmark.csv") : benchmarkOut;
		bool fresh = !File.Exists(path);
		int plants = Plant.All.Count;
		float managedMb = GC.GetTotalMemory(false) / (1024f * 1024f);
		string crop = PathMaker.Instance != null && PathMaker.Instance.selectedCrop != null ? PathMaker.Instance.selectedCrop.name : "";
		StringBuilder sb = new StringBuilder();
		if (fresh) sb.AppendLine("time,quality,render,display,avg_fps,one_percent_low_fps,cpu_ms,gpu_ms,frames,plants,managed_mb,crop,status,gpu_name,vram_mb,cpu_name");
		sb.AppendLine(string.Join(",", new string[] {
			DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture),
			QualitySettings.names[QualitySettings.GetQualityLevel()],
			RenderSizeText(),
			DisplaySizeText(),
			avgFps.ToString("0.0", CultureInfo.InvariantCulture),
			lowFps.ToString("0.0", CultureInfo.InvariantCulture),
			cpuMs.ToString("0.00", CultureInfo.InvariantCulture),
			gpuMs.ToString("0.00", CultureInfo.InvariantCulture),
			frames.ToString(CultureInfo.InvariantCulture),
			plants.ToString(CultureInfo.InvariantCulture),
			managedMb.ToString("0", CultureInfo.InvariantCulture),
			Quote(crop),
			status,
			Quote(SystemInfo.graphicsDeviceName),
			SystemInfo.graphicsMemorySize.ToString(CultureInfo.InvariantCulture),
			Quote(SystemInfo.processorType) }));
		try
		{
			Directory.CreateDirectory(Path.GetDirectoryName(path));
			File.AppendAllText(path, sb.ToString());
		}
		catch (Exception e) { Debug.LogError("[Benchmark] could not write " + path + ": " + e.Message); }
		Debug.Log("[Benchmark] " + status + " quality=" + QualitySettings.names[QualitySettings.GetQualityLevel()] + " plants=" + plants + " managed=" + managedMb.ToString("0") + "MB render=" + RenderSizeText()
			+ " avg=" + avgFps.ToString("0.0", CultureInfo.InvariantCulture) + "fps low1%=" + lowFps.ToString("0.0", CultureInfo.InvariantCulture)
			+ "fps cpu=" + cpuMs.ToString("0.00", CultureInfo.InvariantCulture) + "ms gpu=" + gpuMs.ToString("0.00", CultureInfo.InvariantCulture) + "ms -> " + path);
	}

	static string Quote(string s) { return "\"" + (s ?? "").Replace("\"", "'") + "\""; }

	//The ambient light as the renderer actually has it when measuring. A sky that
	//never made it into the ambient probe shows up here as black, or as whatever
	//the probe held before.
	static string LightingReport()
	{
		Vector3[] dirs = { Vector3.up, Vector3.down };
		Color[] amb = new Color[2];
		RenderSettings.ambientProbe.Evaluate(dirs, amb);
		Light sun = RenderSettings.sun;
		return string.Format(CultureInfo.InvariantCulture,
			"lighting: sky '{0}', ambient {1}, up {2:0.000} {3:0.000} {4:0.000}, down {5:0.000} {6:0.000} {7:0.000}, sun {8}",
			RenderSettings.skybox != null ? RenderSettings.skybox.shader.name : "none", RenderSettings.ambientMode,
			amb[0].r, amb[0].g, amb[0].b, amb[1].r, amb[1].g, amb[1].b,
			sun != null ? sun.intensity.ToString("0.000", CultureInfo.InvariantCulture) + " pitch " + sun.transform.eulerAngles.x.ToString("0.0", CultureInfo.InvariantCulture) : "none");
	}

	//With the scene drawn into a smaller texture, a click still has to land on the
	//world point it would hit unscaled. Compares the converted screen ray against
	//the viewport ray through the same spot.
	static string ClickMappingReport(Camera cam)
	{
		float worst = 0f;
		for (int i = 0; i <= 4; i++)
		{
			for (int j = 0; j <= 4; j++)
			{
				Vector3 p = new Vector3(Screen.width * i / 4f, Screen.height * j / 4f, 0f);
				Ray scaled = RenderScaler.ScreenPointToRay(cam, p);
				Ray expected = cam.ViewportPointToRay(new Vector3(p.x / Screen.width, p.y / Screen.height, 0f));
				worst = Mathf.Max(worst, Vector3.Angle(scaled.direction, expected.direction));
			}
		}
		return "scaled click mapping: worst ray error " + worst.ToString("0.0000", CultureInfo.InvariantCulture)
			+ " deg over 25 points, render " + RenderSizeText() + " on " + DisplaySizeText();
	}

	static void Quit()
	{
#if UNITY_EDITOR
		UnityEditor.EditorApplication.isPlaying = false;
#else
		Application.Quit();
#endif
	}
}
