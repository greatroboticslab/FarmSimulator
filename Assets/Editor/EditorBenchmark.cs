using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

//Runs the -benchmark pass (GraphicsSettings.BenchmarkRoutine) inside the editor,
//over a list of tiers, with the editor render scaling on and off.
//
//The player benchmark in build_and_benchmark.sh cannot prove anything about
//editor-only behaviour such as RenderScaler, and the lab runs the project in the
//editor, so this measures it where it is used:
//
//  Unity.exe -projectPath <repo> -executeMethod EditorBenchmark.Run -logFile <log>
//      -benchmark -benchmark-uncapped -benchmark-out <csv> [-benchmark-seconds 8]
//      [-editorbench-tiers 0,1,2,3] [-editorbench-size 1920x1080] [-editorbench-scaling both|on|off]
//
//Not -batchmode: the Game view has to exist and draw. The Game view is set to a
//fixed resolution so every row fills the same number of pixels. The editor quits
//when the list is done, and the player's saved tier and render height are put back.
//
//Entering play mode reloads the scripting domain, so the job list lives in a file
//under Temp and the static constructor picks it up again after every reload.
[InitializeOnLoad]
public static class EditorBenchmark
{
	const string StatePath = "Temp/EditorBenchmark.state";
	const string TierKey = "FarmSim_QualityTier";
	const string HeightKey = "FarmSim_RenderHeight";
	const string Scene = "Assets/Scenes/SampleScene.unity";

	//a run that has not come back to edit mode by then is stuck
	const double PlayTimeout = 240.0;

	static EditorBenchmark()
	{
		//a state file left by a crashed run belongs to another editor process and
		//must not take over someone's normal session
		if (Load() == null) return;
		Hook();
	}

	static void Hook()
	{
		EditorApplication.playModeStateChanged -= OnPlayModeChanged;
		EditorApplication.playModeStateChanged += OnPlayModeChanged;
		EditorApplication.update -= Watchdog;
		EditorApplication.update += Watchdog;
	}

	//---- entry point ------------------------------------------------------

	public static void Run()
	{
		string tiersArg = Arg("-editorbench-tiers", "0,1,2,3");
		string sizeArg = Arg("-editorbench-size", "1920x1080");
		string scaling = Arg("-editorbench-scaling", "both").ToLowerInvariant();

		List<string> jobs = new List<string>();
		foreach (string t in tiersArg.Split(','))
		{
			int tier;
			if (!int.TryParse(t.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out tier)) continue;
			//"off" pins the render height to native, which disables the scaler;
			//"on" leaves it on the tier's automatic height
			if (scaling == "both" || scaling == "off") jobs.Add(tier + ":off");
			if (scaling == "both" || scaling == "on") jobs.Add(tier + ":on");
		}

		Dictionary<string, string> state = new Dictionary<string, string>();
		state["pid"] = CurrentPid();
		state["jobs"] = string.Join(",", jobs.ToArray());
		state["index"] = "0";
		state["size"] = sizeArg;
		state["savedTier"] = PlayerPrefs.HasKey(TierKey) ? PlayerPrefs.GetInt(TierKey).ToString(CultureInfo.InvariantCulture) : "";
		state["savedHeight"] = PlayerPrefs.HasKey(HeightKey) ? PlayerPrefs.GetInt(HeightKey).ToString(CultureInfo.InvariantCulture) : "";
		Save(state);

		Debug.Log("[EditorBenchmark] " + jobs.Count + " runs: " + state["jobs"] + " at " + sizeArg);
		Hook();

		if (EditorSceneManager.GetActiveScene().path != Scene) EditorSceneManager.OpenScene(Scene);

		//-executeMethod runs while the editor is still starting up (package
		//resolution, XR metadata, licensing). Entering play mode from inside that
		//window crashed the editor natively once, so wait for it to go idle first.
		float delay;
		if (!float.TryParse(Arg("-editorbench-delay", "20"), NumberStyles.Float, CultureInfo.InvariantCulture, out delay)) delay = 20f;
		settleUntil = EditorApplication.timeSinceStartup + delay;
		EditorApplication.update += WaitToSettle;
	}

	static double settleUntil;

	static void WaitToSettle()
	{
		if (EditorApplication.timeSinceStartup < settleUntil) return;
		if (EditorApplication.isCompiling || EditorApplication.isUpdating) return;
		EditorApplication.update -= WaitToSettle;
		StartNext();
	}

	//---- job sequencing ---------------------------------------------------

	static void StartNext()
	{
		Dictionary<string, string> state = Load();
		if (state == null) return;

		string[] jobs = state["jobs"].Split(new[] { ',' }, StringSplitOptions.RemoveEmptyEntries);
		int index = int.Parse(state["index"], CultureInfo.InvariantCulture);
		if (index >= jobs.Length) { Finish(state); return; }

		string[] parts = jobs[index].Split(':');
		int tier = int.Parse(parts[0], CultureInfo.InvariantCulture);
		bool scaled = parts[1] == "on";

		PlayerPrefs.SetInt(TierKey, tier);
		PlayerPrefs.SetInt(HeightKey, scaled ? 0 : -1);
		PlayerPrefs.Save();

		string size = state["size"];
		string[] wh = size.Split('x');
		SetGameViewSize(int.Parse(wh[0], CultureInfo.InvariantCulture), int.Parse(wh[1], CultureInfo.InvariantCulture));

		state["started"] = EditorApplication.timeSinceStartup.ToString("R", CultureInfo.InvariantCulture);
		Save(state);

		Debug.Log("[EditorBenchmark] run " + (index + 1) + "/" + jobs.Length + ": tier " + tier + ", scaling " + (scaled ? "on" : "off"));
		EditorApplication.isPlaying = true;
	}

	static void OnPlayModeChanged(PlayModeStateChange change)
	{
		if (change != PlayModeStateChange.EnteredEditMode) return;
		Dictionary<string, string> state = Load();
		if (state == null) return;
		state["index"] = (int.Parse(state["index"], CultureInfo.InvariantCulture) + 1).ToString(CultureInfo.InvariantCulture);
		state.Remove("started");
		Save(state);
		EditorApplication.delayCall += StartNext;
	}

	static void Watchdog()
	{
		if (!EditorApplication.isPlaying) return;
		Dictionary<string, string> state = Load();
		if (state == null || !state.ContainsKey("started")) return;
		double started = double.Parse(state["started"], CultureInfo.InvariantCulture);
		if (EditorApplication.timeSinceStartup - started < PlayTimeout) return;

		Debug.LogError("[EditorBenchmark] run " + state["index"] + " did not finish in " + PlayTimeout + " s; stopping it");
		state.Remove("started");
		Save(state);
		EditorApplication.isPlaying = false;
	}

	static void Finish(Dictionary<string, string> state)
	{
		Restore(TierKey, state["savedTier"]);
		Restore(HeightKey, state["savedHeight"]);
		PlayerPrefs.Save();
		File.Delete(StatePath);
		Debug.Log("[EditorBenchmark] done");
		EditorApplication.Exit(0);
	}

	static void Restore(string key, string saved)
	{
		int v;
		if (int.TryParse(saved, NumberStyles.Integer, CultureInfo.InvariantCulture, out v)) PlayerPrefs.SetInt(key, v);
		else PlayerPrefs.DeleteKey(key);
	}

	//---- Game view size ---------------------------------------------------

	//The Game view's resolution list is internal API. Adds (or reuses) a fixed
	//resolution entry and selects it; logs what it managed so a failure is visible
	//in the results rather than silently measuring a different surface.
	static void SetGameViewSize(int width, int height)
	{
		try
		{
			Assembly asm = typeof(EditorWindow).Assembly;
			Type gameViewType = asm.GetType("UnityEditor.GameView");
			Type sizesType = asm.GetType("UnityEditor.GameViewSizes");
			Type sizeType = asm.GetType("UnityEditor.GameViewSize");
			Type sizeKind = asm.GetType("UnityEditor.GameViewSizeType");

			EditorWindow gameView = EditorWindow.GetWindow(gameViewType);
			object sizes = typeof(ScriptableSingleton<>).MakeGenericType(sizesType).GetProperty("instance").GetValue(null, null);
			object groupType = sizesType.GetProperty("currentGroupType").GetValue(sizes, null);
			object group = sizesType.GetMethod("GetGroup").Invoke(sizes, new[] { groupType });
			Type groupClass = group.GetType();

			int count = (int)groupClass.GetMethod("GetTotalCount").Invoke(group, null);
			int found = -1;
			for (int i = 0; i < count; i++)
			{
				object s = groupClass.GetMethod("GetGameViewSize").Invoke(group, new object[] { i });
				int sw = (int)sizeType.GetProperty("width").GetValue(s, null);
				int sh = (int)sizeType.GetProperty("height").GetValue(s, null);
				int kind = Convert.ToInt32(sizeType.GetProperty("sizeType").GetValue(s, null));
				if (sw == width && sh == height && kind == 1) { found = i; break; }
			}
			if (found < 0)
			{
				ConstructorInfo ctor = sizeType.GetConstructor(new[] { sizeKind, typeof(int), typeof(int), typeof(string) });
				object size = ctor.Invoke(new object[] { Enum.ToObject(sizeKind, 1), width, height, "Benchmark" });
				groupClass.GetMethod("AddCustomSize").Invoke(group, new[] { size });
				found = (int)groupClass.GetMethod("GetTotalCount").Invoke(group, null) - 1;
			}

			const BindingFlags any = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
			PropertyInfo selected = gameViewType.GetProperty("selectedSizeIndex", any);
			if (selected != null && selected.CanWrite) selected.SetValue(gameView, found, null);
			else gameViewType.GetMethod("SizeSelectionCallback", any).Invoke(gameView, new object[] { found, null });

			gameView.Focus();
			gameView.Repaint();
			Debug.Log("[EditorBenchmark] Game view set to " + width + "x" + height + " (size index " + found + ")");
		}
		catch (Exception e)
		{
			Debug.LogError("[EditorBenchmark] could not set the Game view size: " + e);
		}
	}

	//---- plumbing ---------------------------------------------------------

	static string Arg(string flag, string fallback)
	{
		string[] args = Environment.GetCommandLineArgs();
		for (int i = 0; i + 1 < args.Length; i++)
			if (string.Equals(args[i], flag, StringComparison.OrdinalIgnoreCase)) return args[i + 1];
		return fallback;
	}

	static string CurrentPid()
	{
		return System.Diagnostics.Process.GetCurrentProcess().Id.ToString(CultureInfo.InvariantCulture);
	}

	//null unless a run started by this editor process is in progress
	static Dictionary<string, string> Load()
	{
		if (!File.Exists(StatePath)) return null;
		Dictionary<string, string> state = new Dictionary<string, string>();
		foreach (string line in File.ReadAllLines(StatePath))
		{
			int eq = line.IndexOf('=');
			if (eq > 0) state[line.Substring(0, eq)] = line.Substring(eq + 1);
		}
		string pid;
		if (!state.TryGetValue("pid", out pid) || pid != CurrentPid()) return null;
		return state;
	}

	static void Save(Dictionary<string, string> state)
	{
		List<string> lines = new List<string>();
		foreach (KeyValuePair<string, string> kv in state) lines.Add(kv.Key + "=" + kv.Value);
		Directory.CreateDirectory(Path.GetDirectoryName(StatePath));
		File.WriteAllLines(StatePath, lines.ToArray());
	}
}
