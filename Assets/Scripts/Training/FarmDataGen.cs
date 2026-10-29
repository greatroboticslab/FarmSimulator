using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using UnityEngine;

//Training data export and live policy driving, unattended, from the command line.
//
//Record harvest-driving episodes (the autopilot drives, with random detours):
//  FarmSimulator.exe -batchmode -datagen D:\farmdata\run1 -fw-episodes 40 -fw-seed 1
//
//Let a trained policy drive live (farmingworld's serve_policy.py listening):
//  FarmSimulator.exe -policy 127.0.0.1:5555 -fw-episodes 3 [-datagen <dir>] [-fw-video <dir>]
//
//Each episode: clear the field, plant one plot (random crop, size, weeds, place
//near the farm's spawn), random sky, put the robot down somewhere near the plot
//facing anywhere, turn Self Driving on, record until the plot is done, the time
//runs out, or the robot is stuck. See FarmRecorder for the files.
//
//Options (all optional):
//  -fw-episodes N      episodes (20)
//  -fw-seconds S       simulated seconds per episode at most (90)
//  -fw-seed N          random seed (1)
//  -fw-robot NAME      part of the robot's name ("Cutter")
//  -fw-crops a,b       crops to choose from, by part of the name ("pepper,tomato")
//  -fw-perturb P       share of steps driven by random detours (0.15; 0 with -policy)
//  -fw-hz H            recording / decision rate in simulated time (10)
//  -fw-size N          camera frame size in pixels (128)
//  -fw-timescale K     simulated seconds per real second (1; data only, never with -policy)
//  -fw-video DIR       also save a chase-camera view at 15 fps as JPGs (for a demo clip)
public class FarmDataGen : MonoBehaviour
{
	string outDir, policyAddr, videoDir, robotName = "Cutter";
	int episodes = 20, seed = 1, size = 128;
	float seconds = 90f, perturb = -1f, hz = 10f, timeScale = 1f;
	bool noTeleport; //-fw-no-teleport: the first episode starts where the robot spawned (a diagnostic)
	string[] crops = { "pepper", "tomato" };

	System.Random rng;
	DebugRover rover;
	FarmRecorder rec;
	FarmPolicyLink link;
	Vector3 spawn;

	//random detour state
	int detourLeft;
	float detourF, detourT;

	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
	static void Boot()
	{
		string[] a = Environment.GetCommandLineArgs();
		bool want = false;
		foreach (string s in a)
			if (s == "-datagen" || s == "-policy") want = true;
		if (!want) return;
		GameObject go = new GameObject("FarmDataGen");
		DontDestroyOnLoad(go);
		go.AddComponent<FarmDataGen>().Parse(a);
	}

	void Parse(string[] a)
	{
		foreach (string s in a) if (s == "-fw-no-teleport") noTeleport = true;
		for (int i = 0; i + 1 < a.Length; i++)
		{
			string k = a[i], v = a[i + 1];
			if (k == "-datagen") outDir = v;
			else if (k == "-policy") policyAddr = v;
			else if (k == "-fw-video") videoDir = v;
			else if (k == "-fw-robot") robotName = v;
			else if (k == "-fw-crops") crops = v.Split(',');
			else if (k == "-fw-episodes") int.TryParse(v, out episodes);
			else if (k == "-fw-seed") int.TryParse(v, out seed);
			else if (k == "-fw-size") int.TryParse(v, out size);
			else if (k == "-fw-seconds") seconds = F(v, seconds);
			else if (k == "-fw-perturb") perturb = F(v, perturb);
			else if (k == "-fw-hz") hz = F(v, hz);
			else if (k == "-fw-timescale") timeScale = F(v, timeScale);
		}
		if (perturb < 0f) perturb = policyAddr != null ? 0f : 0.15f;
		if (policyAddr != null) timeScale = 1f; //live means real time
	}

	static float F(string s, float fallback)
	{
		float f;
		return float.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out f) ? f : fallback;
	}

	IEnumerator Start()
	{
		rng = new System.Random(seed);
		UnityEngine.Random.InitState(seed);
		Debug.Log("[FarmDataGen] " + (policyAddr != null ? "policy " + policyAddr : "autopilot") + ", out " + (outDir ?? "none")
			+ ", " + episodes + " episodes of up to " + seconds + " s, robot " + robotName + ", crops " + string.Join("/", crops)
			+ ", perturb " + perturb + ", " + hz + " Hz, " + size + " px, timescale " + timeScale);

		yield return GraphicsSettings.EnterFarm(robotName, 240f);
		if (!GraphicsSettings.EnteredFarm)
		{
			Debug.LogError("[FarmDataGen] could not reach the farm: " + GraphicsSettings.EnterFarmError);
			Application.Quit(2);
			yield break;
		}
		PathMaker pm = PathMaker.Instance;
		rover = pm.rover;
		spawn = rover.transform.position;
		rec = FarmRecorder.Attach(rover, size, hz);
		rec.decide = Decide;

		if (policyAddr != null)
		{
			link = new FarmPolicyLink(policyAddr);
			if (!link.Connect(60f)) { Application.Quit(3); yield break; }
		}
		if (outDir != null) Directory.CreateDirectory(outDir);
		ChaseVideo video = videoDir != null ? ChaseVideo.Attach(rover.transform, videoDir) : null;

		int done = 0;
		for (int ep = 0; ep < episodes; ep++)
		{
			Time.timeScale = 1f;
			yield return ClearField(pm);

			//the plot
			string cropWord = crops[rng.Next(crops.Length)].Trim();
			GameObject crop = GraphicsSettings.FindCrop(cropWord);
			//PlotHelper.size is the half-width (at least 4 m): plots 8-11 m a side, 50-90 plants
			Vector2 plotSize = new Vector2(R(4f, 5.5f), R(4f, 5.5f));
			float weeds = R(0.03f, 0.2f);
			Vector3 centre;
			if (!FindFlatSpot(plotSize * 2f, out centre))
			{
				Debug.LogWarning("[FarmDataGen] episode " + ep + ": no flat open ground found; skipping");
				continue;
			}
			int sky = 0;
			if (SkyPresets.Instance != null && SkyPresets.Instance.presets.Length > 0)
			{
				sky = rng.Next(SkyPresets.Instance.presets.Length);
				SkyPresets.Instance.Apply(sky);
			}
			GraphicsSettings.PlacePlot(crop, centre, plotSize, weeds);
			yield return new WaitForSecondsRealtime(0.5f); //plants find their fruit in Start

			//the robot: somewhere around the plot, facing anywhere
			Vector3 startPos; float yaw;
			StartPose(centre, plotSize, out startPos, out yaw);
			if (rover.robotArm != null) rover.robotArm.ResetArm();
			if (rover.basket != null) rover.basket.weight = 0f;
			rover.netControlled = false;
			detourLeft = 0;
			LogPhysics("before teleport");
			if (!(noTeleport && ep == 0)) rover.TeleportTo(startPos, yaw);
			else { startPos = rover.transform.position; yaw = rover.transform.eulerAngles.y; }
			for (int k = 0; k < 20; k++) yield return new WaitForFixedUpdate(); //settle on the wheels
			LogPhysics("after settling");
			StartCoroutine(LogPhysicsLater(3f));
			if (ep == 0) FigureCensus();

			List<Plant> plot = new List<Plant>();
			int fruitAtStart = 0;
			foreach (Plant p in Plant.All)
				if (p != null && p.desired) { plot.Add(p); fruitAtStart += FarmRecorder.LiveFruit(p); }
			int waypointsAtStart = pm.waypoints != null ? pm.waypoints.Count : 0;

			string task = TaskSentence(cropWord);
			string dir = outDir != null ? Path.Combine(outDir, "ep_" + (policyAddr != null ? "policy_" : "") + seed.ToString("000") + "_" + ep.ToString("0000")) : null;
			string startJson = "{\"episode\":" + ep + ",\"seed\":" + seed + ",\"task\":\"" + task + "\",\"crop\":\"" + Esc(crop != null ? crop.name : cropWord)
				+ "\",\"robot\":\"" + Esc(rover.name.Replace("(Clone)", "")) + "\",\"hz\":" + Num(hz) + ",\"size\":" + size
				+ ",\"plot_centre\":[" + Num(centre.x) + "," + Num(centre.y) + "," + Num(centre.z) + "],\"plot_size\":[" + Num(plotSize.x) + "," + Num(plotSize.y)
				+ "],\"weed_density\":" + Num(weeds) + ",\"sky\":" + sky + ",\"quality\":\"" + QualitySettings.names[QualitySettings.GetQualityLevel()]
				+ "\",\"start\":[" + Num(startPos.x) + "," + Num(startPos.z) + "," + Num(yaw) + "],\"fruit_at_start\":" + fruitAtStart
				+ ",\"plants\":" + plot.Count + ",\"waypoints\":" + waypointsAtStart + ",\"perturb\":" + Num(perturb)
				+ ",\"policy\":" + (policyAddr != null ? "true" : "false") + ",\"columns\":\"" + FarmRecorder.Header + "\"}";
			if (link != null) link.BeginEpisode(startJson);
			if (video != null) video.Begin("ep" + ep.ToString("000"));

			rec.Begin(dir);
			SetSelfDriving(true);
			rover.selfDriving = true;
			Time.timeScale = timeScale;

			float t0 = Time.time, lastCheck = Time.time;
			Vector3 lastPos = rover.transform.position;
			float moved = 0f, stillFor = 0f;
			string end = "time";
			while (Time.time - t0 < seconds)
			{
				yield return null;
				if (Time.time - lastCheck >= 1f)
				{
					float d = Vector3.Distance(rover.transform.position, lastPos);
					moved += d;
					bool busy = rover.robotArm != null && rover.robotArm.harvesting;
					stillFor = d < 0.05f && !busy ? stillFor + (Time.time - lastCheck) : 0f;
					lastPos = rover.transform.position;
					lastCheck = Time.time;
					int live = 0;
					foreach (Plant p in plot) if (p != null) live += FarmRecorder.LiveFruit(p);
					rec.picked = fruitAtStart - live;
				}
				bool idleArm = rover.robotArm == null || !rover.robotArm.harvesting;
				if (rover.currentWaypoint == null && (pm.waypoints == null || pm.waypoints.Count == 0) && idleArm) { end = "done"; break; }
				if (stillFor > 20f) { end = "stuck"; break; }
				if (link != null && link.Failed) { end = "link lost"; break; }
			}
			Time.timeScale = 1f;
			rover.netControlled = false;
			SetSelfDriving(false);
			rover.selfDriving = false;
			int steps = rec.Step;
			rec.End();
			if (video != null) video.End();

			int liveEnd = 0;
			foreach (Plant p in plot) if (p != null) liveEnd += FarmRecorder.LiveFruit(p);
			int left = pm.waypoints != null ? pm.waypoints.Count : 0;
			string result = "{\"end\":\"" + end + "\",\"steps\":" + steps + ",\"sim_seconds\":" + Num(Time.time - t0)
				+ ",\"fruit_picked\":" + (fruitAtStart - liveEnd) + ",\"basket_kg\":" + Num(rover.basket != null ? rover.basket.weight : 0f)
				+ ",\"waypoints_done\":" + (waypointsAtStart - left) + ",\"moved_m\":" + Num(moved) + "}";
			if (dir != null)
				File.WriteAllText(Path.Combine(dir, "meta.json"), startJson.Substring(0, startJson.Length - 1) + ",\"result\":" + result + "}");
			if (link != null) link.EndEpisode(result);
			Debug.Log("[FarmDataGen] episode " + ep + " " + task + ": " + result);
			done++;
		}
		Debug.Log("[FarmDataGen] finished " + done + " episodes");
		if (link != null) link.Dispose();
		Application.Quit(0);
	}

	//Called by FarmRecorder for every frame: who drives for the coming interval
	void Decide(byte[] frame, float[] row)
	{
		bool armBusy = row[FarmRecorder.colArmBusy] > 0.5f;
		rec.policyMs = 0f;
		if (link != null && !link.Failed)
		{
			float f, t, l;
			float start = Time.realtimeSinceStartup;
			if (link.Decide(rec.Step, row, frame, out f, out t, out l))
			{
				rec.policyMs = (Time.realtimeSinceStartup - start) * 1000f;
				rover.netControlled = true;
				rover.netForwardInput = f;
				rover.netTurnInput = t;
				rover.netLightInput = l;
				rec.source = FarmRecorder.SourcePolicy;
				rec.execForward = armBusy ? 0f : f;
				rec.execTurn = armBusy ? 0f : t;
				rec.execLight = l >= 0.5f;
				return;
			}
			rover.netControlled = false;
		}

		//random detours, so the data also shows how to recover from mistakes
		if (detourLeft == 0 && !armBusy && perturb > 0f && rng.NextDouble() < perturb / (10f * (1f - perturb)))
		{
			detourLeft = rng.Next(5, 16);
			do
			{
				detourF = rng.Next(-1, 2);
				detourT = rng.Next(-1, 2);
			} while (detourF == 0f && detourT == 0f);
		}
		if (detourLeft > 0 && !armBusy)
		{
			detourLeft--;
			rover.netControlled = true;
			rover.netForwardInput = detourF;
			rover.netTurnInput = detourT;
			rover.netLightInput = 0f;
			rec.source = FarmRecorder.SourceRandom;
			rec.execForward = detourF;
			rec.execTurn = detourT;
			rec.execLight = false;
			return;
		}
		detourLeft = 0;
		rover.netControlled = false;
		rec.source = FarmRecorder.SourceExpert;
		rec.execForward = rover.expertForward;
		rec.execTurn = rover.expertTurn;
		rec.execLight = rover.expertLight;
	}

	//The HUD copies its Self Driving checkbox onto the robot ten times a second
	//(RoverControls.Update), so the checkbox is what has to change. The scene has
	//more than one RoverControls (one is for the humanoid and stays inactive);
	//set them all.
	void SetSelfDriving(bool on)
	{
		int n = 0;
		foreach (RoverControls rc in FindObjectsOfType<RoverControls>(true))
			if (rc.selfDriving != null) { rc.selfDriving.isOn = on; n++; }
		rover.selfDriving = on;
		if (on) Debug.Log("[FarmDataGen] Self Driving on (" + n + " checkbox(es))");
	}

	//Every human-shaped thing in the scene (anything with an Animator or a skinned
	//mesh), where it is, how far its lowest point is from the ground under it, and
	//what it is made of: to find stray figures (one showed up half in the ground)
	void FigureCensus()
	{
		HashSet<Transform> roots = new HashSet<Transform>();
		foreach (Animator a in FindObjectsOfType<Animator>()) roots.Add(a.transform);
		foreach (SkinnedMeshRenderer smr in FindObjectsOfType<SkinnedMeshRenderer>())
		{
			Animator a = smr.GetComponentInParent<Animator>();
			roots.Add(a != null ? a.transform : smr.transform.root);
		}
		int n = 0;
		foreach (Transform t in roots)
		{
			if (t.IsChildOf(rover.transform)) continue;
			Renderer[] rs = t.GetComponentsInChildren<Renderer>();
			if (rs.Length == 0) continue;
			Bounds b = rs[0].bounds;
			foreach (Renderer r in rs) b.Encapsulate(r.bounds);
			float ground;
			bool hasGround = DebugRover.FindGroundHeight(b.center, t, out ground);
			string mats = "";
			foreach (Renderer r in rs)
				foreach (Material m in r.sharedMaterials)
					if (m != null && mats.Length < 80) mats += m.name.Replace(" (Instance)", "") + " ";
			string path = t.name;
			for (Transform p = t.parent; p != null; p = p.parent) path = p.name + "/" + path;
			Debug.Log(string.Format(CultureInfo.InvariantCulture,
				"[FarmDataGen] figure {0}: {1} active {2}, at ({3:0.0}, {4:0.0}, {5:0.0}), {6:0.0} m from robot, {7:0.00} m tall, lowest point {8} ground, materials {9}",
				n++, path, t.gameObject.activeInHierarchy, b.center.x, b.min.y, b.center.z,
				Vector3.Distance(b.center, rover.transform.position), b.size.y,
				hasGround ? (b.min.y - ground).ToString("+0.00;-0.00", CultureInfo.InvariantCulture) + " m from" : "no", mats));
		}
		Debug.Log("[FarmDataGen] figures: " + n);
	}

	IEnumerator LogPhysicsLater(float s)
	{
		yield return new WaitForSeconds(s);
		LogPhysics(s + " s into the episode");
	}

	//What the robot's body and wheels are doing (why a robot does not move)
	void LogPhysics(string when)
	{
		Rigidbody rb = rover.rb;
		float ground;
		DebugRover.FindGroundHeight(rover.transform.position, rover.transform, out ground);
		StringBuilder sb = new StringBuilder("[FarmDataGen] physics " + when + ": pos " + rover.transform.position
			+ " above ground " + (rover.transform.position.y - ground).ToString("0.00")
			+ ", kinematic " + (rb != null && rb.isKinematic) + ", asleep " + (rb != null && rb.IsSleeping())
			+ ", vel " + (rb != null ? rb.velocity.ToString("0.00") : "-") + ", friction " + rover.currentFriction.ToString("0.00")
			+ " x" + rover.frictionMultiplier + ", stuck " + rover.stuck + ", blocked " + rover.blocked
			+ ", selfDriving " + rover.selfDriving + ", net " + rover.netControlled
			+ ", cmd " + rover.CurrentForward + "/" + rover.CurrentTurn + ", timeScale " + Time.timeScale + ", wheels:");
		foreach (AxleInfo axle in rover.axles)
			foreach (WheelCollider w in axle.wheels)
				if (w != null) sb.Append(" [" + (w.isGrounded ? "ground" : "air") + " torque " + w.motorTorque.ToString("0.0")
					+ " brake " + w.brakeTorque.ToString("0.0") + " rpm " + w.rpm.ToString("0") + "]");
		Debug.Log(sb.ToString());
	}

	IEnumerator ClearField(PathMaker pm)
	{
		Transform contents = pm.FarmContents;
		for (int i = contents.childCount - 1; i >= 0; i--) Destroy(contents.GetChild(i).gameObject);
		if (pm.waypoints != null) pm.waypoints.Clear();
		if (pm.currentWayPointObj != null) Destroy(pm.currentWayPointObj);
		rover.currentWaypoint = null;
		yield return null;
		yield return null;
	}

	float R(float a, float b) { return a + (float)rng.NextDouble() * (b - a); }

	//Open, fairly flat terrain for a plot (plus a margin to drive around it), 12-40 m from the spawn
	bool FindFlatSpot(Vector2 plot, out Vector3 centre)
	{
		for (int attempt = 0; attempt < 60; attempt++)
		{
			float ang = R(0f, Mathf.PI * 2f), dist = R(12f, 40f);
			Vector3 c = spawn + new Vector3(Mathf.Cos(ang), 0f, Mathf.Sin(ang)) * dist;
			float lo = float.MaxValue, hi = float.MinValue;
			bool ok = true;
			for (int ix = 0; ix <= 6 && ok; ix++)
				for (int iz = 0; iz <= 6 && ok; iz++)
				{
					float x = c.x + (ix / 6f - 0.5f) * (plot.x + 8f);
					float z = c.z + (iz / 6f - 0.5f) * (plot.y + 8f);
					RaycastHit hit;
					if (!Physics.Raycast(new Vector3(x, 1000f, z), Vector3.down, out hit, 2000f, ~0, QueryTriggerInteraction.Ignore)
						|| !(hit.collider is TerrainCollider)) { ok = false; break; }
					lo = Mathf.Min(lo, hit.point.y);
					hi = Mathf.Max(hi, hit.point.y);
				}
			if (ok && hi - lo < 1.2f)
			{
				centre = new Vector3(c.x, (lo + hi) * 0.5f, c.z);
				return true;
			}
		}
		centre = spawn;
		return false;
	}

	//A spot 3-7 m from the first point of the robot's route, outside the plot
	//(half-widths half), on open ground, and a random heading. The autopilot
	//drives at about 0.4 m/s, so starting far off spends the episode crossing grass.
	void StartPose(Vector3 centre, Vector2 half, out Vector3 pos, out float yaw)
	{
		PathMaker pm = PathMaker.Instance;
		Vector2 anchor = rover.currentWaypoint != null ? rover.currentWaypoint.pos
			: pm.waypoints != null && pm.waypoints.Count > 0 ? pm.waypoints[0].pos : new Vector2(centre.x, centre.z);
		pos = spawn;
		for (int attempt = 0; attempt < 60; attempt++)
		{
			float ang = R(0f, Mathf.PI * 2f);
			float rad = R(3f, 7f);
			Vector3 p = new Vector3(anchor.x, 0f, anchor.y) + new Vector3(Mathf.Cos(ang), 0f, Mathf.Sin(ang)) * rad;
			bool inside = Mathf.Abs(p.x - centre.x) < half.x + 0.8f && Mathf.Abs(p.z - centre.z) < half.y + 0.8f;
			if (inside && attempt < 50) continue;
			RaycastHit hit;
			if (Physics.Raycast(new Vector3(p.x, 1000f, p.z), Vector3.down, out hit, 2000f, ~0, QueryTriggerInteraction.Ignore)
				&& hit.collider is TerrainCollider)
			{
				pos = hit.point + Vector3.up * 0.8f;
				break;
			}
		}
		yaw = R(0f, 360f);
	}

	static readonly string[] Verbs = { "harvest the {0}", "pick the {0} in this plot", "go harvest the {0}", "collect the ripe {0}", "pick all the {0}" };
	string TaskSentence(string crop)
	{
		string c = crop.ToLowerInvariant();
		string plural = c.Contains("tomato") ? "tomatoes" : c.Contains("pepper") ? "peppers" : c.Contains("corn") ? "corn" : c + "s";
		return string.Format(Verbs[rng.Next(Verbs.Length)], plural);
	}

	static string Num(float f) { return f.ToString("0.###", CultureInfo.InvariantCulture); }
	static string Esc(string s) { return s.Replace("\\", "\\\\").Replace("\"", "\\\""); }
}
