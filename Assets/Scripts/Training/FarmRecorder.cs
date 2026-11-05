using System;
using System.Globalization;
using System.IO;
using System.Text;
using Unity.Collections;
using UnityEngine;

//Records one robot at a fixed rate for training data: a small forward camera
//frame plus one CSV row per frame. FarmDataGen drives it; FarmPolicyLink sends
//the same frame and row to a live policy instead of (or as well as) a file.
//
//Files per episode directory:
//  frames.u8   N x size x size x 3 bytes, RGB, rows top to bottom, no header
//  steps.csv   one row per frame, columns in Header
//  plants.csv  every plant at the start: position, crop or weed, fruit count
//  meta.json   written by FarmDataGen: settings and results
//
//Coordinates: x east, z north, y up, metres, Unity world space. heading in
//degrees clockwise from north (Unity yaw). tgt_dx/tgt_dz are the current
//waypoint in the robot's own frame (x right, z ahead).
[DefaultExecutionOrder(100)] //after DebugRover, so this step's commands are known
public class FarmRecorder : MonoBehaviour
{
	public const string Header =
		"step,t,x,y,z,heading,v_fwd,v_side,yaw_rate,ax,ay,az,gx,gy,gz,charge," +
		"fwd,turn,light,fwd_exp,turn_exp,light_exp,source,arm_busy,arm_stage,basket_kg,picked," +
		"tgt_dx,tgt_dz,tgt_dist,tgt_kind,waypoints_left,crops_in_view,weeds_in_view,fruit_in_view," +
		"nearest_crop,nearest_weed,policy_ms";

	//source column
	public const int SourceExpert = 0, SourceRandom = 1, SourcePolicy = 2;

	public DebugRover rover;
	public HumanoidRobot human; //recorded instead of a rover (kinematic: velocities come from positions)
	public Camera cam;
	Vector3 humanPrevPos;
	float humanPrevHeading;
	bool humanHavePos;

	Transform Body { get { return rover != null ? rover.transform : human != null ? human.transform : null; } }
	public int size = 128;
	public float hz = 10f;

	//set by whoever drives: what produced the command for the coming interval
	public int source;
	public float execForward, execTurn;
	public bool execLight;
	public int picked; //fruit picked so far this episode (FarmDataGen counts them)
	public float policyMs;

	//called for each captured frame before its row is written: sets source,
	//execForward/execTurn/execLight (and policyMs) for the coming interval
	public Action<byte[], float[]> decide;
	//called after each captured frame is written
	public Action<byte[], float[]> onFrame;

	//column numbers, for anyone reading LastRow
	public static readonly int colFwd = Col("fwd"), colTurn = Col("turn"), colLight = Col("light"),
		colSource = Col("source"), colPicked = Col("picked"), colPolicyMs = Col("policy_ms"),
		colArmBusy = Col("arm_busy");
	public static int Col(string name) { return Array.IndexOf(Header.Split(','), name); }

	public int Step { get; private set; }
	public bool Recording { get { return steps != null; } }
	public byte[] LastFrame { get { return frame; } }
	public float[] LastRow { get { return row; } }

	RenderTexture rt;
	Texture2D tex;
	byte[] frame;
	float[] row;
	FileStream frames;
	StreamWriter steps;
	float accum, startTime;
	Vector3 prevVel;
	bool havePrev;
	int fixedPerFrame, fixedCount;
	bool live;

	public static FarmRecorder Attach(DebugRover r, int size, float hz)
	{
		FarmRecorder fr = r.GetComponent<FarmRecorder>();
		if (fr == null) fr = r.gameObject.AddComponent<FarmRecorder>();
		fr.rover = r;
		fr.size = size;
		fr.hz = hz;
		fr.Setup();
		return fr;
	}

	public static FarmRecorder Attach(HumanoidRobot h, int size, float hz)
	{
		FarmRecorder fr = h.GetComponent<FarmRecorder>();
		if (fr == null) fr = h.gameObject.AddComponent<FarmRecorder>();
		fr.human = h;
		fr.size = size;
		fr.hz = hz;
		fr.Setup();
		return fr;
	}

	void Setup()
	{
		if (cam != null) return;
		rt = new RenderTexture(size, size, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
		rt.antiAliasing = 1;
		rt.Create();
		tex = new Texture2D(size, size, TextureFormat.RGB24, false);
		frame = new byte[size * size * 3];
		row = new float[Header.Split(',').Length];

		//mount: above the robot's front, looking ahead and down. Measured with the
		//robot's own rotation undone, so it works for any heading.
		Quaternion keep = transform.rotation;
		Vector3 keepPos = transform.position;
		transform.rotation = Quaternion.identity;
		Bounds b = new Bounds(transform.position, Vector3.zero);
		bool any = false;
		foreach (Renderer r in GetComponentsInChildren<Renderer>())
		{
			if (r is ParticleSystemRenderer || r is LineRenderer || r is TrailRenderer) continue;
			if (!any) { b = r.bounds; any = true; } else b.Encapsulate(r.bounds);
		}
		transform.rotation = keep;
		transform.position = keepPos;
		float top = any ? b.max.y - keepPos.y : 1f;
		float front = any ? b.max.z - keepPos.z : 0.5f;

		GameObject go = new GameObject("TrainingCamera");
		go.transform.SetParent(transform, false);
		go.transform.localPosition = new Vector3(0f, top + 0.25f, front * 0.6f);
		go.transform.localRotation = Quaternion.Euler(28f, 0f, 0f);
		cam = go.AddComponent<Camera>();
		cam.enabled = false; //rendered by hand, at the recording rate only
		cam.fieldOfView = 75f;
		cam.nearClipPlane = 0.05f;
		cam.farClipPlane = 120f;
		cam.allowHDR = false;
		cam.allowMSAA = false;
		cam.targetTexture = rt;
		Camera main = Camera.main;
		cam.cullingMask = (main != null ? main.cullingMask : ~0) & ~(1 << 5); //no UI
		cam.clearFlags = main != null ? main.clearFlags : CameraClearFlags.Skybox;
		cam.backgroundColor = main != null ? main.backgroundColor : Color.grey;
		Debug.Log("[FarmRecorder] camera on " + Body.name + " at " + go.transform.localPosition + ", " + size + "x" + size + " at " + hz + " Hz");
	}

	//Start writing an episode. dir null = live only (frames go to onFrame, no files).
	public void Begin(string dir)
	{
		End();
		live = dir == null;
		if (!live)
		{
			Directory.CreateDirectory(dir);
			frames = new FileStream(Path.Combine(dir, "frames.u8"), FileMode.Create, FileAccess.Write, FileShare.Read, 1 << 20);
			steps = new StreamWriter(Path.Combine(dir, "steps.csv"), false, new UTF8Encoding(false), 1 << 16);
			steps.WriteLine(Header);
			WritePlants(Path.Combine(dir, "plants.csv"));
		}
		else
		{
			steps = StreamWriter.Null;
		}
		Step = 0;
		picked = 0;
		accum = 0f;
		startTime = Time.time;
		havePrev = false;
		humanHavePos = false;
		fixedCount = 0;
		fixedPerFrame = Mathf.Max(1, Mathf.RoundToInt(1f / (hz * Time.fixedDeltaTime)));
	}

	public void End()
	{
		if (frames != null) { frames.Flush(); frames.Dispose(); frames = null; }
		if (steps != null && steps != StreamWriter.Null) { steps.Flush(); steps.Dispose(); }
		steps = null;
	}

	void OnDestroy()
	{
		End();
		if (rt != null) rt.Release();
	}

	//Physics-step clock: exactly every fixedPerFrame physics steps, so the rate
	//holds in simulated time whatever the frame rate or Time.timeScale
	void FixedUpdate()
	{
		if (steps == null || (rover == null && human == null)) return;
		Vector3 vel = Vector3.zero;
		if (rover != null && rover.rb != null) vel = rover.rb.velocity;
		if (fixedCount++ % fixedPerFrame != 0) return;
		if (human != null)
		{
			Vector3 pos = human.transform.position;
			vel = humanHavePos ? (pos - humanPrevPos) * hz : Vector3.zero;
			humanPrevPos = pos;
		}
		Capture(vel);
	}

	void Capture(Vector3 vel)
	{
		//camera frame
		RenderTexture keep = RenderTexture.active;
		//the yellow marker over the next route point is drawn for people watching;
		//the robot's camera does not see it (the policy gets the route point as
		//numbers, not painted onto the ground)
		GameObject marker = PathMaker.Instance != null ? PathMaker.Instance.currentWayPointObj : null;
		bool markerShown = marker != null && marker.activeSelf;
		if (markerShown) marker.SetActive(false);
		cam.Render();
		if (markerShown) marker.SetActive(true);
		RenderTexture.active = rt;
		tex.ReadPixels(new Rect(0, 0, size, size), 0, 0, false);
		RenderTexture.active = keep;
		NativeArray<byte> raw = tex.GetRawTextureData<byte>();
		int stride = size * 3;
		for (int y = 0; y < size; y++)
			NativeArray<byte>.Copy(raw, (size - 1 - y) * stride, frame, y * stride, stride);

		FillRow(vel);
		//the driver (autopilot, a random detour or a live policy) picks the
		//command for the coming interval from this frame and row
		if (decide != null) decide(frame, row);
		row[colFwd] = execForward; row[colTurn] = execTurn; row[colLight] = execLight ? 1f : 0f;
		row[colSource] = source;
		row[colPicked] = picked;
		row[colPolicyMs] = policyMs;
		if (frames != null) frames.Write(frame, 0, frame.Length);
		if (steps != null && steps != StreamWriter.Null)
		{
			StringBuilder sb = new StringBuilder(256);
			for (int i = 0; i < row.Length; i++)
			{
				if (i > 0) sb.Append(',');
				sb.Append(row[i].ToString("0.#####", CultureInfo.InvariantCulture));
			}
			steps.WriteLine(sb.ToString());
		}
		if (onFrame != null) onFrame(frame, row);
		Step++;
	}

	void FillRow(Vector3 vel)
	{
		Transform tr = Body;
		Rigidbody rb = rover != null ? rover.rb : null;
		float heading = tr.eulerAngles.y;
		float yawRate = rb != null ? rb.angularVelocity.y : 0f;
		if (human != null)
		{
			yawRate = humanHavePos ? Mathf.DeltaAngle(humanPrevHeading, heading) * Mathf.Deg2Rad * hz : 0f;
			humanPrevHeading = heading;
			humanHavePos = true;
		}
		Quaternion yawInv = Quaternion.Inverse(Quaternion.Euler(0f, heading, 0f));
		Vector3 lv = yawInv * vel;
		float dt = 1f / hz;
		//accelerometer: specific force in the body frame (gravity included, as a real IMU reads it)
		Vector3 acc = havePrev ? (vel - prevVel) / dt : Vector3.zero;
		Vector3 imuA = tr.InverseTransformDirection(acc - Physics.gravity);
		Vector3 imuG = rb != null ? tr.InverseTransformDirection(rb.angularVelocity) : new Vector3(0f, yawRate, 0f);
		prevVel = vel;
		havePrev = true;

		//target waypoint, in the robot's frame
		float tdx = 0f, tdz = 0f, tdist = 0f, tkind = 0f;
		PathMaker.Waypoint wp = rover != null ? rover.currentWaypoint : human.currentWaypoint;
		if (wp != null)
		{
			Vector3 d = yawInv * new Vector3(wp.pos.x - tr.position.x, 0f, wp.pos.y - tr.position.z);
			tdx = d.x; tdz = d.z; tdist = d.magnitude;
			//1 drive, 2 pick fruit, 3 drive to a plant to test, 4 aim the probe, 5 drive with the UV light on
			tkind = wp.pickFruit ? 2f : wp.checkWater ? (wp.aimOnly ? 4f : 3f) : wp.light ? 5f : 1f;
		}

		//what is in view, from the plants' own labels (Plant.desired: crop or weed)
		int crops = 0, weeds = 0, fruit = 0;
		float nearCrop = 99f, nearWeed = 99f;
		Vector3 cp = cam.transform.position;
		foreach (Plant p in Plant.All)
		{
			if (p == null) continue;
			Vector3 pp = p.transform.position;
			float dist = Vector2.Distance(new Vector2(pp.x, pp.z), new Vector2(tr.position.x, tr.position.z));
			if (p.desired) nearCrop = Mathf.Min(nearCrop, dist); else nearWeed = Mathf.Min(nearWeed, dist);
			if ((pp - cp).sqrMagnitude > 15f * 15f) continue;
			Vector3 v = cam.WorldToViewportPoint(pp + Vector3.up * 0.2f);
			if (v.z <= 0f || v.x < 0f || v.x > 1f || v.y < 0f || v.y > 1f) continue;
			if (p.desired) { crops++; fruit += LiveFruit(p); } else weeds++;
		}

		RobotArm arm = rover != null ? rover.robotArm : null;
		bool busy = human != null ? human.harvesting : arm != null && arm.harvesting;
		int i = 0;
		row[i++] = Step;
		row[i++] = Time.time - startTime;
		row[i++] = tr.position.x; row[i++] = tr.position.y; row[i++] = tr.position.z;
		row[i++] = heading;
		row[i++] = lv.z; row[i++] = lv.x;
		row[i++] = yawRate;
		row[i++] = imuA.x; row[i++] = imuA.y; row[i++] = imuA.z;
		row[i++] = imuG.x; row[i++] = imuG.y; row[i++] = imuG.z;
		row[i++] = rover != null ? rover.charge : 100f;
		row[i++] = execForward; row[i++] = execTurn; row[i++] = execLight ? 1f : 0f;
		if (rover != null) { row[i++] = rover.expertForward; row[i++] = rover.expertTurn; row[i++] = rover.expertLight ? 1f : 0f; }
		else { row[i++] = human.expertForward; row[i++] = human.expertTurn; row[i++] = 0f; }
		row[i++] = source;
		row[i++] = busy ? 1f : 0f;
		row[i++] = arm != null ? arm.stage : -1f;
		row[i++] = rover != null && rover.basket != null ? rover.basket.weight : 0f;
		row[i++] = picked;
		row[i++] = tdx; row[i++] = tdz; row[i++] = tdist; row[i++] = tkind;
		row[i++] = PathMaker.Instance != null && PathMaker.Instance.waypoints != null ? PathMaker.Instance.waypoints.Count : 0;
		row[i++] = crops; row[i++] = weeds; row[i++] = fruit;
		row[i++] = nearCrop; row[i++] = nearWeed;
		row[i++] = policyMs;
	}

	//fruit still hanging on the plant (a picked one is let go: its body stops being kinematic)
	public static int LiveFruit(Plant p)
	{
		if (p.fruits == null) return 0;
		int n = 0;
		foreach (GameObject f in p.fruits)
		{
			if (f == null) continue;
			FruitBody fb = f.GetComponent<FruitBody>();
			if (fb == null || fb.rb == null || fb.rb.isKinematic) n++;
		}
		return n;
	}

	static void WritePlants(string path)
	{
		StringBuilder sb = new StringBuilder();
		sb.AppendLine("x,y,z,desired,fruit,name");
		foreach (Plant p in Plant.All)
		{
			if (p == null) continue;
			Vector3 pp = p.transform.position;
			sb.Append(pp.x.ToString("0.###", CultureInfo.InvariantCulture)).Append(',')
			  .Append(pp.y.ToString("0.###", CultureInfo.InvariantCulture)).Append(',')
			  .Append(pp.z.ToString("0.###", CultureInfo.InvariantCulture)).Append(',')
			  .Append(p.desired ? 1 : 0).Append(',')
			  .Append(LiveFruit(p)).Append(',')
			  .AppendLine(p.name.Replace("(Clone)", "").Replace(",", " "));
		}
		File.WriteAllText(path, sb.ToString());
	}
}
