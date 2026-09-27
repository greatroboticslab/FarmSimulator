using System;
using System.Collections;
using System.Globalization;
using UnityEngine;

//"-benchmark-selfdrive <seconds>": after the benchmark has planted its plot, turn
//Self Driving on and let the robot work it, then report what happened. A
//functional check for the harvest loop, run the same unattended way as the
//frame-time measurement (see GraphicsSettings.BenchmarkRoutine):
//
//  [SelfDrive] 90 s: waypoints 12 of 40 left, plants visited 9, pick cycles 14
//    (dropped in basket 11, timed out 3), nothing to pick 0, basket 1.10 kg,
//    moved 38.2 m, exceptions 0
//
//Progress is also logged every 10 s so a stall shows where it happened.
public static class SelfDriveCheck
{
	//shotDir: where to save one screenshot halfway through (null for none)
	public static IEnumerator Run(float seconds, string shotDir = null)
	{
		PathMaker pm = PathMaker.Instance;
		DebugRover rover = pm != null ? pm.rover : null;
		RoverControls controls = UnityEngine.Object.FindObjectOfType<RoverControls>();
		if (rover == null || controls == null || controls.selfDriving == null)
		{
			Debug.LogError("[SelfDrive] no rover or no Self Driving toggle; nothing checked");
			yield break;
		}

		int exceptions = 0, nothingToPick = 0, timedOut = 0, starts = 0;
		string firstException = null;
		Application.LogCallback count = (message, stack, type) =>
		{
			if (type == LogType.Exception || (type == LogType.Error && message.Contains("Exception")))
			{
				exceptions++;
				if (firstException == null) firstException = message + " | " + FirstLine(stack);
			}
			else if (message.Contains("nothing to pick")) nothingToPick++;
			else if (message.Contains("skipping fruit")) timedOut++;
			else if (message == "Harvesting!") starts++;
		};
		Application.logMessageReceived += count;

		RobotArm arm = rover.robotArm;
		int waypointsAtStart = pm.waypoints != null ? pm.waypoints.Count : 0;
		int drops = 0, lastStage = arm != null ? arm.stage : 0;
		Plant lastPlant = null;
		int plantsVisited = 0;
		Vector3 lastPos = rover.transform.position;
		float moved = 0f;
		float basketAtStart = rover.basket != null ? rover.basket.weight : 0f;

		Debug.Log("[SelfDrive] start: robot " + rover.name + ", crop " + (pm.selectedCrop != null ? pm.selectedCrop.name : "none")
			+ ", " + waypointsAtStart + " waypoints, arm " + (arm != null ? "yes" : "no"));
		LogFigures();
		controls.selfDriving.isOn = true;

		float start = Time.unscaledTime, nextReport = start + 10f;
		bool shot = string.IsNullOrEmpty(shotDir);
		//and one early, while the robot is still crossing the grass to its plot
		bool earlyShot = shot;
		//a moisture reading resets the rover's timeSinceMeasure
		int readings = 0;
		float lastSince = rover.timeSinceMeasure;
		while (Time.unscaledTime - start < seconds)
		{
			yield return null;
			if (!earlyShot && Time.unscaledTime - start >= 4f)
			{
				earlyShot = true;
				string early = System.IO.Path.Combine(shotDir, "selfdrive_" + rover.name.Replace("(Clone)", "") + "_early.png");
				ScreenCapture.CaptureScreenshot(early);
				Debug.Log("[SelfDrive] screenshot " + early);
			}
			if (!shot && Time.unscaledTime - start >= seconds * 0.5f)
			{
				shot = true;
				string crop = pm.selectedCrop != null ? pm.selectedCrop.name : "none";
				string file = System.IO.Path.Combine(shotDir, "selfdrive_" + rover.name.Replace("(Clone)", "") + "_" + crop + ".png");
				ScreenCapture.CaptureScreenshot(file);
				Debug.Log("[SelfDrive] screenshot " + file);
			}
			Vector3 p = rover.transform.position;
			moved += Vector3.Distance(p, lastPos);
			lastPos = p;
			if (rover.timeSinceMeasure < lastSince) readings++;
			lastSince = rover.timeSinceMeasure;
			if (arm != null)
			{
				//stage 4 is the drop over the basket; leaving it means the fruit went in
				if (lastStage == 4 && arm.stage != 4) drops++;
				lastStage = arm.stage;
				if (arm.harvesting && arm.currentCrop != lastPlant)
				{
					lastPlant = arm.currentCrop;
					plantsVisited++;
				}
			}
			if (Time.unscaledTime >= nextReport)
			{
				nextReport += 10f;
				Debug.Log(string.Format(CultureInfo.InvariantCulture,
					"[SelfDrive] {0:0} s: at ({1:0.0}, {2:0.0}), waypoints left {3}, harvesting {4}, stage {5}, fruit {6}, moved {7:0.0} m, exceptions {8}, HUD \"{9}\"",
					Time.unscaledTime - start, p.x, p.z, pm.waypoints != null ? pm.waypoints.Count : 0,
					arm != null && arm.harvesting, arm != null ? arm.stage : -1, arm != null ? arm.currentFruit : -1, moved, exceptions,
					RobotStatus.Current ?? ""));
			}
		}

		controls.selfDriving.isOn = false;
		Application.logMessageReceived -= count;
		float basket = (rover.basket != null ? rover.basket.weight : 0f) - basketAtStart;
		Debug.Log(string.Format(CultureInfo.InvariantCulture,
			"[SelfDrive] {0:0} s: waypoints {1} of {2} left, plants visited {3}, pick cycles {4} (dropped in basket {5}, timed out {6}), nothing to pick {7}, basket {8:0.00} kg{9}, moved {10:0.0} m, exceptions {11}",
			seconds, pm.waypoints != null ? pm.waypoints.Count : 0, waypointsAtStart, plantsVisited, starts, drops, timedOut, nothingToPick,
			basket, rover.basket != null ? "" : " (no basket on this robot)", moved, exceptions));
		if (firstException != null) Debug.Log("[SelfDrive] first exception: " + firstException);

		//what the UV lamp and the moisture probe did to and with the plot
		//(Plant.Start sets crops to mold 20, health 40; weeds to mold 0)
		int crops = 0, moldDown = 0, deadCrops = 0, weeds = 0, deadWeeds = 0;
		foreach (Plant pl in Plant.All)
		{
			if (pl == null) continue;
			if (pl.desired)
			{
				crops++;
				if (pl.mold < 20f) moldDown++;
				if (pl.health <= 0f) deadCrops++;
			}
			else
			{
				weeds++;
				if (pl.health <= 0f) deadWeeds++;
			}
		}
		Debug.Log("[SelfDrive] plants: crops " + crops + " (mold reduced on " + moldDown + ", dead " + deadCrops + "), weeds " + weeds
			+ " (dead " + deadWeeds + "), moisture readings " + readings);

		//and Main Menu should take the farm's contents with it
		int plantsBefore = UnityEngine.Object.FindObjectsOfType<Plant>(true).Length;
		int plotsBefore = UnityEngine.Object.FindObjectsOfType<PlantZone>(true).Length;
		pm.GoToMainMenu();
		//Destroy lands at the end of the frame
		yield return null;
		yield return null;
		Debug.Log("[SelfDrive] Main Menu: plants " + plantsBefore + " -> " + UnityEngine.Object.FindObjectsOfType<Plant>(true).Length
			+ ", plots " + plotsBefore + " -> " + UnityEngine.Object.FindObjectsOfType<PlantZone>(true).Length
			+ ", waypoints queued " + (pm.waypoints != null ? pm.waypoints.Count : 0) + ", marker " + (pm.currentWayPointObj != null ? "left" : "gone"));
	}

	//Every animated figure in the farm (people, humanoid robots) with how far its
	//lowest visible point is below the ground under it, to catch anything
	//spawned into the terrain
	static void LogFigures()
	{
		int total = 0, onGround = 0, sunk = 0, floating = 0, listed = 0;
		foreach (Animator a in UnityEngine.Object.FindObjectsOfType<Animator>())
		{
			if (a.GetComponentInParent<Animator>() != a) continue;
			Renderer[] rs = a.GetComponentsInChildren<Renderer>();
			if (rs.Length == 0) continue;
			Bounds b = rs[0].bounds;
			foreach (Renderer r in rs) b.Encapsulate(r.bounds);
			Vector3 p = a.transform.position;
			Terrain t = TerrainUnder(p);
			total++;
			if (t == null) continue;
			//how far the lowest visible point is from the ground: feet a few
			//centimetres into the grass are fine, a body into the ground or a
			//person in mid-air is not
			float off = b.min.y - (t.SampleHeight(p) + t.transform.position.y);
			if (Mathf.Abs(off) <= 0.15f) { onGround++; continue; }
			if (off < 0f) sunk++; else floating++;
			if (listed++ < 10)
				Debug.Log(string.Format(CultureInfo.InvariantCulture, "[SelfDrive] figure {0} at ({1:0.0}, {2:0.0}, {3:0.0}), {4:0.00} m tall, lowest point {5:0.00} m {6} the ground",
					a.name, p.x, p.y, p.z, b.size.y, Mathf.Abs(off), off < 0f ? "below" : "above"));
		}
		Debug.Log("[SelfDrive] figures: " + total + " (on the ground " + onGround + ", sunk more than 15 cm " + sunk + ", floating more than 15 cm " + floating + ")");
	}

	static Terrain TerrainUnder(Vector3 p)
	{
		foreach (Terrain t in Terrain.activeTerrains)
		{
			if (t == null || t.terrainData == null) continue;
			Vector3 o = t.transform.position, s = t.terrainData.size;
			if (p.x >= o.x && p.x <= o.x + s.x && p.z >= o.z && p.z <= o.z + s.z) return t;
		}
		return null;
	}

	static string FirstLine(string s)
	{
		if (string.IsNullOrEmpty(s)) return "";
		int n = s.IndexOf('\n');
		return n < 0 ? s : s.Substring(0, n);
	}
}
