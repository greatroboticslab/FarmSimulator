using System.Collections;
using System.IO;
using UnityEngine;

//A fixed set of screenshots taken at the end of a -benchmark run when
//-benchmark-tour is given, so visual changes can be judged from the same views
//every time: the working view along the rows, the sky toward the sun, an
//overview of the farm, the robot up close, and the lit side away from the sun.
//
//The benchmark's own screenshot is whatever the gameplay camera shows, which is
//mostly ground; that is fine for a frame-time record and useless for judging a
//sky, a tree line or a robot's materials.
public static class BenchmarkTour
{
	public static IEnumerator Run(string dir, string prefix)
	{
		Camera cam = Camera.main;
		if (cam == null || PathMaker.Instance == null || PathMaker.Instance.rover == null) yield break;

		//the gameplay camera script moves the camera every frame
		MainCam driver = cam.GetComponentInParent<MainCam>();
		bool driverWas = driver != null && driver.enabled;
		if (driver != null) driver.enabled = false;
		Vector3 savedPos = cam.transform.position;
		Quaternion savedRot = cam.transform.rotation;
		float savedFov = cam.fieldOfView;

		Transform rover = PathMaker.Instance.rover.transform;
		Vector3 r = rover.position;
		Vector3 f = rover.forward; f.y = 0f;
		if (f.sqrMagnitude < 1e-4f) f = Vector3.forward;
		f.Normalize();
		Vector3 right = Vector3.Cross(Vector3.up, f);

		Vector3 toSun = RenderSettings.sun != null ? -RenderSettings.sun.transform.forward : Vector3.up;
		Vector3 sunFlat = new Vector3(toSun.x, 0f, toSun.z);
		if (sunFlat.sqrMagnitude < 1e-4f) sunFlat = f;
		sunFlat.Normalize();

		yield return Shot(cam, dir, prefix + "_field", r - f * 10f + Vector3.up * 2.2f, r + f * 25f, 60f);
		yield return Shot(cam, dir, prefix + "_sky", r + Vector3.up * 1.8f, r + Vector3.up * 1.8f + sunFlat * 100f + Vector3.up * 22f, 70f);
		yield return Shot(cam, dir, prefix + "_overview", r - f * 70f + Vector3.up * 45f, r + f * 20f, 60f);
		//frame the robot by what it renders: its transform can sit well away
		//from the body
		Bounds robot = new Bounds(r, Vector3.one);
		bool any = false;
		foreach (Renderer rr in PathMaker.Instance.rover.GetComponentsInChildren<Renderer>())
		{
			if (rr is ParticleSystemRenderer || rr is LineRenderer || rr is TrailRenderer) continue;
			if (!any) { robot = rr.bounds; any = true; }
			else robot.Encapsulate(rr.bounds);
		}
		float reach = Mathf.Clamp(robot.extents.magnitude, 0.5f, 4f);
		Vector3 side = (right * 0.8f - f * 0.6f).normalized;
		yield return Shot(cam, dir, prefix + "_robot", robot.center + side * reach * 2.6f + Vector3.up * reach * 1.1f, robot.center, 40f);
		yield return Shot(cam, dir, prefix + "_backlit", r + sunFlat * 9f + Vector3.up * 2.5f, r - sunFlat * 30f, 60f);
		//the same view again a moment later: any difference in the foliage is wind
		yield return new WaitForSecondsRealtime(0.7f);
		yield return Shot(cam, dir, prefix + "_backlit2", r + sunFlat * 9f + Vector3.up * 2.5f, r - sunFlat * 30f, 60f);

		//close-ups of the nearest scattered tree and rock, lit from the camera's side
		Transform tree = NearestProp(r, "Oak", "Elm", "Cedar");
		if (tree != null)
		{
			Vector3 t = tree.position;
			Vector3 from = t + sunFlat * 16f + Vector3.up * 3f;
			yield return Shot(cam, dir, prefix + "_tree", from, t + Vector3.up * 5f, 60f);
		}
		Transform rock = NearestProp(r, "Rock", "Stone", "Boulder");
		if (rock != null)
		{
			Vector3 k = rock.position;
			Vector3 from = k + sunFlat * 3.2f + Vector3.up * 1.6f;
			yield return Shot(cam, dir, prefix + "_rock", from, k + Vector3.up * 0.3f, 55f);
		}

		cam.transform.SetPositionAndRotation(savedPos, savedRot);
		cam.fieldOfView = savedFov;
		if (driver != null) driver.enabled = driverWas;
	}

	//scattered props are children of the map's ObjectPlacer, named after their
	//prefab ("Oak_B(Clone)")
	static Transform NearestProp(Vector3 near, params string[] prefixes)
	{
		ObjectPlacer placer = Object.FindObjectOfType<ObjectPlacer>();
		if (placer == null) return null;
		Transform best = null;
		float bestD = float.MaxValue;
		foreach (Transform t in placer.transform)
		{
			bool match = false;
			foreach (string p in prefixes) if (t.name.StartsWith(p)) { match = true; break; }
			if (!match) continue;
			float d = (t.position - near).sqrMagnitude;
			if (d < bestD) { bestD = d; best = t; }
		}
		return best;
	}

	static IEnumerator Shot(Camera cam, string dir, string name, Vector3 from, Vector3 lookAt, float fov)
	{
		//stay above the ground wherever the pose lands
		RaycastHit hit;
		if (Physics.Raycast(from + Vector3.up * 200f, Vector3.down, out hit, 400f, ~0, QueryTriggerInteraction.Ignore)
			&& from.y < hit.point.y + 1.2f)
			from.y = hit.point.y + 1.2f;

		cam.transform.SetPositionAndRotation(from, Quaternion.LookRotation(lookAt - from, Vector3.up));
		cam.fieldOfView = fov;
		//a few frames for LOD, grass and the throttled robot cameras to settle
		for (int i = 0; i < 4; i++) yield return null;
		ScreenCapture.CaptureScreenshot(Path.Combine(dir, name + ".png"));
		yield return null;
		yield return new WaitForSecondsRealtime(0.3f);
	}
}
