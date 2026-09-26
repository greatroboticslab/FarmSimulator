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
		yield return Shot(cam, dir, prefix + "_robot", r + right * 2.6f - f * 2.4f + Vector3.up * 1.6f, r + Vector3.up * 0.7f, 50f);
		yield return Shot(cam, dir, prefix + "_backlit", r + sunFlat * 9f + Vector3.up * 2.5f, r - sunFlat * 30f, 60f);
		//the same view again a moment later: any difference in the foliage is wind
		yield return new WaitForSecondsRealtime(0.7f);
		yield return Shot(cam, dir, prefix + "_backlit2", r + sunFlat * 9f + Vector3.up * 2.5f, r - sunFlat * 30f, 60f);

		cam.transform.SetPositionAndRotation(savedPos, savedRot);
		cam.fieldOfView = savedFov;
		if (driver != null) driver.enabled = driverWas;
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
