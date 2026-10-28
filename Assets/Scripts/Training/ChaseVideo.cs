using System.IO;
using UnityEngine;

//A chase-camera view saved as numbered JPGs at 15 fps of simulated time, for a demo clip
public class ChaseVideo : MonoBehaviour
{
	Transform target;
	string dir, prefix;
	Camera cam;
	RenderTexture rt;
	Texture2D tex;
	int n;
	float next;
	bool on;
	Vector3 smoothPos;

	public static ChaseVideo Attach(Transform target, string dir)
	{
		Directory.CreateDirectory(dir);
		GameObject go = new GameObject("ChaseVideo");
		ChaseVideo v = go.AddComponent<ChaseVideo>();
		v.target = target;
		v.dir = dir;
		v.rt = new RenderTexture(960, 540, 24, RenderTextureFormat.ARGB32, RenderTextureReadWrite.sRGB);
		v.rt.antiAliasing = 4;
		v.tex = new Texture2D(960, 540, TextureFormat.RGB24, false);
		v.cam = go.AddComponent<Camera>();
		v.cam.enabled = false;
		v.cam.fieldOfView = 55f;
		v.cam.farClipPlane = 600f;
		v.cam.targetTexture = v.rt;
		Camera main = Camera.main;
		v.cam.cullingMask = (main != null ? main.cullingMask : ~0) & ~(1 << 5);
		return v;
	}

	public void Begin(string p) { prefix = p; n = 0; next = Time.time; on = true; smoothPos = Want(); }
	public void End() { on = false; }

	Vector3 Want()
	{
		Vector3 back = -target.forward; back.y = 0f;
		if (back.sqrMagnitude < 0.01f) back = Vector3.back;
		return target.position + back.normalized * 5.5f + Vector3.up * 3.2f;
	}

	void LateUpdate()
	{
		if (!on || target == null || Time.time < next) return;
		next += 1f / 15f;
		smoothPos = Vector3.Lerp(smoothPos, Want(), 0.15f);
		cam.transform.position = smoothPos;
		cam.transform.LookAt(target.position + Vector3.up * 0.6f);
		cam.Render();
		RenderTexture keep = RenderTexture.active;
		RenderTexture.active = rt;
		tex.ReadPixels(new Rect(0, 0, 960, 540), 0, 0, false);
		RenderTexture.active = keep;
		File.WriteAllBytes(Path.Combine(dir, prefix + "_" + n.ToString("00000") + ".jpg"), tex.EncodeToJPG(88));
		n++;
	}
}
