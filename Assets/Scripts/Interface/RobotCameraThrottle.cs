using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Rendering.PostProcessing;

//Every robot prefab carries a camera that draws the scene again into a
//512x512 texture for the picture-in-picture panel, recordings and the
//optional socket feed. Left enabled, Unity renders each of them every
//frame, which multiplies the GPU work by the number of robots on the farm.
//
//This component takes those cameras out of the normal render loop and
//renders them by hand: only the one camera that owns a given texture, only
//while something is actually looking at that texture, at a tier-dependent
//rate, without post-processing and with reduced shadows on the low tiers.
//
//Recording keeps working unchanged because RobotCamera.CaptureFrame calls
//cam.Render() itself; the socket feed forces a render every frame.
public class RobotCameraThrottle : MonoBehaviour
{
	//renders per second for the picture-in-picture; 0 = every frame
	public static float fps = 10f;
	//shadow distance used while the robot camera renders
	public static float shadowDistance = 0f;
	//drop bloom/FXAA for the small view on the low tiers
	public static bool stripPostProcessing = true;
	//HDR and view distance for the small view, from the same tier table
	public static bool allowHdr = false;
	public static float farClip = 1000f;

	//Several robots, and several cameras on one robot, share cameraShot. Only
	//one image can be in the panel, so only one camera per texture renders.
	static readonly Dictionary<RenderTexture, RobotCameraThrottle> owners = new Dictionary<RenderTexture, RobotCameraThrottle>();

	//textures the HUD is currently showing, refreshed every half second
	static readonly HashSet<Texture> displayed = new HashSet<Texture>();
	static float displayedAt = -99f;

	Camera cam;
	PostProcessLayer post;
	RenderTexture target;
	float originalFar;
	float nextRender;

	void Awake()
	{
		cam = GetComponent<Camera>();
		post = GetComponent<PostProcessLayer>();
		if (cam != null)
		{
			target = cam.targetTexture;
			originalFar = cam.farClipPlane;
		}
		//the scene camera already has a listener; every robot prefab carries one too
		AudioListener listener = GetComponent<AudioListener>();
		if (listener != null) listener.enabled = false;
	}

	void OnEnable()
	{
		if (cam != null) cam.enabled = false;
	}

	void OnDisable()
	{
		RobotCameraThrottle owner;
		if (target != null && owners.TryGetValue(target, out owner) && owner == this) owners.Remove(target);
	}

	void LateUpdate()
	{
		if (cam == null || cam.targetTexture == null) return;
		if (cam.enabled) cam.enabled = false;
		target = cam.targetTexture;

		//applied even when this camera is idle, so an explicit Render() from
		//recording or the socket uses the tier's settings too
		cam.allowHDR = allowHdr;
		cam.allowMSAA = false;
		cam.farClipPlane = Mathf.Min(originalFar, farClip);
		if (post != null) post.enabled = !stripPostProcessing;

		if (!ClaimOwnership()) return;

		bool socket = SocketInterace.Instance != null && SocketInterace.Instance.enableSocket
			&& SocketInterace.Instance.started && SocketInterace.Instance.frame == target;
		if (!socket && !IsDisplayed(target)) return;

		float interval = (socket || fps <= 0f) ? 0f : 1f / fps;
		if (Time.unscaledTime < nextRender) return;
		nextRender = Time.unscaledTime + interval;

		float saved = QualitySettings.shadowDistance;
		QualitySettings.shadowDistance = Mathf.Min(saved, shadowDistance);
		cam.Render();
		QualitySettings.shadowDistance = saved;
	}

	//First camera to ask owns the texture, so the robot the player drives keeps
	//the panel when companions spawn later. A robot's own recording camera wins
	//over its other cameras, since that is the view the panel is meant to show.
	bool ClaimOwnership()
	{
		RobotCameraThrottle owner;
		if (!owners.TryGetValue(target, out owner) || owner == null || !owner.isActiveAndEnabled)
		{
			owners[target] = this;
			return true;
		}
		if (owner == this) return true;

		if (owner.GetComponent<RobotCamera>() == null && GetComponent<RobotCamera>() != null
			&& owner.transform.root == transform.root)
		{
			owners[target] = this;
			return true;
		}
		return false;
	}

	//A texture nothing is showing does not need to be redrawn at all. The farm
	//HUD puts the robot view in a small panel and, via Toggle View, full screen.
	static bool IsDisplayed(RenderTexture rt)
	{
		if (Time.unscaledTime - displayedAt > 0.5f)
		{
			displayedAt = Time.unscaledTime;
			displayed.Clear();
			foreach (RawImage img in UnityEngine.Object.FindObjectsOfType<RawImage>())
			{
				if (img.texture != null && img.isActiveAndEnabled) displayed.Add(img.texture);
			}
		}
		return displayed.Contains(rt);
	}
}
