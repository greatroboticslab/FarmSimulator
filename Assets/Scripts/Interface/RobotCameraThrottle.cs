using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

//Every robot prefab carries a camera that draws the scene again into a
//512x512 texture for the picture-in-picture panel, recordings and the
//optional socket feed. Left enabled, Unity renders it every frame, which
//roughly doubles the GPU work. This component takes the camera out of the
//normal render loop and renders it by hand: only while something is
//actually looking at the texture, at a tier-dependent rate, without
//post-processing and with reduced shadows on the low tiers.
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

	Camera cam;
	PostProcessLayer post;
	float nextRender;

	void Awake()
	{
		cam = GetComponent<Camera>();
		post = GetComponent<PostProcessLayer>();
		//the prefabs also carry an AudioListener each; the scene camera already has one
		AudioListener listener = GetComponent<AudioListener>();
		if (listener != null) listener.enabled = false;
	}

	void OnEnable()
	{
		if (cam != null) cam.enabled = false;
	}

	void LateUpdate()
	{
		if (cam == null || cam.targetTexture == null) return;
		if (cam.enabled) cam.enabled = false;

		bool socket = SocketInterace.Instance != null && SocketInterace.Instance.enableSocket && SocketInterace.Instance.started;
		if (!socket && !PanelVisible()) return;

		float interval = (socket || fps <= 0f) ? 0f : 1f / fps;
		if (Time.unscaledTime < nextRender) return;
		nextRender = Time.unscaledTime + interval;

		if (post != null) post.enabled = !stripPostProcessing;

		float saved = QualitySettings.shadowDistance;
		QualitySettings.shadowDistance = Mathf.Min(saved, shadowDistance);
		cam.Render();
		QualitySettings.shadowDistance = saved;
	}

	//The HUD shows the robot view in a small panel and, via Toggle View, full screen
	static bool PanelVisible()
	{
		Manip manip = PathMaker.Instance != null ? PathMaker.Instance.manip : null;
		if (manip == null) return true;
		if (manip.smallCamScreen != null && manip.smallCamScreen.activeInHierarchy) return true;
		if (manip.fullCamScreen != null && manip.fullCamScreen.activeInHierarchy) return true;
		return false;
	}
}
