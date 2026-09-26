using UnityEngine;
using UnityEngine.UI;

//Renders the scene into a smaller render texture and presents it full screen.
//
//In a built player Screen.SetResolution does this job for free and GraphicsSettings
//still uses it there. But Screen.SetResolution is a no-op in the editor, and the
//person this performance work exists for runs the project in the Linux editor with
//PRIME offload, not in a build. Without this, every tier below High gave him reduced
//shadows, grass and prop counts while still paying full Game-view fill rate, which is
//exactly the cost integrated graphics run out of first.
//
//Presentation goes through an Overlay canvas at a very low sorting order rather than
//the usual OnRenderImage blit on a second camera. The blit trick relies on image
//effect callbacks firing on a camera that draws nothing, and when it does not fire
//the screen goes black. A canvas draws in a defined order and costs one quad.
//
//There is still a second camera, but it only exists so that something renders to
//the display: it culls nothing and just clears. Without it no camera references
//Display 1 while the main camera draws into the texture, and the editor Game view
//paints its "No cameras rendering" notice over the picture.
//
//Anything that tells robot picture-in-picture cameras apart by "has a target
//texture" has to ask Owns() first, because the scaled main camera has one too.
[DisallowMultipleComponent]
public class RenderScaler : MonoBehaviour
{
	static RenderScaler instance;

	Camera cam;
	Camera presenter;
	RenderTexture rt;
	Canvas canvas;
	RawImage image;
	int screenW, screenH;

	public static bool Active { get { return instance != null && instance.rt != null; } }

	//size the scene is actually drawn at; the screen when nothing is scaled
	public static int RenderWidth { get { return Active ? instance.rt.width : Screen.width; } }
	public static int RenderHeight { get { return Active ? instance.rt.height : Screen.height; } }

	//True for the main camera while it renders into the scaler's texture. It then
	//has a targetTexture like a robot picture-in-picture camera, but it is still the
	//player's view and must keep getting the main view's settings.
	public static bool Owns(Camera camera)
	{
		return camera != null && Active && instance.cam == camera && camera.targetTexture == instance.rt;
	}

	//The clear-only camera that keeps the display referenced. Tier code skips it.
	public static bool IsPresenter(Camera camera)
	{
		return camera != null && instance != null && instance.presenter == camera;
	}

	//Scale the scene camera to w x h. Safe to call every time the tier changes.
	public static void Apply(Camera camera, int w, int h)
	{
		if (camera == null || w <= 0 || h <= 0) { Disable(); return; }

		if (instance == null)
		{
			GameObject go = new GameObject("RenderScaler");
			DontDestroyOnLoad(go);
			instance = go.AddComponent<RenderScaler>();
		}
		instance.Rebuild(camera, w, h);
	}

	public static void Disable()
	{
		if (instance != null) instance.Teardown();
	}

	//Input.mousePosition is in screen pixels, but a camera with a target texture
	//measures its viewport in that texture's pixels, so every screen-space ray has
	//to be converted or clicks land in the wrong place. Click-to-plant and robot
	//selection both go through this.
	public static Ray ScreenPointToRay(Camera camera, Vector3 screenPoint)
	{
		return camera.ScreenPointToRay(ScreenPoint(camera, screenPoint));
	}

	public static Vector3 ScreenPoint(Camera camera, Vector3 screenPoint)
	{
		if (camera == null || camera.targetTexture == null) return screenPoint;
		if (Screen.width <= 0 || Screen.height <= 0) return screenPoint;

		screenPoint.x *= camera.pixelWidth / (float)Screen.width;
		screenPoint.y *= camera.pixelHeight / (float)Screen.height;
		return screenPoint;
	}

	void Rebuild(Camera camera, int w, int h)
	{
		bool changed = cam != camera || rt == null || rt.width != w || rt.height != h;

		//a camera swap leaves the old one rendering into our texture
		if (cam != null && cam != camera && cam.targetTexture == rt) cam.targetTexture = null;
		cam = camera;

		if (rt != null && (rt.width != w || rt.height != h)) ReleaseTexture();

		if (rt == null)
		{
			rt = new RenderTexture(w, h, 24, RenderTextureFormat.Default);
			rt.name = "RenderScaler";
			rt.filterMode = FilterMode.Bilinear;
			rt.antiAliasing = Mathf.Max(1, QualitySettings.antiAliasing);
			rt.Create();
		}

		cam.targetTexture = rt;
		EnsurePresenter();
		image.texture = rt;
		image.enabled = cam.isActiveAndEnabled;
		screenW = Screen.width;
		screenH = Screen.height;

		//Apply runs on every focus change; only say something when it did something
		if (changed)
			Debug.Log("[RenderScaler] scene rendering at " + w + "x" + h
				+ " presented at " + Screen.width + "x" + Screen.height + " (camera " + cam.name + ")");
	}

	void EnsurePresenter()
	{
		if (presenter == null)
		{
			presenter = gameObject.AddComponent<Camera>();
			presenter.cullingMask = 0;
			presenter.clearFlags = CameraClearFlags.SolidColor;
			presenter.backgroundColor = Color.black;
			//before every scene camera, so anything else that draws to the screen
			//lands on top of the clear rather than under it
			presenter.depth = -100f;
			presenter.useOcclusionCulling = false;
			presenter.allowHDR = false;
			presenter.allowMSAA = false;
			presenter.nearClipPlane = 0.1f;
			presenter.farClipPlane = 1f;
		}
		presenter.enabled = true;

		if (canvas != null) return;

		GameObject go = new GameObject("RenderScalerPresenter");
		go.transform.SetParent(transform, false);
		canvas = go.AddComponent<Canvas>();
		canvas.renderMode = RenderMode.ScreenSpaceOverlay;
		//well below anything the game puts on screen, so the HUD still draws on top
		//at full resolution and stays crisp
		canvas.sortingOrder = -1000;

		GameObject imageGo = new GameObject("Scene");
		imageGo.transform.SetParent(go.transform, false);
		image = imageGo.AddComponent<RawImage>();
		image.raycastTarget = false;

		RectTransform rect = image.rectTransform;
		rect.anchorMin = Vector2.zero;
		rect.anchorMax = Vector2.one;
		rect.offsetMin = Vector2.zero;
		rect.offsetMax = Vector2.zero;
	}

	void Teardown()
	{
		bool was = rt != null;
		if (cam != null && cam.targetTexture == rt) cam.targetTexture = null;
		cam = null;
		ReleaseTexture();
		if (canvas != null) Destroy(canvas.gameObject);
		canvas = null;
		image = null;
		if (presenter != null) presenter.enabled = false;
		if (was) Debug.Log("[RenderScaler] off, scene renders at the Game view size");
	}

	void ReleaseTexture()
	{
		if (rt == null) return;
		if (image != null) image.texture = null;
		rt.Release();
		Destroy(rt);
		rt = null;
	}

	void LateUpdate()
	{
		if (rt == null) return;

		//the editor Game view is resized constantly; follow it so the presented
		//image keeps the right aspect instead of stretching
		if (Screen.width != screenW || Screen.height != screenH)
		{
			screenW = Screen.width;
			screenH = Screen.height;
			GraphicsSettings.RefreshRenderScale();
			return;
		}

		//a different camera became the main one: scale that one instead
		Camera main = Camera.main;
		if (main != null && main != cam)
		{
			GraphicsSettings.RefreshRenderScale();
			return;
		}

		//something else claimed the camera (a cutscene, another effect); take it back
		if (cam != null && cam.targetTexture != rt) cam.targetTexture = rt;

		//a disabled or destroyed camera leaves its last frame in the texture, and
		//presenting that would freeze the screen over whatever renders instead
		bool live = cam != null && cam.isActiveAndEnabled;
		if (image != null && image.enabled != live) image.enabled = live;
	}

	void OnDestroy()
	{
		if (instance == this) instance = null;
		Teardown();
	}
}
