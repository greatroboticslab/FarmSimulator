using UnityEngine;

//Display dressing for the robot select screen: a glowing ring pad under each
//robot that brightens when the mouse hovers it, so the screen reads as an
//interactive showroom instead of models parked on a blank slab.
public class RobotShowcase : MonoBehaviour
{
	public Color ringColor = new Color(0.15f, 0.8f, 1f);

	RobotInfo info;
	Material ringMat;
	float glow;

	static Texture2D ringTexture;

	void Start()
	{
		info = GetComponent<RobotInfo>();

		//thin glowing outline on a flat quad; a filled disc reads like a lily pad
		GameObject ring = GameObject.CreatePrimitive(PrimitiveType.Quad);
		ring.name = "ShowcaseRing";
		Object.Destroy(ring.GetComponent<Collider>());
		ring.transform.SetParent(transform.parent, false);
		ring.transform.position = new Vector3(transform.position.x, 0.02f, transform.position.z);
		ring.transform.rotation = Quaternion.Euler(90f, 0f, 0f);
		ring.transform.localScale = new Vector3(1.9f, 1.9f, 1f);

		ringMat = new Material(Shader.Find("Legacy Shaders/Particles/Additive"));
		ringMat.mainTexture = GetRingTexture();
		ringMat.SetColor("_TintColor", ringColor * 0.45f);
		ring.GetComponent<MeshRenderer>().material = ringMat;
	}

	void Update()
	{
		bool hovered = info != null && info.hovering;
		if (info != null) info.hovering = false; //consume; MainCam re-sets it while hovering

		glow = Mathf.Lerp(glow, hovered ? 1f : 0f, Time.deltaTime * 10f);
		if (ringMat != null)
		{
			float pulse = hovered ? 1f + 0.2f * Mathf.Sin(Time.time * 6f) : 1f;
			ringMat.SetColor("_TintColor", ringColor * Mathf.Lerp(0.45f, 1.4f * pulse, glow));
		}
	}

	static Texture2D GetRingTexture()
	{
		if (ringTexture != null) return ringTexture;

		const int size = 256;
		const float outer = 0.48f, inner = 0.36f, soft = 0.03f;
		ringTexture = new Texture2D(size, size, TextureFormat.RGBA32, true);
		for (int y = 0; y < size; y++)
		{
			for (int x = 0; x < size; x++)
			{
				float dx = (x + 0.5f) / size - 0.5f;
				float dy = (y + 0.5f) / size - 0.5f;
				float r = Mathf.Sqrt(dx * dx + dy * dy);
				float a = Mathf.Clamp01((r - inner) / soft) * Mathf.Clamp01((outer - r) / soft);
				ringTexture.SetPixel(x, y, new Color(1f, 1f, 1f, a));
			}
		}
		ringTexture.Apply();
		ringTexture.wrapMode = TextureWrapMode.Clamp;
		return ringTexture;
	}
}
