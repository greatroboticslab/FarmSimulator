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

	void Start()
	{
		info = GetComponent<RobotInfo>();

		GameObject ring = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
		ring.name = "ShowcaseRing";
		Object.Destroy(ring.GetComponent<Collider>());
		//parented to the stage, not the robot, so physics jitter cannot move it
		ring.transform.SetParent(transform.parent, false);
		ring.transform.position = new Vector3(transform.position.x, 0.015f, transform.position.z);
		ring.transform.localScale = new Vector3(1.65f, 0.012f, 1.65f);

		ringMat = new Material(Shader.Find("Standard"));
		ringMat.color = new Color(0.05f, 0.07f, 0.09f);
		ringMat.EnableKeyword("_EMISSION");
		ringMat.SetColor("_EmissionColor", ringColor * 0.15f);
		ring.GetComponent<MeshRenderer>().material = ringMat;
	}

	void Update()
	{
		bool hovered = info != null && info.hovering;
		if (info != null) info.hovering = false; //consume; MainCam re-sets it while hovering

		glow = Mathf.Lerp(glow, hovered ? 1f : 0f, Time.deltaTime * 10f);
		if (ringMat != null)
		{
			float pulse = hovered ? 1f + 0.15f * Mathf.Sin(Time.time * 6f) : 1f;
			ringMat.SetColor("_EmissionColor", ringColor * Mathf.Lerp(0.15f, 3.2f * pulse, glow));
		}
	}
}
