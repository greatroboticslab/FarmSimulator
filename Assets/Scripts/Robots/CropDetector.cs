using System.Collections.Generic;
using UnityEngine;

//Crop detection and laser projection for the farm robots.
//Scans for plants around the robot, classifies them as crops or weeds,
//and projects a visible laser beam onto the current target so you can
//see exactly which plant the robot has detected.
public class CropDetector : MonoBehaviour
{
	[Header("Detection")]
	public float range = 7f;
	public float scanInterval = 0.25f;
	public bool preferWeeds = true; //cutter/UV style robots hunt weeds first

	[Header("Laser")]
	public bool laserEnabled = true;
	public Color laserColor = new Color(1f, 0.15f, 0.1f);
	public Transform emitter; //defaults to a point above the robot body

	[Header("State (read only)")]
	public int cropsInRange;
	public int weedsInRange;
	public Plant currentTarget;

	LineRenderer beam;
	GameObject hitDot;
	Light hitLight;
	float scanTimer;
	Plant lastLogged;

	void Start()
	{
		if (emitter == null)
		{
			GameObject e = new GameObject("LaserEmitter");
			e.transform.SetParent(transform, false);
			e.transform.localPosition = new Vector3(0f, 1.1f, 0.4f);
			emitter = e.transform;
		}

		Material glow = new Material(Shader.Find("Legacy Shaders/Particles/Additive"));
		glow.SetColor("_TintColor", laserColor);

		GameObject beamGO = new GameObject("LaserBeam");
		beamGO.transform.SetParent(transform, false);
		beam = beamGO.AddComponent<LineRenderer>();
		beam.material = glow;
		beam.startWidth = 0.05f;
		beam.endWidth = 0.02f;
		beam.positionCount = 2;
		beam.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
		beam.enabled = false;

		hitDot = GameObject.CreatePrimitive(PrimitiveType.Sphere);
		hitDot.name = "LaserHitDot";
		Object.Destroy(hitDot.GetComponent<Collider>());
		hitDot.transform.localScale = Vector3.one * 0.16f;
		hitDot.GetComponent<MeshRenderer>().material = glow;

		hitLight = hitDot.AddComponent<Light>();
		hitLight.type = LightType.Point;
		hitLight.color = laserColor;
		hitLight.range = 1.6f;
		hitLight.intensity = 2.2f;
		hitDot.SetActive(false);
	}

	void Update()
	{
		scanTimer -= Time.deltaTime;
		if (scanTimer <= 0f)
		{
			scanTimer = scanInterval;
			Scan();
		}

		bool showLaser = laserEnabled && currentTarget != null;
		if (beam != null) beam.enabled = showLaser;
		if (hitDot != null && hitDot.activeSelf != showLaser) hitDot.SetActive(showLaser);

		if (showLaser)
		{
			Vector3 targetPoint = currentTarget.transform.position + Vector3.up * 0.25f;
			beam.SetPosition(0, emitter.position);
			beam.SetPosition(1, targetPoint);
			hitDot.transform.position = targetPoint;

			//subtle pulse so the lock-on reads as active
			float pulse = 0.75f + 0.25f * Mathf.Sin(Time.time * 9f);
			beam.startWidth = 0.05f * pulse;
			hitLight.intensity = 2.2f * pulse;
		}
	}

	void Scan()
	{
		cropsInRange = 0;
		weedsInRange = 0;
		Plant nearestCrop = null, nearestWeed = null;
		float bestCrop = float.MaxValue, bestWeed = float.MaxValue;
		Vector3 pos = transform.position;

		for (int i = 0; i < Plant.All.Count; i++)
		{
			Plant p = Plant.All[i];
			if (p == null) continue;
			float d = Vector3.Distance(pos, p.transform.position);
			if (d > range) continue;

			if (p.desired)
			{
				cropsInRange++;
				if (d < bestCrop) { bestCrop = d; nearestCrop = p; }
			}
			else
			{
				weedsInRange++;
				if (d < bestWeed) { bestWeed = d; nearestWeed = p; }
			}
		}

		currentTarget = (preferWeeds && nearestWeed != null) ? nearestWeed : (nearestCrop != null ? nearestCrop : nearestWeed);

		if (currentTarget != null && currentTarget != lastLogged)
		{
			lastLogged = currentTarget;
			Debug.Log("[CropDetector] " + transform.root.name + " locked on " + currentTarget.name
				+ (currentTarget.desired ? " (crop)" : " (weed)")
				+ " | in range: " + cropsInRange + " crops, " + weedsInRange + " weeds");
		}
	}

	public void SetColor(Color c)
	{
		laserColor = c;
		if (beam != null) beam.material.SetColor("_TintColor", c);
		if (hitLight != null) hitLight.color = c;
	}
}
