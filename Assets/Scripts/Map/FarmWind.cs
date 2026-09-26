using UnityEngine;

//Drives the global wind the vegetation shaders read (_FarmWind, see
//Assets/Shaders/FarmWind.cginc). A light breeze with slow gusts, blowing from the
//south-west, the prevailing direction in Middle Tennessee.
//
//Created on its own before the first scene loads, so no scene or prefab has to
//carry it. Materials opt in with _WindWeight; the crop materials leave it at 0.
public class FarmWind : MonoBehaviour
{
	//metres of lean at a leaf cluster's tip in a steady breeze
	public static float strength = 0.14f;
	//how much gusts add on top, as a fraction of strength
	public static float gustiness = 0.8f;
	public static float speed = 1f;
	//the direction the wind blows toward (from the south-west: toward north-east)
	public static Vector3 direction = new Vector3(0.7f, 0f, 0.7f);

	static readonly int WindId = Shader.PropertyToID("_FarmWind");

	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
	static void Create()
	{
		GameObject go = new GameObject("FarmWind");
		DontDestroyOnLoad(go);
		go.AddComponent<FarmWind>();
	}

	void Update()
	{
		float t = Time.time;
		//two slow noise octaves: gusts build over a few seconds and die away
		float gust = Mathf.PerlinNoise(t * 0.11f, 0.37f) * 0.7f + Mathf.PerlinNoise(t * 0.43f, 5.1f) * 0.3f;
		float s = strength * (1f + gustiness * (gust - 0.35f));
		Vector3 d = direction.sqrMagnitude > 1e-6f ? direction.normalized : Vector3.forward;
		//gusts also swing the direction a little
		float swing = (Mathf.PerlinNoise(t * 0.07f, 9.3f) - 0.5f) * 0.5f;
		d = Quaternion.Euler(0f, swing * Mathf.Rad2Deg, 0f) * d;
		Shader.SetGlobalVector(WindId, new Vector4(d.x * s, 0f, d.z * s, speed));
	}

	void OnDestroy()
	{
		Shader.SetGlobalVector(WindId, Vector4.zero);
	}
}
