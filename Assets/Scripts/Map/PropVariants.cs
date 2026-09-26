using UnityEngine;

//Lets one prefab in a map's ObjectPlacer list stand for several models.
//
//tree1 and rock1 carry this component. The map prefabs keep pointing at them,
//so neither LFS-tracked map prefab has to change, and ObjectPlacer spawns one of
//the variants, with a random heading and size, instead of the same blockout a
//few thousand times.
[DisallowMultipleComponent]
public class PropVariants : MonoBehaviour
{
	public GameObject[] variants;
	//relative chance of each variant; missing or empty means equal chances
	public float[] weights;
	//uniform size range applied on top of the variant's own scale
	public Vector2 scaleRange = new Vector2(0.85f, 1.15f);
	public bool randomYaw = true;
	//rocks: tilt with the ground and sink in by this fraction of their size, so
	//they sit in the ground instead of balancing on it
	public bool alignToGround;
	[Range(0f, 0.5f)] public float sinkFraction;

	public GameObject Pick(System.Random rnd)
	{
		if (variants == null || variants.Length == 0) return null;
		float total = 0f;
		for (int i = 0; i < variants.Length; i++) total += Weight(i);
		if (total <= 0f) return variants[rnd.Next(variants.Length)];
		double r = rnd.NextDouble() * total;
		for (int i = 0; i < variants.Length; i++)
		{
			r -= Weight(i);
			if (r <= 0) return variants[i];
		}
		return variants[variants.Length - 1];
	}

	float Weight(int i)
	{
		if (variants[i] == null) return 0f;
		if (weights == null || i >= weights.Length) return 1f;
		return Mathf.Max(0f, weights[i]);
	}
}
