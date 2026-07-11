using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GrassSpawner : MonoBehaviour
{
	
	public GameObject grassPrefab;
	public Transform grassParent;
	public Mesh mesh;
	public List<GameObject> grassObjects;
	private bool started;
	public float renderDistance;

	//mesh.vertices copies the whole array on every access, so cache it once
	private Vector3[] vertices;
	//grass instance per vertex index; null entry means no grass spawned there
	private GameObject[] grassPerVertex;

	public void StartGrass() {
		MeshCollider meshCollider = GetComponent<MeshCollider>();
		if(meshCollider == null || meshCollider.sharedMesh == null || grassPrefab == null) {
			enabled = false;
			return;
		}

		started = true;
		mesh = meshCollider.sharedMesh;
		vertices = mesh.vertices;
		grassPerVertex = new GameObject[vertices.Length];
		grassObjects = new List<GameObject>();
		GameObject newGrassParent = new GameObject();
		newGrassParent.name = "Grass";
		newGrassParent.transform.parent = transform;
		grassParent = newGrassParent.transform;
	}

	public void UpdateGrass() {
		if(vertices == null || grassPerVertex == null || grassParent == null || Camera.main == null) return;

		Vector3 camPos = Camera.main.transform.position;
		float sqrRenderDistance = renderDistance * renderDistance;

		for(int i = 0; i < vertices.Length; i++) {

			//compare in world space; vertices are local to this mesh
			Vector3 wPos = transform.TransformPoint(vertices[i]);
			bool inRange = (wPos - camPos).sqrMagnitude <= sqrRenderDistance;
			GameObject existingGrass = grassPerVertex[i];

			if(inRange && existingGrass == null) {
				GameObject newGrass = Instantiate(grassPrefab, wPos, Quaternion.identity, grassParent);
				grassPerVertex[i] = newGrass;
				grassObjects.Add(newGrass);
			}
			else if(!inRange && existingGrass != null) {
				grassObjects.Remove(existingGrass);
				grassPerVertex[i] = null;
				Destroy(existingGrass);
			}
		}

	}
	
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(started) {
			UpdateGrass();
		}
    }
}
