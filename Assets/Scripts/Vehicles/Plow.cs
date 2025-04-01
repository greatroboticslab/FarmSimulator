using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Plow : MonoBehaviour
{
	
	public GameObject dirtRowPrefab;
	public Transform contactsParent;
	public List<Transform> contactPoints;
	public Rigidbody rb;
	private float velAdd;
	
    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
		foreach(Transform c in contactsParent) {
			contactPoints.Add(c);
		}
    }

    // Update is called once per frame
    void Update()
    {
        if(rb.velocity.magnitude >= 0.01f) {
			velAdd += rb.velocity.magnitude;
			if(velAdd > 8.0f) {
				velAdd = 0;
				foreach(Transform c in contactPoints) {
					
					LayerMask layerMask = LayerMask.GetMask("Ground");

					RaycastHit hit;
					// Does the ray intersect any objects excluding the player layer
					if (Physics.Raycast(c.position, transform.TransformDirection(-Vector3.up), out hit, 2f, layerMask))

					{
						GameObject newDirt = Instantiate(dirtRowPrefab);
						newDirt.transform.position = hit.point;
						newDirt.transform.rotation = transform.rotation;
					}
					
				}
			}
		}
    }
}
