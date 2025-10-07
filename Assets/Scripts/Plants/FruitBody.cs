using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FruitBody : MonoBehaviour
{
	
	public float weight = 0.1f; //The weight of the fruit/vegetable in kilograms
	public Rigidbody rb;
	
	public bool readyToDrop;
	
    // Start is called before the first frame update
    void Start()
    {
        transform.localScale = new Vector3(1,1,1);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
	
	public void Detach() {
		rb.isKinematic = false;
	}
	
	void OnCollisionEnter(Collision collision)
    {
        //Detach();
    }
	
	void OnTriggerEnter(Collider collider)
    {
		if(collider.gameObject.tag == "basketZone" && !rb.isKinematic) {
			
			FollowerRover f = collider.gameObject.transform.parent.gameObject.GetComponent<FollowerRover>();
			if(f) {
				f.AddFruit(this);
			}
			else {
				Basket b = collider.gameObject.transform.parent.parent.gameObject.GetComponent<Basket>();
				if(b) {
					b.AddFruit(this);
				}
			}
			Destroy(gameObject);
		}
		if(collider.gameObject.tag == "dropZone") {
			readyToDrop = true;
		}
    }
}
