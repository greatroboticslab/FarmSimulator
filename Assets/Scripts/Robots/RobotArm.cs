using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RobotArm : MonoBehaviour
{
	
	private bool cutter;
	private bool cutting;
	private float cutTime = 0f;
	public List<RobotJoint> joints;
	public Plant currentCrop;
	public int currentFruit;
	public List<float> gradients;
	public Transform desiredSpot;
	public Transform basketSpot;
	public Transform preAlignSpot; // Align with this spot first to avoid coming in from the side
	public float samplingDistance = 0.3f;
	public float lr = 7f;
	public float minDist = 0.03f;
	public float minSpeed = 0.25f;
	public float maxSpeed = 1.0f;
	public float timeHarvesting = 0f;
	public bool freeze;
	
	private Transform currentSpot;
	private Transform currentTarget; //Can switch from fruit to above the basket
	public int stage; //0 = aligning, 1 = grabbing, 2 = moving to basket, 3 = dropping
	
	public bool harvesting;
	
	[System.Serializable]
	public class RobotJoint {
		
		public string name;
		public Vector3 axis;
		public int type; //2 = Grabber joint, 3 = Cutter joint
		public float input;
		public float maxDelta;
		public float min;
		public float max;
		public float curVal = 0f;
		public GameObject obj;
		public Vector3 basePos;
		
	}
	
	[System.Serializable]
	public class ArmMotion {
		public List<Vector2> moves;
	}
	
	
	public void HarvestCrop(Plant p) {
		Debug.Log("Harvesting!");
		harvesting = true;
		currentCrop = p;
		currentFruit = 0;
		currentTarget = currentCrop.fruits[currentFruit].transform;
	}

	public void SkipFruit() {
	    stage = 0;
		currentFruit += 1;
		timeHarvesting = 0;
		Debug.Log("Time limit reached, skipping fruit!");
		if(currentFruit >= currentCrop.fruits.Count) {
			harvesting = false;
			
			//currentFruit = 0;
		}
		else {
		    currentSpot = preAlignSpot;
		    currentTarget = currentCrop.fruits[currentFruit].transform;
		}
	}
	
    // Start is called before the first frame update
    void Start()
    {
        stage = 0;
        currentSpot = preAlignSpot;

        gradients = new List<float>();
		for(int i = 0; i < joints.Count; i++) {
			gradients.Add(0);
			joints[i].basePos = joints[i].obj.transform.localPosition;
			if(joints[i].type == 3) {
				cutter = true;
			}
		}
    }
	
	//Idea: Get gradients and do gradient descent on all of the moveable joints to get the currentSpot as close to the fruit as possible.
	//A form of inverse kinematics?
	float GetGradient(int i) {


		float output = 0f;
		
		if(currentCrop.fruits[currentFruit] != null) {
		
			//Vector3 targetPos = currentCrop.fruits[currentFruit].transform.position;
			//Debug.Log(currentTarget);
			//Debug.Log("TRUE");


			Vector3 targetPos = currentTarget.position;
			//Debug.Log(Vector3.Distance(currentSpot.position, targetPos));
			//Debug.Log("" + currentSpot + ", " + currentTarget);
			if(cutter) {
				targetPos += new Vector3(0f,0.03f,0f);
			}
			
			float f_x = 0;
			float f_x_d = 0;
			
			if(joints[i].type == 0) {
			
				f_x = Vector3.Distance(currentSpot.position, targetPos);
				joints[i].obj.transform.Rotate(joints[i].axis * samplingDistance, Space.Self);
				f_x_d = Vector3.Distance(currentSpot.position, targetPos);
				joints[i].obj.transform.Rotate(joints[i].axis * -samplingDistance, Space.Self);
			}
			if(joints[i].type == 1) {
				f_x = Vector3.Distance(currentSpot.position, targetPos);
				joints[i].obj.transform.localPosition += (joints[i].axis * samplingDistance);
				f_x_d = Vector3.Distance(currentSpot.position, targetPos);
				joints[i].obj.transform.localPosition -= (joints[i].axis * samplingDistance);
			}
			output = (f_x - f_x_d) / samplingDistance;
		}
		
		return output;
	}

    // Update is called once per frame
    void Update()
    {
        for(int i = 0; i < joints.Count; i++) {
            joints[i].input = 0;
        }

		if(harvesting) {

            Debug.Log(stage);

			timeHarvesting += Time.deltaTime;
			
			if(cutTime >= 1.0f && cutter) {
				cutting = false;
				cutTime = 0;
				currentFruit += 1;
				if(currentFruit >= currentCrop.fruits.Count) {
					harvesting = false;
				}
				timeHarvesting = 0;
			}

            if(stage == 3) {
                currentTarget = basketSpot;
            }
			
			for(int i = 0; i < joints.Count; i++) {
				if(joints[i].type < 2) {
					gradients[i] = GetGradient(i);
					float moveAmount = gradients[i] * lr;
					if(moveAmount < 0) {
						if(-moveAmount > -minSpeed) {
							moveAmount = -minSpeed;
						}
						if(-moveAmount < -maxSpeed) {
							moveAmount = -maxSpeed;
						}
					}
					if(moveAmount > 0) {
						if(moveAmount < minSpeed) {
							moveAmount = minSpeed;
						}
						if(moveAmount > maxSpeed) {
							moveAmount = maxSpeed;
						}
					}
					joints[i].input = moveAmount;
				}
			}
			
			if(currentCrop.fruits[currentFruit] != null) {
			    currentTarget = currentCrop.fruits[currentFruit].transform;

				
				Vector2 fruitPos = new Vector2(currentCrop.fruits[currentFruit].transform.position.x,currentCrop.fruits[currentFruit].transform.position.z);
				Vector2 cropPos = new Vector2(currentCrop.transform.position.x, currentCrop.transform.position.z);
				
				float fruitDist = Vector2.Distance(transform.parent.transform.position, fruitPos);
				float cropDist = Vector2.Distance(transform.parent.transform.position, cropPos);
				
				if(cropDist > fruitDist) {
					currentFruit += 1;
					if(currentFruit >= currentCrop.fruits.Count) {
						harvesting = false;
					}
				}
				
				if(harvesting) {
					Vector3 targetPos = currentTarget.position;
					if(cutter) {
						targetPos += new Vector3(0f,0.03f,0f);
					}
                    //Debug.Log(Vector3.Distance(currentSpot.position, currentTarget.position));
                    //Debug.Log(stage);

                    // Align
                    if(stage == 0) {
                        freeze = false;
                        joints[joints.Count-1].input = -1.0f;
                        if(Vector3.Distance(preAlignSpot.position, targetPos) <= minDist) {
                            //cutting = true;
                            stage = 1;
                            currentSpot = desiredSpot;
                        }
					}
					// Reach
					if(stage == 1) {

					    if(Vector3.Distance(desiredSpot.position, targetPos) <= minDist) {

                            cutting = true;
                            stage = 2;
                            currentSpot = desiredSpot;
                        }

					}
					// Grab
					if(stage == 2) {
                        Debug.Log("GRABBING");
                        //Debug.Log(joints[joints.Count-1].curVal);
                        joints[joints.Count-1].input = 0.2f;
                        freeze = true;

					    if(joints[joints.Count-1].curVal >= 0.97f) {
                            //joints[joints.Count-1].input = -1.0f;
                            stage = 3;
                            currentTarget = basketSpot;
                            targetPos = currentTarget.position;
                        }

					}

					//Reach to Basket
					if(stage == 3) {
                        currentTarget = basketSpot;
                        targetPos = currentTarget.position;
                        freeze = false;
                        Debug.Log("" + desiredSpot + ", " + currentTarget + ", " + Vector3.Distance(desiredSpot.position, targetPos));
					    if(Vector3.Distance(desiredSpot.position, targetPos) <= minDist) {
                            //joints[joints.Count-1].input = -1.0f;
                            cutting = false;
                            stage = 4;
                            currentSpot = basketSpot;
                        }

					}

				}
			}
			if(cutting) {
				cutTime += Time.deltaTime;
			}
			
		}
		
        for(int i = 0; i < joints.Count; i++) {
			if(joints[i].input != 0) {
				
				//Rotator
				if(joints[i].type == 0) {
				    if(!freeze) {
					    joints[i].obj.transform.Rotate(joints[i].axis * joints[i].input * joints[i].maxDelta * Time.deltaTime, Space.Self);
					}
				}
				
				//Slider
				if(joints[i].type == 1) {
					if(!freeze) {
                        joints[i].obj.transform.localPosition += (joints[i].axis * joints[i].input * joints[i].maxDelta * Time.deltaTime);
                        if(joints[i].obj.transform.localPosition.x > (joints[i].max)) {
                            joints[i].obj.transform.localPosition = new Vector3(joints[i].max,joints[i].obj.transform.localPosition.y,joints[i].obj.transform.localPosition.z);
                        }
                        if(joints[i].obj.transform.localPosition.x < (joints[i].min)) {
                            joints[i].obj.transform.localPosition = new Vector3(joints[i].min,joints[i].obj.transform.localPosition.y,joints[i].obj.transform.localPosition.z);
                        }
					}
				}
				
				//Gripper
				if(joints[i].type == 2) {
					
					joints[i].curVal += joints[i].input * joints[i].maxDelta * Time.deltaTime;
					if(joints[i].curVal < 0) {
						joints[i].curVal = 0;
					}
					if(joints[i].curVal > 1) {
						joints[i].curVal = 1;
					}
					joints[i].obj.GetComponent<SkinnedMeshRenderer>().SetBlendShapeWeight(0, joints[i].curVal * 100f);;
					joints[i].obj.transform.GetComponent<Gripper>().val = joints[i].curVal;
					
				}
				
				//Cutter
				if(joints[i].type == 3) {
					
					joints[i].curVal += joints[i].input * joints[i].maxDelta * Time.deltaTime;
					if(joints[i].curVal < 0) {
						joints[i].curVal = 0;
					}
					if(joints[i].curVal > 1) {
						joints[i].curVal = 1;
					}
					joints[i].obj.GetComponent<SkinnedMeshRenderer>().SetBlendShapeWeight(0, joints[i].curVal * 100f);;
					joints[i].obj.transform.GetComponent<Snipper>().val = joints[i].curVal;
					
				}
				

			}
		}
    }
}
