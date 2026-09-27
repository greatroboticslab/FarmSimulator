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
	public int stage; //0 = aligning, 1 = reaching, 2 = grabbing, 3 = moving to basket, 4 = dropping

	//set by DebugRover while the player drives: the player's joint inputs, written
	//every physics step, then stand until the rover writes them again instead of
	//being cleared at the start of every frame before they were applied
	[HideInInspector] public bool manualControl;

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
		//Nothing to pick: soybean has no fruit bodies, and a plant can already be
		//picked clean. This used to index the empty list after setting
		//harvesting, which threw before the rover took its next waypoint and
		//parked it at that plant for good.
		int first = NextFruit(p, 0);
		if(first < 0) {
			Debug.Log("[RobotArm] " + (p != null ? p.name : "no plant") + ": nothing to pick");
			RobotStatus.Notice(RobotStatus.CropName(p) + ": no fruit on this plant to pick, moving on");
			harvesting = false;
			return;
		}
		Debug.Log("Harvesting!");
		harvesting = true;
		currentCrop = p;
		currentFruit = first;
		BeginFruit();
	}

	public void SkipFruit() {
		Debug.Log("Time limit reached, skipping fruit! (stage " + stage + ", closest " + closest.ToString("0.000") + " m, needs " + minDist.ToString("0.000") + ")");
		RobotStatus.Notice("Could not reach that fruit in time: skipping it", 3f);
		NextFruitOrFinish();
	}

	//for the HUD's status line: snip or grip
	public bool IsCutter { get { return cutter; } }

	//How the arm reaches. The original finite-difference gradient descent, with
	//these gains, clamps nearly every step to a fixed speed and ended 4-36 cm
	//short of most fruit (SelfDriveCheck logs), so almost nothing was picked.
	//Cyclic coordinate descent (SolveCCD) instead; off brings the original back.
	public bool ccdReach = true;
	//fraction of each joint's maxDelta it may turn per second while reaching
	public float ccdSpeed = 0.35f;

	//Cyclic coordinate descent: from the joint nearest the tool back to the base,
	//each rotator turns about its own axis to swing the working spot toward the
	//target, at most ccdSpeed of its maxDelta per second so the arm still moves
	//like a machine, and a turn that would put the spot further from the target
	//is undone. The base slider slides along its axis within min..max.
	void SolveCCD(Transform spot, Vector3 target, float dt) {
		for(int i = joints.Count - 1; i >= 0; i--) {
			RobotJoint j = joints[i];
			if(j.obj == null) continue;
			Transform t = j.obj.transform;
			float step = j.maxDelta * ccdSpeed * dt;
			if(j.type == 0) {
				Vector3 axis = t.rotation * j.axis;
				if(axis.sqrMagnitude < 1e-8f) continue;
				axis.Normalize();
				Vector3 toSpot = Vector3.ProjectOnPlane(spot.position - t.position, axis);
				Vector3 toTarget = Vector3.ProjectOnPlane(target - t.position, axis);
				if(toSpot.sqrMagnitude < 1e-6f || toTarget.sqrMagnitude < 1e-6f) continue;
				float angle = Mathf.Clamp(Vector3.SignedAngle(toSpot, toTarget, axis), -step, step);
				if(Mathf.Abs(angle) < 1e-4f) continue;
				float before = (spot.position - target).sqrMagnitude;
				Quaternion was = t.rotation;
				t.rotation = Quaternion.AngleAxis(angle, axis) * was;
				if((spot.position - target).sqrMagnitude > before) t.rotation = was;
			}
			else if(j.type == 1) {
				//localPosition is in the parent's space, as in the original slider
				Vector3 perUnit = t.parent != null ? t.parent.TransformVector(j.axis) : j.axis;
				float len = perUnit.magnitude;
				if(len < 1e-6f) continue;
				float along = Vector3.Dot(target - spot.position, perUnit / len) / len;
				Vector3 lp = t.localPosition + j.axis * Mathf.Clamp(along, -step, step);
				lp.x = Mathf.Clamp(lp.x, j.min, j.max);
				t.localPosition = lp;
			}
		}
	}

	//nearest the working spot came to its target in the current stage, for the log
	float closest = float.MaxValue;
	int closestStage = -1;

	//Index of the first fruit still on the plant at or after 'from', or -1.
	//A picked fruit is destroyed in the basket and leaves a null in the list.
	static int NextFruit(Plant p, int from) {
		if(p == null || p.fruits == null) return -1;
		for(int i = Mathf.Max(0, from); i < p.fruits.Count; i++) {
			if(p.fruits[i] != null) return i;
		}
		return -1;
	}

	//The cutters have no align spot and no basket spot (they snip, the fruit
	//falls), and every stage that read one threw on them. Without an align
	//spot the cycle starts at the reach; without a basket spot it ends at the
	//cut (see Update).
	Transform AlignSpot { get { return preAlignSpot != null ? preAlignSpot : desiredSpot; } }

	//Start the align, reach, grab, basket, drop cycle on currentFruit
	void BeginFruit() {
		stage = preAlignSpot != null ? 0 : 1;
		timeHarvesting = 0;
		cutTime = 0;
		cutting = false;
		currentSpot = AlignSpot;
		currentTarget = currentCrop.fruits[currentFruit].transform;
		closest = float.MaxValue;
	}

	//On to the plant's next fruit, or done with the plant
	void NextFruitOrFinish() {
		int next = NextFruit(currentCrop, currentFruit + 1);
		if(next < 0) {
			harvesting = false;
			stage = 0;
			cutting = false;
			currentSpot = AlignSpot;
			return;
		}
		currentFruit = next;
		BeginFruit();
	}
	
    // Start is called before the first frame update
    void Start()
    {
        stage = 0;
        currentSpot = AlignSpot;

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

		//the target, not the fruit: once the fruit is in the basket it is gone,
		//and the arm still has to get there and open
		if(currentTarget != null) {

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
        if(harvesting || !manualControl) {
            for(int i = 0; i < joints.Count; i++) {
                joints[i].input = 0;
            }
        }

		//the plant itself can be removed mid-harvest
		if(harvesting && currentCrop == null) {
			harvesting = false;
			stage = 0;
			cutting = false;
			currentSpot = AlignSpot;
		}

		if(harvesting) {


			timeHarvesting += Time.deltaTime;

			//a second with the snipper closed on the stem: that one is cut
			if(cutTime >= 1.0f && cutter) {
				NextFruitOrFinish();
			}

			//the fruit went before the arm got hold of it
			if(harvesting && stage < 3 && currentCrop.fruits[currentFruit] == null) {
				NextFruitOrFinish();
			}
		}

		if(harvesting) {

            if(stage >= 3) {
                currentTarget = basketSpot;
            }

			//the original finite-difference gradient descent; with ccdReach the
			//joints are turned by SolveCCD below instead
			for(int i = 0; i < joints.Count && !ccdReach; i++) {
				if(joints[i].type < 2) {
					gradients[i] = GetGradient(i);
					float moveAmount = gradients[i] * lr;
					//NOTE: -moveAmount > -minSpeed is always true here, so every step
					//in the negative direction is exactly minSpeed, unlike the
					//positive branch. Mirroring the positive clamp was tried on the
					//Grabbing Robot and tomatoes: reach-stage closest approach median
					//13 cm against 22 cm as is, 0 fruit picked in 120 s either way.
					//Not a clear win, and lr, minSpeed and maxSpeed were tuned with
					//this in place, so it stays until someone retunes the arm.
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
			
			GameObject fruit = currentCrop.fruits[currentFruit];
			//Only before the grab: once in the gripper the fruit is nearer the
			//robot than the plant by definition, and this used to drop it mid-carry.
			//(The rover's position goes through Vector3 -> Vector2, which keeps x
			//and y, against x and z for the fruit and the plant. Left as it was: it
			//decides which fruits get picked.)
			if(fruit != null && stage < 3) {
			    currentTarget = fruit.transform;


				Vector2 fruitPos = new Vector2(fruit.transform.position.x, fruit.transform.position.z);
				Vector2 cropPos = new Vector2(currentCrop.transform.position.x, currentCrop.transform.position.z);

				float fruitDist = Vector2.Distance(transform.parent.transform.position, fruitPos);
				float cropDist = Vector2.Distance(transform.parent.transform.position, cropPos);

				if(cropDist > fruitDist) {
					NextFruitOrFinish();
				}
			}

				if(harvesting) {
					Vector3 targetPos = currentTarget.position;
					if(cutter) {
						targetPos += new Vector3(0f,0.03f,0f);
					}
					if(stage != closestStage) { closestStage = stage; closest = float.MaxValue; }
					if(currentSpot != null) closest = Mathf.Min(closest, Vector3.Distance(currentSpot.position, targetPos));
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
                        //with no align stage to open the tool, open it on the way in
                        if(preAlignSpot == null) {
                            freeze = false;
                            joints[joints.Count-1].input = -1.0f;
                        }

					    if(Vector3.Distance(desiredSpot.position, targetPos) <= minDist) {

                            cutting = true;
                            stage = 2;
                            currentSpot = desiredSpot;
                        }

					}
					// Grab
					if(stage == 2) {
                        //Debug.Log(joints[joints.Count-1].curVal);
                        joints[joints.Count-1].input = 0.2f;
                        freeze = true;

					    //a cutter without a basket spot holds the snip until the cut
					    //timer above moves it on
					    if(joints[joints.Count-1].curVal >= 0.97f && basketSpot != null) {
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
					    if(Vector3.Distance(desiredSpot.position, targetPos) <= minDist) {
                            //joints[joints.Count-1].input = -1.0f;
                            cutting = false;
                            stage = 4;
                            currentSpot = basketSpot;
                        }

					}

					//Drop: open over the basket, then the next fruit. Nothing handled
					//this stage, so the arm held each fruit until the rover's time
					//limit skipped it.
					if(stage == 4) {
                        freeze = true;
                        joints[joints.Count-1].input = -1.0f;
                        if(joints[joints.Count-1].curVal <= 0.02f) {
                            NextFruitOrFinish();
                        }
					}

					if(harvesting && ccdReach && !freeze && currentSpot != null) {
						SolveCCD(currentSpot, targetPos, Time.deltaTime);
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
