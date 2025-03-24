using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Director : MonoBehaviour
{
	
	public List<HumanoidRobot> robots;
	public float mutationMultiplier = 0.01f;
	public float survivorFactor = 0.1f;
	public float mutantFactor = 0.1f; //A "mutant" gets their network changed by the supermutation multiplier rather than the mutation multiplier
	public float superMutationMultiplier = 0.5f;
	public float timeout = 5f;
	public float timeElapsed = 0f;
	public float positionMultiplier = 100.0f;
	public float animationMatchMultiplier = 0.01f;
	public float uprightMultiplier = 0.05f;
	public bool alwaysContinue;
	public List<float> scores;
	
	private float maxFitnessEver = -999999999f;
	private int timeSinceImprovement = 0;
	
	public List<float[]> lastNNs;
	public float lastScore = -9999f;
	public HumanoidRobot.ReferenceSkeleton refSkeleton;
	public Animator refSkeletonAnim;
	
	public float[] Mutate(float[] input, float amount) {
		float[] output = new float[input.Length];
		for(int i = 0; i < output.Length; i++) {
			output[i] = input[i] + Random.Range(-amount, amount);
		}
		return output;
	}
	
	public void ResetRobot(HumanoidRobot r) {
		r.transform.position = r.startPos;
		r.transform.rotation = r.startRot;
		r.curTime = 0;
		foreach(HumanoidRobot.HumanoidJoint j in r.joints) {
			j.rb.velocity = Vector3.zero;
			j.rb.angularVelocity = Vector3.zero;
			j.gameObject.transform.localPosition = j.startPos;
			j.gameObject.transform.localRotation = j.startRot;
		}
	}
	
	public void CopyPose(HumanoidRobot r) {
		
		for(int i = 0; i < r.joints.Length; i++) {
			
			r.joints[i].rb.isKinematic = true;
			
		}
		
		r.transform.position = r.startPos;
		
		if(r.refSkeleton.pelvis) {
				r.refSkeleton.pelvis.rotation =  refSkeleton.pelvis.rotation;
			}
			if(r.refSkeleton.lowerSpine) {
				r.refSkeleton.lowerSpine.rotation = refSkeleton.lowerSpine.rotation;
			}
			if(r.refSkeleton.midSpine) {
				r.refSkeleton.midSpine.rotation = refSkeleton.midSpine.rotation;
			}
			if(r.refSkeleton.upperSpine) {
				r.refSkeleton.upperSpine.rotation = refSkeleton.upperSpine.rotation;
			}
			if(r.refSkeleton.leftFemur) {
				r.refSkeleton.leftFemur.rotation = refSkeleton.leftFemur.rotation;
			}
			if(r.refSkeleton.leftTibia) {
				r.refSkeleton.leftTibia.rotation = refSkeleton.leftTibia.rotation;
			}
			if(r.refSkeleton.leftFoot) {
				r.refSkeleton.leftFoot.rotation = refSkeleton.leftFoot.rotation;
			}
			
			if(r.refSkeleton.rightFemur) {
				r.refSkeleton.rightFemur.rotation = refSkeleton.rightFemur.rotation;
			}
			if(r.refSkeleton.rightTibia) {
				r.refSkeleton.rightTibia.rotation = refSkeleton.rightTibia.rotation;
			}
			if(r.refSkeleton.rightFoot) {
				r.refSkeleton.rightFoot.rotation = refSkeleton.rightFoot.rotation;
			}
			
			if(r.refSkeleton.neck) {
				r.refSkeleton.neck.rotation = refSkeleton.neck.rotation;
			}
			if(r.refSkeleton.head) {
				r.refSkeleton.head.rotation = refSkeleton.head.rotation;
			}
			
			if(r.refSkeleton.leftShoulder) {
				r.refSkeleton.leftShoulder.rotation = refSkeleton.leftShoulder.rotation;
			}
			if(r.refSkeleton.leftBicep) {
				r.refSkeleton.leftBicep.rotation = refSkeleton.leftBicep.rotation;
			}
			if(r.refSkeleton.leftForearm) {
				r.refSkeleton.leftForearm.rotation = refSkeleton.leftForearm.rotation;
			}
			if(r.refSkeleton.leftHand) {
				r.refSkeleton.leftHand.rotation = refSkeleton.leftHand.rotation;
			}
			
			if(r.refSkeleton.rightShoulder) {
				r.refSkeleton.rightShoulder.rotation = refSkeleton.rightShoulder.rotation;
			}
			if(r.refSkeleton.rightBicep) {
				r.refSkeleton.rightBicep.rotation = refSkeleton.rightBicep.rotation;
			}
			if(r.refSkeleton.rightForearm) {
				r.refSkeleton.rightForearm.rotation = refSkeleton.rightForearm.rotation;
			}
			if(r.refSkeleton.rightHand) {
				r.refSkeleton.rightHand.rotation = refSkeleton.rightHand.rotation;
			}
		
	}
	
	public float FrameFitness(HumanoidRobot r, string action) {
		
		float fitness = 0;
		float uprightFitness = 0;
		
		if(action == "walk") {
			
			//uprightFitness -= Vector3.Angle(r.chest.transform.up, Vector3.up)*uprightMultiplier;
			uprightFitness += r.refSkeleton.head.position.y*uprightMultiplier;
			
			if(r.refSkeleton.pelvis) {
				fitness -= Vector3.Angle(r.refSkeleton.pelvis.up, refSkeleton.pelvis.up);
			}
			if(r.refSkeleton.lowerSpine) {
				fitness -= Vector3.Angle(r.refSkeleton.lowerSpine.up, refSkeleton.lowerSpine.up);
			}
			if(r.refSkeleton.midSpine) {
				fitness -= Vector3.Angle(r.refSkeleton.midSpine.up, refSkeleton.midSpine.up);
			}
			if(r.refSkeleton.upperSpine) {
				fitness -= Vector3.Angle(r.refSkeleton.upperSpine.up, refSkeleton.upperSpine.up);
			}
			if(r.refSkeleton.leftFemur) {
				fitness -= Vector3.Angle(r.refSkeleton.leftFemur.up, refSkeleton.leftFemur.up);
			}
			if(r.refSkeleton.leftTibia) {
				fitness -= Vector3.Angle(r.refSkeleton.leftTibia.up, refSkeleton.leftTibia.up);
			}
			if(r.refSkeleton.leftFoot) {
				fitness -= Vector3.Angle(r.refSkeleton.leftFoot.up, refSkeleton.leftFoot.up);
			}
			
			if(r.refSkeleton.rightFemur) {
				fitness -= Vector3.Angle(r.refSkeleton.rightFemur.up, refSkeleton.rightFemur.up);
			}
			if(r.refSkeleton.rightTibia) {
				fitness -= Vector3.Angle(r.refSkeleton.rightTibia.up, refSkeleton.rightTibia.up);
			}
			if(r.refSkeleton.rightFoot) {
				fitness -= Vector3.Angle(r.refSkeleton.rightFoot.up, refSkeleton.rightFoot.up);
			}
			
			if(r.refSkeleton.neck) {
				fitness -= Vector3.Angle(r.refSkeleton.neck.up, refSkeleton.neck.up);
			}
			if(r.refSkeleton.head) {
				fitness -= Vector3.Angle(r.refSkeleton.head.up, refSkeleton.head.up);
			}
			
			if(r.refSkeleton.leftShoulder) {
				fitness -= Vector3.Angle(r.refSkeleton.leftShoulder.up, refSkeleton.leftShoulder.up);
			}
			if(r.refSkeleton.leftBicep) {
				fitness -= Vector3.Angle(r.refSkeleton.leftBicep.up, refSkeleton.leftBicep.up);
			}
			if(r.refSkeleton.leftForearm) {
				fitness -= Vector3.Angle(r.refSkeleton.leftForearm.up, refSkeleton.leftForearm.up);
			}
			if(r.refSkeleton.leftHand) {
				fitness -= Vector3.Angle(r.refSkeleton.leftHand.up, refSkeleton.leftHand.up);
			}
			
			if(r.refSkeleton.rightShoulder) {
				fitness -= Vector3.Angle(r.refSkeleton.rightShoulder.up, refSkeleton.rightShoulder.up);
			}
			if(r.refSkeleton.rightBicep) {
				fitness -= Vector3.Angle(r.refSkeleton.rightBicep.up, refSkeleton.rightBicep.up);
			}
			if(r.refSkeleton.rightForearm) {
				fitness -= Vector3.Angle(r.refSkeleton.rightForearm.up, refSkeleton.rightForearm.up);
			}
			if(r.refSkeleton.rightHand) {
				fitness -= Vector3.Angle(r.refSkeleton.rightHand.up, refSkeleton.rightHand.up);
			}
			
		}
		return (fitness*animationMatchMultiplier) + uprightFitness;
		
	}
	
	public float GetFitness(HumanoidRobot r, string action) {
		
		float fitness = 0f;
		
		if(action == "walk") {
			fitness += (positionMultiplier*(r.transform.position.z - r.startPos.z)) - (Vector3.Angle(r.chest.transform.up, Vector3.up)*0) - (positionMultiplier*(Mathf.Abs(r.transform.position.x - r.startPos.x) + Mathf.Abs(r.transform.position.y  - r.startPos.y)));
			
		}
		return fitness;
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
		lastNNs = new List<float[]>();
		
		robots = new List<HumanoidRobot>();
        foreach(Transform child in transform) {
			robots.Add(child.gameObject.GetComponent<HumanoidRobot>());
		}
		NewEpoch();
    }
	
	public void NewEpoch() {
		
		timeElapsed = 0;
		scores = new List<float>();
		for(int i = 0; i < robots.Count; i++) {
				scores.Add(0);
		}
		
	}

    // Update is called once per frame
    void Update()
    {
		
		//CopyPose(robots[0]);
		//Debug.Log(FrameFitness(robots[0], "walk"));
		
		if(timeElapsed > timeout) {
			timeElapsed = 0;
			refSkeletonAnim.Play("Walk_N", 0, 0.25f);
		}
		
		/*
		if(timeElapsed > timeout) {
			
			
			float scoreSum = 0;
			float maxFitness = -99999999f;
			float maxId = 0;
			List<float[]> networks = new List<float[]>();
			for(int i = 0; i < robots.Count; i++) {
				//Debug.Log(GetFitness(robots[i], "walk"));
				
				networks.Add(robots[i].nn.SaveNetwork());
				float fitness = GetFitness(robots[i], "walk");
				scores[i] += (fitness);
				scoreSum += scores[i];
				if(scores[i] > maxFitness) {
					maxId = i;
					maxFitness = scores[i];
				}
				ResetRobot(robots[i]);
			}
			
			float meanFitness = scoreSum/scores.Count;
			
			
			
			
			float survivorScoreSum = 0f;
			
			//Do evolution
			int desiredSurvivors = (int)Mathf.Round(robots.Count * survivorFactor);
			List<int> survivorIds = new List<int>();
			while(desiredSurvivors > 0) {
				float max = -999999999f;
				int _maxId = 0;
				for(int i = 0; i < robots.Count; i++) {
					bool exists = false;
					foreach(int _i in survivorIds) {
						if(i == _i) {
							exists = true;
						}
					}
					if(!exists) {
						
						if(scores[i] > max) {
							max = scores[i];
							_maxId = i;
						}
						
					}
				}
				survivorScoreSum += scores[_maxId];
				survivorIds.Add(_maxId);
				desiredSurvivors -= 1;
			}
			
			float survivorFitness = survivorScoreSum/survivorIds.Count;
			
			
			bool improvement = false;
			int mutants = (int)Mathf.Round(robots.Count * mutantFactor);
			
			//Apply genetics
			for(int i = 0; i < robots.Count; i++) {
				
				float mutateAmount = mutationMultiplier;
				if(mutants > 0) {
					mutateAmount = superMutationMultiplier;
					mutants -= 1;
				}
				
				if(survivorFitness > lastScore) {
					improvement = true;
				}
				if(maxFitness > maxFitnessEver) {
					improvement = true;
				}
				if(improvement || alwaysContinue) {
					robots[i].nn.LoadNetwork(Mutate(robots[survivorIds[i%survivorIds.Count]].nn.SaveNetwork(),mutateAmount));
				}
				else {
					robots[i].nn.LoadNetwork(Mutate(lastNNs[i%lastNNs.Count],mutateAmount));
				}
				
			}
			
			if(improvement) {
				timeSinceImprovement = 0;
				lastNNs = new List<float[]>();
				lastScore = survivorFitness;
				foreach(int i in survivorIds) {
					lastNNs.Add(robots[i].nn.SaveNetwork());
				}
			}
			else {
				timeSinceImprovement += 1;
			}
			
			if(maxFitness > maxFitnessEver) {
				maxFitnessEver = maxFitness;
			}
			
			Debug.Log("Mean Fitness: " + (meanFitness) + " | Mean Survivor Fitness: " + survivorFitness + " | Max Fitness: " + maxFitness + " | Max Fitness Ever: " + maxFitnessEver + " | Time Since Improvement: " + timeSinceImprovement);
			
			NewEpoch();
			
		}
		else {
			for(int i = 0; i < robots.Count; i++) {
				scores[i] += FrameFitness(robots[i], "walk");
			}
		}
		*/
		
		timeElapsed += Time.deltaTime;
    }
}
