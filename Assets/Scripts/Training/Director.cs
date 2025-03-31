using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Director : MonoBehaviour
{
	
	public List<MLHumanoidController> robots;
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
	public GameObject actor;
	public HumanoidRobot.ReferenceSkeleton refSkeleton;
	public Animator refSkeletonAnim;
	
	public float[] Mutate(float[] input, float amount) {
		float[] output = new float[input.Length];
		for(int i = 0; i < output.Length; i++) {
			output[i] = input[i] + Random.Range(-amount, amount);
		}
		return output;
	}
	
	public void ResetRobot(MLHumanoidController r) {
		r.transform.position = r.startPos;
		r.transform.rotation = r.startRot;
		r.h.curTime = 0;
		foreach(HumanoidRobot.HumanoidJoint j in r.h.joints) {
			j.rb.velocity = Vector3.zero;
			j.rb.angularVelocity = Vector3.zero;
			j.gameObject.transform.localPosition = j.startPos;
			j.gameObject.transform.localRotation = j.startRot;
		}
	}
	
	public void CopyPose(MLHumanoidController r) {
		
		for(int i = 0; i < r.h.joints.Length; i++) {
			
			r.h.joints[i].rb.isKinematic = true;
			
		}
		
		r.transform.position = r.startPos;
		
		if(r.h.refSkeleton.pelvis) {
				r.h.refSkeleton.pelvis.rotation =  refSkeleton.pelvis.rotation;
			}
			if(r.h.refSkeleton.lowerSpine) {
				r.h.refSkeleton.lowerSpine.rotation = refSkeleton.lowerSpine.rotation;
			}
			if(r.h.refSkeleton.midSpine) {
				r.h.refSkeleton.midSpine.rotation = refSkeleton.midSpine.rotation;
			}
			if(r.h.refSkeleton.upperSpine) {
				r.h.refSkeleton.upperSpine.rotation = refSkeleton.upperSpine.rotation;
			}
			if(r.h.refSkeleton.leftFemur) {
				r.h.refSkeleton.leftFemur.rotation = refSkeleton.leftFemur.rotation;
			}
			if(r.h.refSkeleton.leftTibia) {
				r.h.refSkeleton.leftTibia.rotation = refSkeleton.leftTibia.rotation;
			}
			if(r.h.refSkeleton.leftFoot) {
				r.h.refSkeleton.leftFoot.rotation = refSkeleton.leftFoot.rotation;
			}
			
			if(r.h.refSkeleton.rightFemur) {
				r.h.refSkeleton.rightFemur.rotation = refSkeleton.rightFemur.rotation;
			}
			if(r.h.refSkeleton.rightTibia) {
				r.h.refSkeleton.rightTibia.rotation = refSkeleton.rightTibia.rotation;
			}
			if(r.h.refSkeleton.rightFoot) {
				r.h.refSkeleton.rightFoot.rotation = refSkeleton.rightFoot.rotation;
			}
			
			if(r.h.refSkeleton.neck) {
				r.h.refSkeleton.neck.rotation = refSkeleton.neck.rotation;
			}
			if(r.h.refSkeleton.head) {
				r.h.refSkeleton.head.rotation = refSkeleton.head.rotation;
			}
			
			if(r.h.refSkeleton.leftShoulder) {
				r.h.refSkeleton.leftShoulder.rotation = refSkeleton.leftShoulder.rotation;
			}
			if(r.h.refSkeleton.leftBicep) {
				r.h.refSkeleton.leftBicep.rotation = refSkeleton.leftBicep.rotation;
			}
			if(r.h.refSkeleton.leftForearm) {
				r.h.refSkeleton.leftForearm.rotation = refSkeleton.leftForearm.rotation;
			}
			if(r.h.refSkeleton.leftHand) {
				r.h.refSkeleton.leftHand.rotation = refSkeleton.leftHand.rotation;
			}
			
			if(r.h.refSkeleton.rightShoulder) {
				r.h.refSkeleton.rightShoulder.rotation = refSkeleton.rightShoulder.rotation;
			}
			if(r.h.refSkeleton.rightBicep) {
				r.h.refSkeleton.rightBicep.rotation = refSkeleton.rightBicep.rotation;
			}
			if(r.h.refSkeleton.rightForearm) {
				r.h.refSkeleton.rightForearm.rotation = refSkeleton.rightForearm.rotation;
			}
			if(r.h.refSkeleton.rightHand) {
				r.h.refSkeleton.rightHand.rotation = refSkeleton.rightHand.rotation;
			}
		
	}
	
	public float FrameFitness(MLHumanoidController r, string action) {
		
		float fitness = 0;
		float uprightFitness = 0;
		
		if(action == "walk") {
			
			//uprightFitness -= Vector3.Angle(r.chest.transform.up, Vector3.up)*uprightMultiplier;
			uprightFitness += r.h.refSkeleton.head.position.y*uprightMultiplier;
			
			if(r.h.refSkeleton.pelvis) {
				fitness -= Vector3.Angle(r.h.refSkeleton.pelvis.up, refSkeleton.pelvis.up);
			}
			if(r.h.refSkeleton.lowerSpine) {
				fitness -= Vector3.Angle(r.h.refSkeleton.lowerSpine.up, refSkeleton.lowerSpine.up);
			}
			if(r.h.refSkeleton.midSpine) {
				fitness -= Vector3.Angle(r.h.refSkeleton.midSpine.up, refSkeleton.midSpine.up);
			}
			if(r.h.refSkeleton.upperSpine) {
				fitness -= Vector3.Angle(r.h.refSkeleton.upperSpine.up, refSkeleton.upperSpine.up);
			}
			if(r.h.refSkeleton.leftFemur) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftFemur.up, refSkeleton.leftFemur.up);
			}
			if(r.h.refSkeleton.leftTibia) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftTibia.up, refSkeleton.leftTibia.up);
			}
			if(r.h.refSkeleton.leftFoot) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftFoot.up, refSkeleton.leftFoot.up);
			}
			
			if(r.h.refSkeleton.rightFemur) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightFemur.up, refSkeleton.rightFemur.up);
			}
			if(r.h.refSkeleton.rightTibia) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightTibia.up, refSkeleton.rightTibia.up);
			}
			if(r.h.refSkeleton.rightFoot) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightFoot.up, refSkeleton.rightFoot.up);
			}
			
			if(r.h.refSkeleton.neck) {
				fitness -= Vector3.Angle(r.h.refSkeleton.neck.up, refSkeleton.neck.up);
			}
			if(r.h.refSkeleton.head) {
				fitness -= Vector3.Angle(r.h.refSkeleton.head.up, refSkeleton.head.up);
			}
			
			if(r.h.refSkeleton.leftShoulder) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftShoulder.up, refSkeleton.leftShoulder.up);
			}
			if(r.h.refSkeleton.leftBicep) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftBicep.up, refSkeleton.leftBicep.up);
			}
			if(r.h.refSkeleton.leftForearm) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftForearm.up, refSkeleton.leftForearm.up);
			}
			if(r.h.refSkeleton.leftHand) {
				fitness -= Vector3.Angle(r.h.refSkeleton.leftHand.up, refSkeleton.leftHand.up);
			}
			
			if(r.h.refSkeleton.rightShoulder) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightShoulder.up, refSkeleton.rightShoulder.up);
			}
			if(r.h.refSkeleton.rightBicep) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightBicep.up, refSkeleton.rightBicep.up);
			}
			if(r.h.refSkeleton.rightForearm) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightForearm.up, refSkeleton.rightForearm.up);
			}
			if(r.h.refSkeleton.rightHand) {
				fitness -= Vector3.Angle(r.h.refSkeleton.rightHand.up, refSkeleton.rightHand.up);
			}
			
		}
		return (fitness*animationMatchMultiplier) + uprightFitness;
		
	}
	
	public float GetFitness(MLHumanoidController r, string action) {
		
		float fitness = 0f;
		
		if(action == "walk") {
			fitness += (positionMultiplier*(r.transform.position.z - r.startPos.z)) - (Vector3.Angle(r.h.chest.transform.up, Vector3.up)*0) - (positionMultiplier*(Mathf.Abs(r.transform.position.x - r.startPos.x) + Mathf.Abs(r.transform.position.y  - r.startPos.y)));
			
		}
		return fitness;
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
		lastNNs = new List<float[]>();
		
		robots = new List<MLHumanoidController>();
        foreach(Transform child in transform) {
			robots.Add(child.gameObject.GetComponent<MLHumanoidController>());
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
