using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Actor : MonoBehaviour
{
	
	public Animator anim;
	public HumanoidRobot.ReferenceSkeleton refSkeleton;
	
	private AnimatorOverrideController overrideController;
	public AnimatorStateInfo animInfo;

	public void OnFootstep() {
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
        overrideController = new AnimatorOverrideController(anim.runtimeAnimatorController);
        anim.runtimeAnimatorController = overrideController;
	animInfo = anim.GetCurrentAnimatorStateInfo(0);
    }

    // Update is called once per frame
    void Update()
    {
	
    }

	void ReplaceClip(string stateName, AnimationClip newClip)
    {
        // Get all overrides from the controller
        var overrides = new List<KeyValuePair<AnimationClip, AnimationClip>>();
        overrideController.GetOverrides(overrides);

        // Loop through and replace the one matching the state clip
        for (int i = 0; i < overrides.Count; i++)
        {
            var originalClip = overrides[i].Key;
            if (originalClip.name == stateName)
            {
                overrides[i] = new KeyValuePair<AnimationClip, AnimationClip>(originalClip, newClip);
                break;
            }
        }

        // Apply updated overrides
        overrideController.ApplyOverrides(overrides);
    }

	public void PlayClip(AnimationClip newClip)
    {
        // Assume "Idle" is a state name that exists in your base controller
        // Replace the motion in that state
        var overrides = new List<KeyValuePair<AnimationClip, AnimationClip>>();
        overrideController.GetOverrides(overrides);

        // Replace the first entry (you can match by name if needed)
        if (overrides.Count > 0)
        {
            overrides[0] = new KeyValuePair<AnimationClip, AnimationClip>(overrides[0].Key, newClip);
            overrideController.ApplyOverrides(overrides);
        }

        // Play the state that the override is bound to
        anim.Play("customanimation", 0, 0f);
    }

}
