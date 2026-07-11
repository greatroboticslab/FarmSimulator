using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class AnimationButton : MonoBehaviour
{

	public AnimationLoader.AnimationEntry animation;
	public int animId = -1; //If not -1, then remove the animation from the queue based on its animId

    // Start is called before the first frame update
    void Start()
    {
        Button button = GetComponent<Button>();
		if(button != null) button.onClick.AddListener(ButtonClicked);
    }

    // Update is called once per frame
    void Update()
    {
        
    }

	public void ButtonClicked() {

		if(PathMaker.Instance == null || PathMaker.Instance.animationLoader == null) return;

		if(animId >= 0) {
			PathMaker.Instance.animationLoader.RemoveTask(animId);
			PathMaker.Instance.animationLoader.RefreshCurrentTasks();
			Destroy(gameObject);
		}
		else {

			PathMaker.Instance.animationLoader.AddTask(animation);
			PathMaker.Instance.animationLoader.RefreshCurrentTasks();

		}

	}

}
