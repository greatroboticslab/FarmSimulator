//This is the script that contains the buttons for crops, and their icons.

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

[System.Serializable]
public class ItemBox {

	public string name;
	public Sprite icon;
	public GameObject obj;

}

public class CropSelection : MonoBehaviour
{

    
	public List<ItemBox> items;
	public GameObject buttonPrefab;
	

    // Start is called before the first frame update
    void Start()
    {
	
    }

	public void AddButtons() {
		if(items == null || buttonPrefab == null || transform.childCount == 0) return;

		foreach (var item in items)
		{
			if(item == null) continue;
		    GameObject newButton = Instantiate(buttonPrefab, transform.GetChild(0));
		    TMP_Text label = newButton.GetComponentInChildren<TMP_Text>();
			if(label != null) label.text = item.name;
			Image image = newButton.GetComponent<Image>();
			if(image != null) image.sprite = item.icon;

			Button button = newButton.GetComponent<Button>();
			if(button != null) button.onClick.AddListener(() => OnItemSelected(item));
		}

	}

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnItemSelected(ItemBox item)
    {
		if(item == null || PathMaker.Instance == null) return;
        PathMaker.Instance.selectedCrop = item.obj;
        // Do something with the selection
        if(transform.parent != null) transform.parent.gameObject.SetActive(false); // Hide popup after selection
    }

}
