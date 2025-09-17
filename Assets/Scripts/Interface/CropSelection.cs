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

		foreach (var item in items)
		{
		    GameObject newButton = Instantiate(buttonPrefab, transform.GetChild(0));
		    newButton.GetComponentInChildren<TMP_Text>().text = item.name;
		    newButton.GetComponent<Image>().sprite = item.icon;

		    newButton.GetComponent<Button>().onClick.AddListener(() => OnItemSelected(item));
		}

	}

    // Update is called once per frame
    void Update()
    {
        
    }

    void OnItemSelected(ItemBox item)
    {
        Debug.Log("Selected: " + item.name);
        PathMaker.Instance.selectedCrop = item.obj;
        // Do something with the selection
        transform.parent.gameObject.SetActive(false); // Hide popup after selection
    }

}
