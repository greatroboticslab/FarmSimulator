using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ItemBox {

	public string name;
	public Sprite icon;
	public GameObject obj;

}

public class CropSelection : MonoBehaviour
{

    
	public List<ItemBox> items;
	
	

    // Start is called before the first frame update
    void Start()
    {
        foreach (var item in items)
        {
            GameObject newButton = Instantiate(buttonPrefab, transform);
            newButton.GetComponentInChildren<TMP_Text>().text = item.name;
            newButton.transform.Find("Icon").GetComponent<Image>().sprite = item.icon;

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
        // Do something with the selection
        gameObject.SetActive(false); // Hide popup after selection
    }

}
