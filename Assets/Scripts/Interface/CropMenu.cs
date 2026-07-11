// This is the script for the whole panel that contains the submenus, each submenu has a CropSelection script

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

[System.Serializable]
public class CropCategory {
	
	public Button button;
	public GameObject menu;

}

public class CropMenu : MonoBehaviour
{

	public List<CropCategory> menus;

    // Start is called before the first frame update
    void Start()
    {

	//Procedurally make menus based on Resources folder
	
	

        foreach(CropCategory c in menus) {
		if(c == null || c.menu == null || c.button == null) continue;

		CropSelection m = c.menu.GetComponent<CropSelection>();
		if(m) {
			foreach(ItemBox i in m.items) {
				if(i.obj != null) {
					i.icon = GenerateSprite(i.obj);
				}
			}
			m.AddButtons();
		}

		c.button.onClick.AddListener(() => OnMenuSelected(c));
	}
    }

    // Update is called once per frame
    void Update()
    {
        
    }

	void OnMenuSelected(CropCategory c) {
		if(c == null || c.menu == null) return;
		for(int i = 0; i < menus.Count; i++) {
			if(menus[i] != null && menus[i].menu != null) menus[i].menu.SetActive(false);
		}
		c.menu.SetActive(true);
	}

	public Sprite GenerateSprite(GameObject prefab) {
		if(prefab == null) return null;

		RenderTexture rt = new RenderTexture(256, 256, 24);

		GameObject camGO = new GameObject("IconCamera");
		Camera cam = camGO.AddComponent<Camera>();
		//cam.backgroundColor = Color.clear;
		cam.clearFlags = CameraClearFlags.SolidColor;
		cam.backgroundColor = Color.white;

		cam.targetTexture = rt;

		GameObject temp = Instantiate(prefab, new Vector3(500,-500,500), Quaternion.identity);
		cam.transform.position = temp.transform.position + new Vector3(0, 2, -3);
		cam.transform.LookAt(temp.transform.position + new Vector3(0,1,0));

		cam.Render();
		RenderTexture.active = rt;

		Texture2D tex = new Texture2D(256, 256, TextureFormat.RGBA32, false);
		tex.ReadPixels(new Rect(0, 0, 256, 256), 0, 0);
		tex.Apply();

		if(temp != null) Object.DestroyImmediate(temp);
		cam.targetTexture = null;
		if(camGO != null) Object.DestroyImmediate(camGO);
		
		RenderTexture.active = null;
		if(rt != null) Destroy(rt);

		return Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));

	}
	

}


