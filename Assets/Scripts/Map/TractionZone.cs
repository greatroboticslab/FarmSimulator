using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TractionZone : MonoBehaviour
{
	
	public int resolution; //Map is divided into resolution x resolution grid
	public MapInfo mapInfo;
	public float[,] tractionGrid;
	
	//  Test function to give the grid a checkerboard pattern of values 0 and 1
	public void Checkerboard() {
		int v = 0;
		for(int y = 0; y < resolution; y++) {
			for(int x = 0; x < resolution; x++) {
				tractionGrid[x,y] = v;
				v += 1;
				if(v > 1) {
					v = 0;
				}
			}
		}
	}
	
	//Get traction from world position
	public float GetTraction(Vector3 pos) {
		
		int x0 = (int)Mathf.FloorToInt((pos.x - transform.position.x) / (mapInfo.size / resolution));
		int y0 = (int)Mathf.FloorToInt((pos.z - (transform.position.z - mapInfo.size)) / (mapInfo.size / resolution));
		
		int x1 = x0 + 1;
        int y1 = y0 + 1;
		
        x0 = Mathf.Clamp(x0, 0, resolution - 1);
        x1 = Mathf.Clamp(x1, 0, resolution - 1);
        y0 = Mathf.Clamp(y0, 0, resolution - 1);
        y1 = Mathf.Clamp(y1, 0, resolution - 1);
		
		float tx = ((pos.x - transform.position.x) / (mapInfo.size / resolution)) - x0;
        float ty = ((pos.z - transform.position.z) / (mapInfo.size / resolution)) - y0;
		
		float v00 = tractionGrid[x0, y0];
        float v10 = tractionGrid[x1, y0];
        float v01 = tractionGrid[x0, y1];
        float v11 = tractionGrid[x1, y1];

        // Interpolate along x
        float a = Mathf.Lerp(v00, v10, tx);
        float b = Mathf.Lerp(v01, v11, tx);

		//Debug.Log("x: " + x0 + ", y: " + y0 + ", v: " + tractionGrid[x0,y0]);

        // Interpolate along y
        return Mathf.Lerp(a, b, ty);
		
	}
	
    // Start is called before the first frame update
    void Start()
    {
        tractionGrid = new float[resolution,resolution];
		Checkerboard();
		
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
