using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainCam : MonoBehaviour
{

	public CharacterController cc;

    public float sensitivity;
    public int mode;
    public bool chaseCam;
    
	private float maxChaseDist = 6f;

    public Vector3 targetPos;
    public Vector3 camPos;
    public GameObject focusedRobot;
	public RobotInfo robotSelection;
	public Vector3 currentFocusPoint;

	public Transform chaseCamOrigin;
	public Transform chaseCamOrbit;
	
	public Transform camTransform;
	public Transform leftHandTransform;
	
	public float walkSpeed = 5f;
	
	public bool isVRCam;
	
	public bool initialized;
	
	public float height = 1.75f;
    
    // Start is called before the first frame update
    void Start()
    {
		currentFocusPoint = new Vector3(0,0,0);
		
    	//Cursor.lockState = CursorLockMode.Locked;
        camPos = transform.position;
    }

    // Update is called once per frame
    void Update()
    {
		
		
		if(PathMaker.Instance != null) {
			if(isVRCam) {
				if(!PathMaker.Instance.VR) {
					gameObject.SetActive(false);
					return;
				}
			}
			else {
				if(PathMaker.Instance.VR) {
					gameObject.SetActive(false);
					return;
				}
			}
		}
		
		if(Input.GetKeyDown("p") && PathMaker.Instance != null && focusedRobot != null) {
			if(mode == 2) {
				mode = 1;
			}
			else if(PathMaker.Instance.placeCamOrg != null) {
				mode = 2;
				PathMaker.Instance.placeCamOrg.transform.position = focusedRobot.transform.position + new Vector3(0,15,0);
			}
		}
		
		
		//Chasecam
		if(mode == 1 && focusedRobot != null) {
			currentFocusPoint = Vector3.Lerp(currentFocusPoint, focusedRobot.transform.position, Time.deltaTime * 5.0f);
			transform.LookAt(currentFocusPoint);
		}
		
		if(mode == 2 && PathMaker.Instance != null && PathMaker.Instance.placeCamOrg != null) {
			
			if(!PathMaker.Instance.VR) {
				if(Input.GetKey("up")) {
					PathMaker.Instance.placeCamOrg.transform.position += new Vector3(0,0,5) * Time.deltaTime;
				}
				if(Input.GetKey("down")) {
					PathMaker.Instance.placeCamOrg.transform.position += new Vector3(0,0,-5) * Time.deltaTime;
				}
				
				if(Input.GetKey("left")) {
					PathMaker.Instance.placeCamOrg.transform.position += new Vector3(-5,0,0) * Time.deltaTime;
				}
				if(Input.GetKey("right")) {
					PathMaker.Instance.placeCamOrg.transform.position += new Vector3(5,0,0) * Time.deltaTime;
				}
				
				PathMaker.Instance.placeCamOrg.transform.position += Input.mouseScrollDelta.y * Time.deltaTime * transform.forward * 50f;
				
			}
			
			if(Input.GetKeyDown(KeyCode.Return) && PathMaker.Instance.manip != null) {
				
				PathMaker.Instance.manip.EndEditMode();
				
			}
		}
		
		//Select robot mode
		if(mode == 3) {
			if(Camera.main == null || PathMaker.Instance == null) return;
			Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit, 100))
            {
                
				//colliders often live on child parts; resolve up to the robot root
				RobotInfo ri = hit.collider.gameObject.GetComponentInParent<RobotInfo>();
				if(ri != null) {
					ri.hovering = true;
					
					if(Input.GetMouseButtonDown(0)) {
						PathMaker.Instance.selectedRobot = ri;
						PathMaker.Instance.humanoid = ri.humanoid;
						PathMaker.Instance.useTractor = ri.includeTractor;
						PathMaker.Instance.training = ri.training;
						PathMaker.Instance.useTerrain = ri.terrain;
						mode = 0;
						transform.position = Vector3.zero;
						if(PathMaker.Instance.coordMenu != null) PathMaker.Instance.coordMenu.SetActive(true);
					}
				}
            }
			transform.LookAt(currentFocusPoint);
		}
		
		if(!isVRCam) {
			transform.position = Vector3.Lerp(transform.position, camPos, 0.6f*Time.deltaTime);
		}
		
		Vector2 mv = OVRInput.Get(OVRInput.Axis2D.PrimaryThumbstick);
		
		
		Vector3 cForward = Vector3.zero;
		
		if(isVRCam && cc != null && cc.enabled && leftHandTransform != null) {
			cForward = leftHandTransform.transform.forward;
			cForward = new Vector3(cForward.x, 0, cForward.z);
			cForward.Normalize();
			
			Vector3 sideMv = Quaternion.Euler(0, 90, 0) * cForward;
			
			cc.Move(mv.y*cForward * Time.deltaTime * walkSpeed);
			cc.Move(mv.x*sideMv * Time.deltaTime * walkSpeed);
			cc.Move(-transform.up * Time.deltaTime);
		}
    	
        if(mode == 0) {
        	Vector3 lookRot = transform.localRotation.eulerAngles;
        	lookRot.y += Input.GetAxis("Mouse X") *Time.deltaTime * sensitivity;
        	lookRot.x -= Input.GetAxis("Mouse Y") *Time.deltaTime * sensitivity;
        	//transform.localRotation = Quaternion.Euler(lookRot);
        }
		else if(mode == 2) {
			if(PathMaker.Instance == null || PathMaker.Instance.placeCamOrg == null) return;
				targetPos = PathMaker.Instance.placeCamOrg.transform.position;
				
				transform.position = targetPos;
		}
        else {
        	if(chaseCam) {
        		if(focusedRobot != null) {
					
					if(chaseCamOrigin) {

                        float scroll = Input.GetAxis("Mouse ScrollWheel");
                        if(scroll > 0) {
                            chaseCamOrigin.transform.localScale *= 0.9f;
                        }
                        if(scroll < 0) {
                            chaseCamOrigin.transform.localScale *= 1.1f;
                        }

                        if(Input.GetMouseButton(1)) {
                            float _mx = Input.GetAxis("Mouse X");
                            float _my = Input.GetAxis("Mouse Y");
                            //chaseCamOrigin.transform.Rotate(-_my, 0f, 0f);
                            chaseCamOrigin.transform.Rotate(Vector3.up * _mx, Space.World);
                            chaseCamOrbit.transform.position += new Vector3(0,-_my*0.05f,0);
                        }

					    if(PathMaker.Instance.humanoid) {
                            chaseCamOrigin.transform.position = focusedRobot.transform.position;
                        }
                        else {
                            chaseCamOrigin.transform.position = focusedRobot.transform.position;
                        }

					    targetPos = chaseCamOrbit.transform.position;
					}
					else {

                        if(PathMaker.Instance.humanoid) {
                            targetPos = focusedRobot.transform.position + new Vector3(5,2,5);
                        }
                        else if(focusedRobot.GetComponent<DebugRover>() != null && focusedRobot.GetComponent<DebugRover>().camOrg != null) {
                            targetPos = focusedRobot.GetComponent<DebugRover>().camOrg.transform.position;
                        }
					}
        		}
				
				if(!isVRCam) {
					transform.position = targetPos;
				}
        	}
			
			if(focusedRobot != null) {
				Quaternion desiredRotation = focusedRobot.transform.rotation;
			}
			
        }
    }
}
