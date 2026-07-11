using UnityEngine;
using UnityEngine.EventSystems;
using System.Collections.Generic;
using UnityEditor;

//Temporary diagnostic: logs every UI element under a point inside the chat panel,
//top-most first, so we can see what is stealing clicks from the chat input field.
public static class UIRaycastDebug
{
	[MenuItem("Tools/Debug UI Raycast At Chat")]
	public static void RaycastAtChat()
	{
		if (EventSystem.current == null) { Debug.Log("UIRaycastDebug: no EventSystem (enter Play mode first)."); return; }

		Vector2 point = new Vector2(Screen.width * 0.12f, Screen.height * 0.12f);
		PointerEventData ped = new PointerEventData(EventSystem.current) { position = point };
		List<RaycastResult> results = new List<RaycastResult>();
		EventSystem.current.RaycastAll(ped, results);

		Debug.Log("UIRaycastDebug: " + results.Count + " hits at " + point + " (screen " + Screen.width + "x" + Screen.height + ")");
		foreach (RaycastResult r in results)
		{
			Debug.Log("  hit: " + GetPath(r.gameObject) + "  module=" + r.module.GetType().Name + " sortingOrder=" + r.sortingOrder + " depth=" + r.depth);
		}
	}

	static string GetPath(GameObject go)
	{
		string p = go.name;
		Transform t = go.transform.parent;
		while (t != null) { p = t.name + "/" + p; t = t.parent; }
		return p;
	}

	[MenuItem("Tools/Debug Chat State")]
	public static void ChatState()
	{
		RobotChat chat = Object.FindObjectOfType<RobotChat>(true);
		if (chat == null) { Debug.Log("ChatState: no RobotChat in scene."); return; }

		Debug.Log("ChatState: chatGroup=" + (chat.chatGroup ? "alpha " + chat.chatGroup.alpha + " interactable " + chat.chatGroup.interactable + " blocksRaycasts " + chat.chatGroup.blocksRaycasts : "NULL"));
		if (chat.chatInputField != null)
		{
			Debug.Log("ChatState: input '" + GetPath(chat.chatInputField.gameObject) + "' interactable=" + chat.chatInputField.interactable + " isFocused=" + chat.chatInputField.isFocused + " text='" + chat.chatInputField.text + "'");
			foreach (Component c in chat.chatInputField.GetComponents<Component>()) Debug.Log("  input component: " + c.GetType().Name);
		}
		else Debug.Log("ChatState: chatInputField NULL");

		var es = UnityEngine.EventSystems.EventSystem.current;
		Debug.Log("ChatState: EventSystem selected=" + (es != null && es.currentSelectedGameObject != null ? GetPath(es.currentSelectedGameObject) : "none") + " module=" + (es != null && es.currentInputModule != null ? es.currentInputModule.GetType().Name : "none"));

		if (chat.chatInputField != null)
		{
			Debug.Log("ChatState: input IsInteractable() resolved=" + chat.chatInputField.IsInteractable());
			Transform t = chat.chatInputField.transform;
			while (t != null)
			{
				CanvasGroup g = t.GetComponent<CanvasGroup>();
				if (g != null) Debug.Log("  CanvasGroup on " + GetPath(t.gameObject) + ": interactable=" + g.interactable + " blocksRaycasts=" + g.blocksRaycasts + " alpha=" + g.alpha + " ignoreParent=" + g.ignoreParentGroups);
				t = t.parent;
			}
		}
	}

	[MenuItem("Tools/Force Focus Chat")]
	public static void ForceFocus()
	{
		RobotChat chat = Object.FindObjectOfType<RobotChat>(true);
		if (chat == null || chat.chatInputField == null) { Debug.Log("ForceFocus: missing chat/input."); return; }
		UnityEngine.EventSystems.EventSystem.current.SetSelectedGameObject(chat.chatInputField.gameObject);
		chat.chatInputField.ActivateInputField();
		Debug.Log("ForceFocus: done, isFocused=" + chat.chatInputField.isFocused);
	}

	[MenuItem("Tools/Debug Pointer Now")]
	public static void PointerNow()
	{
		Debug.Log("PointerNow: Cursor.lockState=" + Cursor.lockState + " visible=" + Cursor.visible + " Input.mousePosition=" + Input.mousePosition + " screen=" + Screen.width + "x" + Screen.height);

		var systems = Object.FindObjectsOfType<UnityEngine.EventSystems.EventSystem>(true);
		Debug.Log("PointerNow: EventSystems in scene: " + systems.Length);
		foreach (var s in systems) Debug.Log("  EventSystem: " + GetPath(s.gameObject) + " activeAndEnabled=" + s.isActiveAndEnabled + " isCurrent=" + (s == UnityEngine.EventSystems.EventSystem.current));

		var es = UnityEngine.EventSystems.EventSystem.current;
		if (es != null && es.currentInputModule != null)
		{
			var input = es.currentInputModule.input;
			Debug.Log("PointerNow: module=" + es.currentInputModule.GetType().Name + " inputClass=" + (input != null ? input.GetType().Name : "null") + " module.mousePosition=" + (input != null ? input.mousePosition.ToString() : "n/a") + " mousePresent=" + (input != null && input.mousePresent));

			if (input != null)
			{
				var ped = new UnityEngine.EventSystems.PointerEventData(es) { position = input.mousePosition };
				var results = new List<UnityEngine.EventSystems.RaycastResult>();
				es.RaycastAll(ped, results);
				Debug.Log("PointerNow: raycast at module mouse pos: " + results.Count + " hits" + (results.Count > 0 ? ", top: " + GetPath(results[0].gameObject) : ""));
			}
		}
		else Debug.Log("PointerNow: no current input module");
	}

	[MenuItem("Tools/Report Rover State")]
	public static void RoverState()
	{
		PathMaker pm = Object.FindObjectOfType<PathMaker>();
		if (pm == null || pm.rover == null) { Debug.Log("RoverState: no PathMaker/rover."); return; }

		DebugRover rover = pm.rover;
		Vector3 p = rover.transform.position;
		string spawnInfo = "no mapInfo/spawn";
		if (rover.mapInfo != null && rover.mapInfo.spawn != null) spawnInfo = "spawn at " + rover.mapInfo.spawn.position;

		bool grounded = Physics.Raycast(p + Vector3.up * 0.5f, Vector3.down, out RaycastHit hit, 5f, ~0, QueryTriggerInteraction.Ignore);
		Debug.Log("RoverState: pos=" + p + " vel=" + rover.GetComponent<Rigidbody>().velocity.magnitude.ToString("0.00")
			+ " groundBelow=" + (grounded ? hit.collider.name + " dist " + hit.distance.ToString("0.00") : "NONE within 5m")
			+ " | " + spawnInfo);
	}

	[MenuItem("Tools/Debug Terrain Details")]
	public static void TerrainDetails()
	{
		Terrain t = Object.FindObjectOfType<Terrain>();
		if (t == null || t.terrainData == null) { Debug.Log("TerrainDetails: no terrain in scene."); return; }
		TerrainData d = t.terrainData;
		int sum = 0;
		if (d.detailPrototypes.Length > 0)
		{
			int[,] layer = d.GetDetailLayer(0, 0, d.detailResolution, d.detailResolution, 0);
			foreach (int v in layer) sum += v;
		}
		Debug.Log("TerrainDetails: terrain=" + t.name + " prototypes=" + d.detailPrototypes.Length
			+ " detailRes=" + d.detailResolution + " layer0Sum=" + sum
			+ " drawFoliage=" + t.drawTreesAndFoliage + " detailDist=" + t.detailObjectDistance
			+ " density=" + t.detailObjectDensity + " qualityDensityScale=" + QualitySettings.terrainDetailDensityScale);
	}

	[MenuItem("Tools/Simulate Chat Submit")]
	public static void SimulateSubmit()
	{
		RobotChat chat = Object.FindObjectOfType<RobotChat>(true);
		if (chat == null || chat.chatInputField == null) { Debug.Log("SimulateSubmit: missing chat/input."); return; }
		chat.chatInputField.text = "start farming";
		chat.OnSubmit();
		Debug.Log("SimulateSubmit: submitted 'start farming'");
	}
}
