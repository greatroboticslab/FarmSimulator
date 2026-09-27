using System.Text.RegularExpressions;
using UnityEngine;
using UnityEngine.UI;
using TMPro;

//One line across the top of the farm HUD that says what the robot is doing, and
//why it is not doing anything when it is not. The HUD showed coordinates, power
//and basket weight, but nothing said the robot was picking fruit, had finished
//its rows, had found nothing to pick, or needed a plot first.
//
//It reads the robot ten times a second, like PosDisplay, so the robot scripts
//only call Notice() for events that leave no state behind (a plant with nothing
//to pick, a fruit given up on). Built at runtime from RoverControls.Start, like
//the Tutorial and Graphics buttons; no scene or prefab change.
public class RobotStatus : MonoBehaviour
{
	static RobotStatus instance;

	//the line as last shown, for logs and checks (SelfDriveCheck)
	public static string Current { get; private set; }

	RoverControls controls;
	GameObject panel;
	TMP_Text label;
	string notice;
	float noticeUntil;
	float nextRefresh;
	//waypoints in the plot being worked, for "stop k of n"
	int plotStops;
	bool finishedPlot;

	public static void Attach(RoverControls controls, Button template)
	{
		if (instance != null || template == null) return;
		Canvas canvas = template.GetComponentInParent<Canvas>();
		if (canvas == null) return;
		GameObject root = new GameObject("RobotStatus", typeof(RectTransform));
		root.transform.SetParent(canvas.rootCanvas.transform, false);
		instance = root.AddComponent<RobotStatus>();
		instance.controls = controls;
		instance.Build(root.GetComponent<RectTransform>());
	}

	//A message that stays up for a few seconds over the running description
	public static void Notice(string message, float seconds = 4f)
	{
		if (instance == null) return;
		instance.notice = message;
		instance.noticeUntil = Time.unscaledTime + seconds;
		instance.nextRefresh = 0f;
	}

	void OnDestroy()
	{
		if (instance == this) instance = null;
		Current = null;
	}

	void Build(RectTransform root)
	{
		//a strip at the top centre, clear of the robot view (top left) and the
		//button column (right); the root stays active so this keeps updating
		root.anchorMin = root.anchorMax = new Vector2(0.5f, 1f);
		root.pivot = new Vector2(0.5f, 1f);
		root.anchoredPosition = new Vector2(0f, -6f);
		root.sizeDelta = new Vector2(760f, 32f);

		panel = new GameObject("Panel", typeof(RectTransform));
		panel.transform.SetParent(root, false);
		Stretch(panel.GetComponent<RectTransform>(), 0f, 0f);
		Image bg = panel.AddComponent<Image>();
		bg.color = new Color(0f, 0f, 0f, 0.55f);
		bg.raycastTarget = false;

		GameObject textGO = new GameObject("Text", typeof(RectTransform));
		textGO.transform.SetParent(panel.transform, false);
		Stretch(textGO.GetComponent<RectTransform>(), 12f, 2f);
		TextMeshProUGUI t = textGO.AddComponent<TextMeshProUGUI>();
		t.fontSize = 19f;
		t.color = Color.white;
		t.alignment = TextAlignmentOptions.Center;
		t.enableWordWrapping = false;
		t.overflowMode = TextOverflowModes.Ellipsis;
		t.raycastTarget = false;
		label = t;
		panel.SetActive(false);
	}

	static void Stretch(RectTransform rt, float padX, float padY)
	{
		rt.anchorMin = Vector2.zero;
		rt.anchorMax = Vector2.one;
		rt.offsetMin = new Vector2(padX, padY);
		rt.offsetMax = new Vector2(-padX, -padY);
	}

	void Update()
	{
		if (Time.unscaledTime < nextRefresh) return;
		nextRefresh = Time.unscaledTime + 0.1f;

		string text = null;
		if (controls != null && controls.isActiveAndEnabled)
			text = Time.unscaledTime < noticeUntil ? notice : Describe();
		Current = text;
		bool show = !string.IsNullOrEmpty(text);
		if (panel.activeSelf != show) panel.SetActive(show);
		if (show && label.text != text) label.text = text;
	}

	string Describe()
	{
		PathMaker pm = PathMaker.Instance;
		if (pm == null) return null;
		int queued = pm.waypoints != null ? pm.waypoints.Count : 0;
		if (pm.rover != null) return DescribeRover(pm.rover, queued);
		HumanoidRobot h = pm.humanoidRobot;
		if (h != null && h.playerControlled) return DescribeHumanoid(h, queued);
		return null;
	}

	string DescribeRover(DebugRover r, int queued)
	{
		int left = queued + (r.currentWaypoint != null ? 1 : 0);
		string progress = Progress(left);

		if (r.stuck) return "Stuck. Try Flip Robot, or drive it free with W A S D";
		if (!r.selfDriving) return left > 0 ? "Plot ready: turn on Self Driving to work it" : null;

		RobotArm arm = r.robotArm;
		if (arm != null && arm.harvesting && arm.currentCrop != null)
			return "Picking " + CropName(arm.currentCrop) + ": " + ArmStage(arm) + FruitCount(arm.currentCrop, arm.currentFruit);
		if (left == 0) return Idle();

		PathMaker.Waypoint wp = r.currentWaypoint;
		string doing;
		if (wp == null) doing = "Heading to the first row";
		else if (wp.pickFruit) doing = "Driving to the next plant to pick";
		else if (wp.checkWater)
			doing = r.timeSinceMeasure < 3f ? "Soil moisture here: " + r.measuredWater.ToString("0") + "%" : "Driving to the next moisture check";
		else if (r.lightOn) doing = "Treating the row with UV light";
		//robotType 0 is the moisture robot: its path alternates a drive to each
		//plant with an aim-and-measure stop
		else if (r.robotType == 0) doing = r.timeSinceMeasure < 3f
			? "Soil moisture at the last plant: " + r.measuredWater.ToString("0") + "%"
			: "Driving to the next plant to check its soil";
		else doing = "Driving along the rows";
		return doing + progress;
	}

	string DescribeHumanoid(HumanoidRobot h, int queued)
	{
		int left = queued + (h.currentWaypoint != null ? 1 : 0);
		string progress = Progress(left);

		if (h.falling) return "Fallen over. Press C to set the robot back on its feet";
		if (!h.selfDriving) return left > 0 ? "Plot ready: turn on Self Driving to work it" : null;
		if (h.harvesting && h.currentCrop != null)
			return "Picking " + CropName(h.currentCrop) + FruitCount(h.currentCrop, h.currentFruit);
		if (left == 0) return Idle();
		return "Walking to the next plant" + progress;
	}

	string Idle()
	{
		return finishedPlot
			? "All rows done. Plant another plot with Farm Building, or drive by hand"
			: "Nothing to work yet: plant a plot with Farm Building first";
	}

	//"  (stop 12 of 40)"; the count is taken when a plot's waypoints appear
	string Progress(int left)
	{
		if (left == 0)
		{
			if (plotStops > 0) finishedPlot = true;
			plotStops = 0;
			return "";
		}
		if (left > plotStops)
		{
			plotStops = left;
			finishedPlot = false;
		}
		return "  (stop " + (plotStops - left + 1) + " of " + plotStops + ")";
	}

	static string FruitCount(Plant p, int index)
	{
		int n = p.fruits != null ? p.fruits.Count : 0;
		return n > 0 ? "  (fruit " + Mathf.Min(index + 1, n) + " of " + n + ")" : "";
	}

	static string ArmStage(RobotArm arm)
	{
		switch (arm.stage)
		{
			case 0: return "lining up";
			case 1: return "reaching";
			case 2: return arm.IsCutter ? "cutting" : "gripping";
			case 3: return "carrying it to the basket";
			default: return "dropping it in the basket";
		}
	}

	//"pepper_plant(Clone)" -> "Pepper"
	public static string CropName(Plant p)
	{
		if (p == null) return "Plant";
		string n = p.name.Replace("(Clone)", "").Replace('_', ' ').Trim();
		n = Regex.Replace(n, @"\s*\d+$", "");
		switch (n.ToLowerInvariant())
		{
			case "starberry unit":
			case "strawberry plant": return "Strawberry";
			case "pepper plant": return "Pepper";
			case "corn stalk": return "Corn";
		}
		return n.Length > 0 ? char.ToUpperInvariant(n[0]) + n.Substring(1) : "Plant";
	}
}
