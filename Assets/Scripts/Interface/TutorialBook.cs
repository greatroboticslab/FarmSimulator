using UnityEngine;
using UnityEngine.UI;
using TMPro;

//In-game manual. Pops up the first time a farm loads each session, can be
//flipped through page by page or closed instantly, and reopens from a
//Tutorial button cloned into the right-side HUD.
public class TutorialBook : MonoBehaviour
{
	static TutorialBook instance;
	static bool shownThisSession;

	struct Page { public string title; public string body; public Page(string t, string b) { title = t; body = b; } }

	static readonly Page[] Pages =
	{
		new Page("Welcome to Farm Simulator",
			"This simulator lets you drive and automate farm robots on real farm terrain.\n\n" +
			"The basic loop:\n" +
			"1. Pick a robot on the select screen\n" +
			"2. Pick a farm and press Go to Farm\n" +
			"3. Build a crop plot\n" +
			"4. Work it yourself, or let the robot do it\n\n" +
			"Flip through this manual with Next, or close it and come back anytime with the Tutorial button on the right."),

		new Page("Driving",
			"W / S - drive forward and backward\n" +
			"A / D - steer left and right\n\n" +
			"The camera follows your robot automatically.\n\n" +
			"Toggle View (top left button) switches camera angles. " +
			"If you flip your robot over, use the Flip Robot button on the right panel to right it.\n\n" +
			"Watch the Power readout at the bottom right; driving and lights drain the battery."),

		new Page("Building a Farm Plot",
			"Press Farm Building on the right panel. The camera rises to an overhead view.\n\n" +
			"Click the field for a small plot, or click and drag to stake out a larger one. " +
			"When you release, crop rows are planted, weeds sprout, and path waypoints are " +
			"laid for the robots.\n\n" +
			"The sliders on the right control row density, column density, and weed amount. " +
			"The Farming Components dropdown picks what to plant.\n\n" +
			"Press Farm Building again to return to driving."),

		new Page("Robot Chat",
			"The white box at the bottom left is a chat with your robot, powered by an LLM.\n\n" +
			"Type plain English like:\n" +
			"  \"start farming\"\n" +
			"  \"stop\"\n\n" +
			"The robot understands and acts: start farming engages self-driving along the crop rows.\n\n" +
			"(Chat needs a free Groq API key in StreamingAssets/config.json; see SETUP_AND_RUN.md in the project.)"),

		new Page("Self-Driving and Lasers",
			"Tick Self Driving (top right) or tell the chat to start farming.\n\n" +
			"The robot drives the planted rows waypoint by waypoint and works as it goes.\n\n" +
			"Every rover carries a crop detector: it scans nearby plants, tells crops from weeds, " +
			"and locks a visible laser onto its current target. Red beam means a normal rover; " +
			"the UV robot lases violet."),

		new Page("The Right-Side Panel",
			"Self Driving - the robot works the planted rows on its own\n" +
			"Science QA Format - switches robot data reporting to the lab's science Q&A format\n" +
			"Farm Building - overhead view; drag to stake crop plots\n" +
			"Start Recording - saves camera frames to the Recordings folder for building datasets\n" +
			"Configure Robot - tune speeds, distances, and friction for the current robot\n" +
			"Flip Robot - rights your robot if it rolls over\n" +
			"Edit Traction - paint slippery or muddy zones on the field to test rough terrain\n" +
			"Main Menu (Exit) - back to the robot select screen\n" +
			"Tutorial - reopens this manual"),

		new Page("Rovers: Cutting and UV",
			"Cutting Robot and Cutting Robot 2 hunt weeds. Their detectors prefer weeds " +
			"over crops, so their laser shows you exactly which weed is next.\n\n" +
			"The UV Robot and Laser Cart carry a sterilizing ultraviolet light. Drive close to " +
			"moldy crops with the light on and the mold burns off. The violet glow under the " +
			"cart shows the treatment area."),

		new Page("Rovers: Moisture and Grabbing",
			"The Moisture Sensing Robot samples soil water at each crop. Its readings show " +
			"in the Moisture display at the bottom right; in self-driving it measures every " +
			"plant on the row automatically.\n\n" +
			"The Grabbing Robot has an articulated arm and a basket. It targets ripe fruit " +
			"and harvests into the basket; the Basket Load readout tracks the weight collected."),

		new Page("Humanoids",
			"HumanoidR + Rover: a walking robot with a basket rover that follows and parks near you.\n\n" +
			"Humanoid + Tractor: the same robot paired with a drivable tractor.\n\n" +
			"Humanoid General Tasks: task playback; queue up animations from the Kinematic menu.\n\n" +
			"Humanoid Training is for ML-Agents machine learning runs and will ragdoll unless " +
			"a training process is attached; it is a research mode, not a gameplay one."),
	};

	GameObject panel;
	TMP_Text titleText;
	TMP_Text bodyText;
	TMP_Text pageText;
	int page;

	//Called after the farm loads; builds UI on first use
	public static void ShowFirstTime()
	{
		if (shownThisSession) return;
		shownThisSession = true;
		Show();
	}

	public static void Show()
	{
		if (instance == null) instance = Create();
		if (instance == null) return;
		instance.page = 0;
		instance.Refresh();
		instance.panel.SetActive(true);
	}

	//Clones an existing HUD button so the Tutorial button matches its siblings
	public static void AttachHudButton(Button template)
	{
		if (template == null || template.transform.parent == null) return;
		if (template.transform.parent.Find("TutorialButton") != null) return;

		Button b = Object.Instantiate(template, template.transform.parent);
		b.name = "TutorialButton";
		b.onClick.RemoveAllListeners();
		b.onClick.AddListener(Show);

		TMP_Text label = b.GetComponentInChildren<TMP_Text>();
		if (label != null) label.text = "Tutorial";
		else
		{
			Text legacy = b.GetComponentInChildren<Text>();
			if (legacy != null) legacy.text = "Tutorial";
		}

		RectTransform rt = b.GetComponent<RectTransform>();
		RectTransform src = template.GetComponent<RectTransform>();
		rt.anchoredPosition = src.anchoredPosition + new Vector2(0f, -(src.sizeDelta.y + 4f));
	}

	static TutorialBook Create()
	{
		Canvas canvas = null;
		foreach (Canvas c in Object.FindObjectsOfType<Canvas>())
		{
			if (c.isRootCanvas) { canvas = c; break; }
		}
		if (canvas == null) return null;

		GameObject root = new GameObject("TutorialBook", typeof(RectTransform));
		root.transform.SetParent(canvas.transform, false);
		TutorialBook book = root.AddComponent<TutorialBook>();
		book.Build(root.GetComponent<RectTransform>());
		return book;
	}

	void Build(RectTransform root)
	{
		root.anchorMin = Vector2.zero;
		root.anchorMax = Vector2.one;
		root.offsetMin = Vector2.zero;
		root.offsetMax = Vector2.zero;

		//dim backdrop
		Image backdrop = root.gameObject.AddComponent<Image>();
		backdrop.color = new Color(0f, 0f, 0f, 0.45f);

		//book panel
		panel = root.gameObject;
		GameObject bookGO = MakeRect("Book", root, new Vector2(720f, 460f), Vector2.zero);
		Image bookBg = bookGO.AddComponent<Image>();
		bookBg.color = new Color(0.96f, 0.94f, 0.86f); //paper

		//title
		titleText = MakeText(bookGO.transform, "Title", new Vector2(0f, 185f), new Vector2(640f, 50f), 26f, FontStyles.Bold);
		titleText.alignment = TextAlignmentOptions.Center;

		//body
		bodyText = MakeText(bookGO.transform, "Body", new Vector2(0f, -15f), new Vector2(620f, 330f), 17f, FontStyles.Normal);
		bodyText.alignment = TextAlignmentOptions.TopLeft;

		//page counter
		pageText = MakeText(bookGO.transform, "PageNum", new Vector2(0f, -207f), new Vector2(200f, 30f), 14f, FontStyles.Italic);
		pageText.alignment = TextAlignmentOptions.Center;

		//nav buttons
		MakeButton(bookGO.transform, "Prev", "< Prev", new Vector2(-270f, -207f), new Vector2(110f, 34f), () => Turn(-1));
		MakeButton(bookGO.transform, "Next", "Next >", new Vector2(270f, -207f), new Vector2(110f, 34f), () => Turn(1));

		//close X
		MakeButton(bookGO.transform, "Close", "X", new Vector2(338f, 208f), new Vector2(34f, 34f), () => panel.SetActive(false));

		panel.SetActive(false);
	}

	void Turn(int dir)
	{
		page = Mathf.Clamp(page + dir, 0, Pages.Length - 1);
		Refresh();
	}

	void Refresh()
	{
		titleText.text = Pages[page].title;
		bodyText.text = Pages[page].body;
		pageText.text = "Page " + (page + 1) + " / " + Pages.Length;
	}

	static GameObject MakeRect(string name, Transform parent, Vector2 size, Vector2 pos)
	{
		GameObject go = new GameObject(name, typeof(RectTransform));
		go.transform.SetParent(parent, false);
		RectTransform rt = go.GetComponent<RectTransform>();
		rt.sizeDelta = size;
		rt.anchoredPosition = pos;
		return go;
	}

	static TMP_Text MakeText(Transform parent, string name, Vector2 pos, Vector2 size, float fontSize, FontStyles style)
	{
		GameObject go = MakeRect(name, parent, size, pos);
		TextMeshProUGUI t = go.AddComponent<TextMeshProUGUI>();
		t.fontSize = fontSize;
		t.fontStyle = style;
		t.color = new Color(0.15f, 0.12f, 0.08f);
		t.raycastTarget = false;
		return t;
	}

	void MakeButton(Transform parent, string name, string label, Vector2 pos, Vector2 size, UnityEngine.Events.UnityAction onClick)
	{
		GameObject go = MakeRect(name, parent, size, pos);
		Image img = go.AddComponent<Image>();
		img.color = new Color(0.25f, 0.22f, 0.18f);
		Button b = go.AddComponent<Button>();
		b.onClick.AddListener(onClick);

		TMP_Text t = MakeText(go.transform, "Label", Vector2.zero, size, 15f, FontStyles.Bold);
		t.text = label;
		t.color = new Color(0.95f, 0.92f, 0.85f);
		t.alignment = TextAlignmentOptions.Center;
	}
}
