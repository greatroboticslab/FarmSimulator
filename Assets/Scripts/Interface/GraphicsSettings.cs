using UnityEngine;
using UnityEngine.UI;
using TMPro;

//Player-facing graphics quality control. Adds a Graphics button to the HUD
//that cycles quality levels, scales the expensive terrain grass with the
//chosen level, and remembers the choice between runs.
public class GraphicsSettings : MonoBehaviour
{
	const string PrefKey = "FarmSim_QualityLevel";

	static Button hudButton;

	//Restore the player's saved quality before anything renders
	[RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
	static void ApplySavedQuality()
	{
		if (PlayerPrefs.HasKey(PrefKey))
		{
			int level = Mathf.Clamp(PlayerPrefs.GetInt(PrefKey), 0, QualitySettings.names.Length - 1);
			QualitySettings.SetQualityLevel(level, true);
		}
		ApplyTerrainCost();
	}

	//Clones an existing HUD button so Graphics matches its siblings
	public static void AttachHudButton(Button template)
	{
		if (template == null || template.transform.parent == null) return;
		if (template.transform.parent.Find("GraphicsButton") != null) return;

		hudButton = Object.Instantiate(template, template.transform.parent);
		hudButton.name = "GraphicsButton";
		hudButton.onClick.RemoveAllListeners();
		hudButton.onClick.AddListener(CycleQuality);

		RectTransform rt = hudButton.GetComponent<RectTransform>();
		RectTransform src = template.GetComponent<RectTransform>();
		//sits below the Tutorial button, which is one slot below the template
		rt.anchoredPosition = src.anchoredPosition + new Vector2(0f, -2f * (src.sizeDelta.y + 4f));

		RefreshLabel();
	}

	public static void CycleQuality()
	{
		int next = (QualitySettings.GetQualityLevel() + 1) % QualitySettings.names.Length;
		SetQuality(next);
	}

	public static void SetQuality(int level)
	{
		level = Mathf.Clamp(level, 0, QualitySettings.names.Length - 1);
		QualitySettings.SetQualityLevel(level, true);
		PlayerPrefs.SetInt(PrefKey, level);
		PlayerPrefs.Save();
		ApplyTerrainCost();
		RefreshLabel();
		Debug.Log("[GraphicsSettings] quality set to " + QualitySettings.names[level]);
	}

	//Detail grass is the heaviest thing in the scene, so tie it to the level
	//instead of leaving it fixed regardless of what the player picked
	public static void ApplyTerrainCost()
	{
		int level = QualitySettings.GetQualityLevel();

		float detailDistance;
		float detailDensity;
		float shadowDistance;
		if (level <= 0) { detailDistance = 0f; detailDensity = 0f; shadowDistance = 40f; }
		else if (level == 1) { detailDistance = 35f; detailDensity = 0.4f; shadowDistance = 60f; }
		else if (level == 2) { detailDistance = 70f; detailDensity = 0.7f; shadowDistance = 90f; }
		else if (level == 3) { detailDistance = 100f; detailDensity = 0.85f; shadowDistance = 120f; }
		else { detailDistance = 140f; detailDensity = 1f; shadowDistance = 160f; }

		QualitySettings.shadowDistance = shadowDistance;

		foreach (Terrain t in Terrain.activeTerrains)
		{
			if (t == null) continue;
			t.detailObjectDistance = detailDistance;
			t.detailObjectDensity = detailDensity;
			t.drawTreesAndFoliage = detailDistance > 0f;
		}
	}

	static void RefreshLabel()
	{
		if (hudButton == null) return;
		string text = "Graphics: " + QualitySettings.names[QualitySettings.GetQualityLevel()];

		TMP_Text label = hudButton.GetComponentInChildren<TMP_Text>();
		if (label != null) { label.text = text; return; }

		Text legacy = hudButton.GetComponentInChildren<Text>();
		if (legacy != null) legacy.text = text;
	}
}
