using UnityEngine;
using UnityEditor;

//Shaders that scripts look up by name at runtime are not part of any scene
//or prefab, so the player build strips them unless the project lists them
//as always included. Without this the laser beam (CropDetector) and the
//showroom rings (RobotShowcase) threw in every build and never showed.
public static class RuntimeShaders
{
	static readonly string[] Names =
	{
		"Legacy Shaders/Particles/Additive",
		"Sprites/Default",
	};

	[MenuItem("Tools/Include Runtime Shaders")]
	public static void Include()
	{
		Object[] assets = AssetDatabase.LoadAllAssetsAtPath("ProjectSettings/GraphicsSettings.asset");
		if (assets == null || assets.Length == 0)
		{
			Debug.LogError("RuntimeShaders: could not load ProjectSettings/GraphicsSettings.asset");
			return;
		}

		SerializedObject settings = new SerializedObject(assets[0]);
		SerializedProperty list = settings.FindProperty("m_AlwaysIncludedShaders");
		if (list == null)
		{
			Debug.LogError("RuntimeShaders: m_AlwaysIncludedShaders not found");
			return;
		}

		int added = 0;
		foreach (string name in Names)
		{
			Shader shader = Shader.Find(name);
			if (shader == null)
			{
				Debug.LogWarning("RuntimeShaders: shader not found in the editor: " + name);
				continue;
			}

			bool present = false;
			for (int i = 0; i < list.arraySize; i++)
			{
				if (list.GetArrayElementAtIndex(i).objectReferenceValue == shader) { present = true; break; }
			}
			if (present) continue;

			list.InsertArrayElementAtIndex(list.arraySize);
			list.GetArrayElementAtIndex(list.arraySize - 1).objectReferenceValue = shader;
			added++;
		}

		settings.ApplyModifiedProperties();
		AssetDatabase.SaveAssets();
		Debug.Log("RuntimeShaders: " + added + " shader(s) added to Always Included Shaders (" + list.arraySize + " total).");
	}
}
