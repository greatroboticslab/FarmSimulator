using UnityEngine;
using UnityEditor;
using UnityEditor.SceneManagement;

//One-shot lighting and atmosphere pass for the main scene.
//
//The sky, the sun, ambient light, reflections and the fog colour all come from
//the HDRI now (SkySetup). This used to build a procedural sky with trilight
//ambient and a hand-picked sun; running that version today would quietly undo
//the HDRI, so it delegates to SkySetup and only adds the fog distances.
//
//Shadow distance is left alone: GraphicsSettings sets it per quality tier at
//runtime, and setting it here only rewrote the current level in
//QualitySettings.asset.
public static class ScenePolish
{
	[MenuItem("Tools/Polish Scene Lighting")]
	public static void Polish()
	{
		SkySetup.Install();

		RenderSettings.fog = true;
		RenderSettings.fogMode = FogMode.Linear;
		RenderSettings.fogStartDistance = 160f;
		RenderSettings.fogEndDistance = 950f;

		EditorSceneManager.MarkAllScenesDirty();
		EditorSceneManager.SaveOpenScenes();
		AssetDatabase.SaveAssets();
		Debug.Log("ScenePolish: HDRI sky installed, distance fog set, scene saved.");
	}
}
