using UnityEngine;
using UnityEditor;
using UnityEditor.Build.Reporting;
using System.IO;

//Repeatable standalone build. Output lands in Builds/Windows (gitignored);
//zip that folder to distribute the game.
public static class BuildTools
{
	[MenuItem("Tools/Build Windows Player")]
	public static void BuildWindows()
	{
		string dir = "Builds/Windows";
		Directory.CreateDirectory(dir);

		//shaders that scripts find by name must be listed or the player strips them
		RuntimeShaders.Include();

		BuildPlayerOptions opts = new BuildPlayerOptions
		{
			scenes = new[] { "Assets/Scenes/SampleScene.unity" },
			locationPathName = dir + "/FarmSimulator.exe",
			target = BuildTarget.StandaloneWindows64,
			options = BuildOptions.None
		};

		BuildReport report = BuildPipeline.BuildPlayer(opts);
		Debug.Log("BuildTools: " + report.summary.result
			+ ", " + (report.summary.totalSize / (1024 * 1024)) + " MB"
			+ ", errors " + report.summary.totalErrors
			+ " -> " + report.summary.outputPath);

		if (report.summary.result == BuildResult.Succeeded) CopyDataFiles(dir + "/FarmSimulator_Data");
	}

	//CoordSelector reads Assets/Data/locations.csv through Application.dataPath,
	//which Unity does not copy into a player; without it the farm list is empty
	//and "Go to Farm" cannot load a map. Ship the folder next to the player data.
	static void CopyDataFiles(string dataDir)
	{
		const string src = "Assets/Data";
		if (!Directory.Exists(src)) return;
		string dst = Path.Combine(dataDir, "Data");
		Directory.CreateDirectory(dst);
		int copied = 0;
		foreach (string file in Directory.GetFiles(src))
		{
			if (file.EndsWith(".meta")) continue;
			File.Copy(file, Path.Combine(dst, Path.GetFileName(file)), true);
			copied++;
		}
		Debug.Log("BuildTools: copied " + copied + " data file(s) to " + dst);
	}
}
