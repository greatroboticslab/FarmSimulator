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
	}
}
