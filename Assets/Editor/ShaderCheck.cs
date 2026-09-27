using System.Linq;
using UnityEditor;
using UnityEditor.Rendering;
using UnityEngine;

//Compiles the project's own shaders for the editor's graphics API and reports
//their errors and warnings, without building a player.
//
//compile_check.sh covers C#; this is the same for shaders.
//Tools > Check Project Shaders, or -batchmode -executeMethod ShaderCheck.Run
//(batch mode needs graphics: leave out -nographics). Exits 1 on any error.
public static class ShaderCheck
{
	static readonly string[] Shaders =
	{
		"FarmSim/Foliage", "FarmSim/Bark", "FarmSim/RobotSurface", "FarmSim/TriplanarMetal",
		"Custom/WaterTrueDepthFade", "Hidden/FarmSim/ScreenSpaceShadows", "FarmSim/Grass", "FarmSim/GrassBlades",
	};

	[MenuItem("Tools/Check Project Shaders")]
	public static void Run()
	{
		int errors = 0;
		foreach (string name in Shaders)
		{
			Shader s = Shader.Find(name);
			if (s == null) { Debug.LogError("ShaderCheck: " + name + " not found"); errors++; continue; }
			//force a compile of every variant the editor would use
			ShaderUtil.CompilePass(new Material(s), 0, true);
			ShaderMessage[] msgs = ShaderUtil.GetShaderMessages(s);
			int e = msgs.Count(m => m.severity == ShaderCompilerMessageSeverity.Error);
			int w = msgs.Length - e;
			errors += e;
			Debug.Log("ShaderCheck: " + name + ": " + e + " errors, " + w + " warnings, supported " + s.isSupported);
			foreach (ShaderMessage m in msgs)
				Debug.Log("ShaderCheck:   " + m.severity + " " + m.platform + " line " + m.line + ": " + m.message);
		}
		Debug.Log("ShaderCheck: " + errors + " errors total");
		if (Application.isBatchMode) EditorApplication.Exit(errors > 0 ? 1 : 0);
	}
}
