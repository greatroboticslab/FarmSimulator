//Grass tufts on the terrain (FarmTerrainBuilder's instanced detail meshes).
//
//The tufts have to sit in the ground they grow from. The terrain is drawn with
//the Standard model, and at grazing angles a good share of its brightness is
//specular: looking toward the sun, the sun's broad sheen, and everywhere, the
//sky reflected at a low angle. A plain Lambert tuft has neither, so tufts read
//as dark cut-outs at half the ground's brightness (measured: 0.42-0.50). Real
//grass does brighten in those views, mostly because the blades let light
//through. So on top of Lambert:
// - translucency: sunlight through the blades when the camera looks toward the
//   sun, tinted by the blade, shadowed like the sun itself
// - sheen: the terrain's own sky-reflection term (Unity's FresnelLerp and
//   surface reduction at the layer's smoothness) with the sky read from the
//   ambient SH instead of a cubemap, which is what keeps it cheap
// - the Disney diffuse the terrain's Standard model gives the sun, which
//   brightens rough surfaces toward grazing angles
//The full Standard model got closer on sheen alone and cost 1.3 ms of GPU at
//High 1080p on the tufts' overdraw, so not that. VegetationSetup sets the
//material's tint from measurements of tuft against ground.
//
//The mesh is two crossed quads 1 m tall with the root at y = 0 and normals
//pointing up, so height does the work vertex colour G does on the trees: how
//far a vertex bends in the wind (FarmWind.cginc).
Shader "FarmSim/Grass"
{
	Properties
	{
		_MainTex ("Albedo", 2D) = "white" {}
		_Color ("Tint", Color) = (1,1,1,1)
		_Cutoff ("Alpha Cutoff", Range(0,1)) = 0.45
		_Translucency ("Translucency", Range(0,2)) = 1
		_TranslucencyColor ("Translucency Tint", Color) = (0.95,1,0.6,1)
		_Glossiness ("Sheen Smoothness (the terrain layer's)", Range(0,1)) = 0.12
		_WindWeight ("Wind Weight", Range(0,2)) = 1
	}
	SubShader
	{
		Tags { "RenderType"="TransparentCutout" "Queue"="AlphaTest" }
		Cull Off

		CGPROGRAM
		#pragma surface surf Grass alphatest:_Cutoff addshadow vertex:vert noforwardadd
		#pragma target 3.0
		#include "FarmWind.cginc"

		sampler2D _MainTex;
		fixed4 _Color;
		half _Translucency;
		fixed4 _TranslucencyColor;
		half _Glossiness;
		half _WindWeight;

		struct Input
		{
			float2 uv_MainTex;
			float3 viewDir;
			float3 worldNormal;
		};

		void vert (inout appdata_full v)
		{
			//G = flexibility from height (FarmWindDisplace squares it, so the
			//root stays put), B = a fixed phase; the tuft's position already
			//varies it. A zero normal leaves out the leaf flutter, which on
			//up-facing normals would bob the blades up and down.
			fixed4 bend = fixed4(1, saturate(v.vertex.y), 0.5, 1);
			v.vertex.xyz = FarmWindDisplace(v.vertex.xyz, float3(0, 0, 0), bend, _WindWeight);
		}

		void surf (Input IN, inout SurfaceOutput o)
		{
			fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
			o.Albedo = c.rgb;
			o.Alpha = c.a;

			//the terrain's sky reflection (UnityStandardBRDF: FresnelLerp from the
			//dielectric 0.04 to smoothness + 0.04, times 1 / (roughness^2 + 1))
			half3 n = normalize(IN.worldNormal);
			half3 v = normalize(IN.viewDir);
			half nv = saturate(dot(n, v));
			half t = 1.0 - nv;
			half t2 = t * t;
			half fresnel = lerp(0.04, _Glossiness + 0.04, t2 * t2 * t);
			half roughness = (1.0 - _Glossiness) * (1.0 - _Glossiness);
			half3 sky = max(0, ShadeSH9(half4(reflect(-v, n), 1)));
			o.Emission = sky * fresnel / (roughness * roughness + 1.0);
		}

		half4 LightingGrass (SurfaceOutput s, half3 lightDir, half3 viewDir, half atten)
		{
			half nl = max(0, dot(s.Normal, lightDir));
			//the Disney diffuse the terrain's Standard model applies to the sun
			//(UnityStandardBRDF DisneyDiffuse): rough surfaces brighten toward
			//grazing angles, and the tuft shares the ground's up-facing normal
			half nv = saturate(dot(s.Normal, viewDir));
			half lh = saturate(dot(lightDir, normalize(lightDir + viewDir)));
			half fd90 = 0.5 + 2.0 * lh * lh * (1.0 - _Glossiness);
			half ol = 1.0 - nl, ov = 1.0 - nv;
			half diffuse = nl * (1.0 + (fd90 - 1.0) * ol * ol * ol * ol * ol) * (1.0 + (fd90 - 1.0) * ov * ov * ov * ov * ov);
			//how directly the camera looks through the tuft toward the light
			half back = saturate(dot(viewDir, -lightDir));
			back *= back;
			back *= back;
			half3 through = _TranslucencyColor.rgb * (_Translucency * back);
			half4 c;
			c.rgb = s.Albedo * _LightColor0.rgb * atten * (diffuse + through);
			c.a = s.Alpha;
			return c;
		}
		ENDCG
	}
	FallBack "Legacy Shaders/Transparent/Cutout/VertexLit"
}
