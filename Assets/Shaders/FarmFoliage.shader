//Two-sided cutout foliage shader for crops, weeds and tree leaf cards.
//
//The default one-sided shaders make leaves invisible from behind, so plants need
//Cull Off. With Cull Off a back face would otherwise be lit with the front face's
//normal; the normal is flipped for back faces (VFACE), and light arriving from
//behind a leaf shows through it (translucency), which is most of what makes
//sunlit foliage look alive.
//
//Everything added after the original version defaults to off, so existing crop
//materials render exactly as before:
// - _BumpMap: flat by default
// - _Translucency: 0
// - _OcclusionFromVertex: 0; generated trees store ambient occlusion in vertex
//   colour R (inner leaves darker)
// - _WindWeight: 0; generated trees store how far each vertex may bend in vertex
//   colour G and a per-cluster phase in B. The wind itself is global
//   (_FarmWind, set by FarmWind.cs) so every tree moves in the same wind.
// - LOD cross-fade: dithered, only active when a LODGroup fades
Shader "FarmSim/Foliage"
{
	Properties
	{
		_MainTex ("Albedo", 2D) = "white" {}
		_Color ("Tint", Color) = (1,1,1,1)
		_Cutoff ("Alpha Cutoff", Range(0,1)) = 0.35
		[Normal] _BumpMap ("Normal Map", 2D) = "bump" {}
		_Translucency ("Translucency", Range(0,1)) = 0
		_TranslucencyColor ("Translucency Tint", Color) = (0.73,0.85,0.41,1)
		_OcclusionFromVertex ("Vertex Colour Occlusion", Range(0,1)) = 0
		_WindWeight ("Wind Weight", Range(0,2)) = 0
	}
	SubShader
	{
		Tags { "RenderType"="TransparentCutout" "Queue"="AlphaTest" }
		Cull Off

		CGPROGRAM
		#pragma surface surf Foliage alphatest:_Cutoff addshadow vertex:vert dithercrossfade
		#pragma target 3.0
		#include "FarmWind.cginc"

		sampler2D _MainTex;
		sampler2D _BumpMap;
		fixed4 _Color;
		half _Translucency;
		fixed4 _TranslucencyColor;
		half _OcclusionFromVertex;
		half _WindWeight;

		struct Input
		{
			float2 uv_MainTex;
			fixed4 color : COLOR;
			fixed facing : VFACE;
		};

		struct SurfaceOutputFoliage
		{
			fixed3 Albedo;
			fixed3 Normal;
			fixed3 Emission;
			half Specular;
			fixed Gloss;
			fixed Alpha;
			fixed Translucency;
		};

		void vert (inout appdata_full v)
		{
			v.vertex.xyz = FarmWindDisplace(v.vertex.xyz, v.normal, v.color, _WindWeight);
		}

		void surf (Input IN, inout SurfaceOutputFoliage o)
		{
			fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
			o.Albedo = c.rgb * lerp(1.0, IN.color.r, _OcclusionFromVertex);
			o.Alpha = c.a;
			fixed3 n = UnpackNormal(tex2D(_BumpMap, IN.uv_MainTex));
			//the back of a leaf faces the other way
			n.z *= IN.facing > 0 ? 1.0 : -1.0;
			o.Normal = n;
			o.Translucency = _Translucency * lerp(1.0, IN.color.r, _OcclusionFromVertex);
		}

		half4 LightingFoliage (SurfaceOutputFoliage s, half3 lightDir, half atten)
		{
			half ndl = dot(s.Normal, lightDir);
			half diffuse = max(0, ndl);
			//light passing through from behind the leaf
			half through = max(0, -ndl) * s.Translucency;
			half4 c;
			c.rgb = s.Albedo * _LightColor0.rgb * atten * (diffuse + through * _TranslucencyColor.rgb);
			c.a = s.Alpha;
			return c;
		}
		ENDCG
	}
	FallBack "Legacy Shaders/Transparent/Cutout/Diffuse"
}
