//Bark for the generated trees: Standard lighting with a normal map, ambient
//occlusion from vertex colour R, and the same wind bend as the leaf cards
//(FarmWind.cginc), so branches carry their leaves with them instead of the
//leaves sliding along still wood.
Shader "FarmSim/Bark"
{
	Properties
	{
		_MainTex ("Albedo", 2D) = "white" {}
		_Color ("Tint", Color) = (1,1,1,1)
		[Normal] _BumpMap ("Normal Map", 2D) = "bump" {}
		_BumpScale ("Normal Strength", Float) = 1
		_Glossiness ("Smoothness", Range(0,1)) = 0.12
		_OcclusionFromVertex ("Vertex Colour Occlusion", Range(0,1)) = 1
		_WindWeight ("Wind Weight", Range(0,2)) = 1
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }

		CGPROGRAM
		#pragma surface surf Standard fullforwardshadows addshadow vertex:vert dithercrossfade
		#pragma target 3.0
		#include "FarmWind.cginc"

		sampler2D _MainTex;
		sampler2D _BumpMap;
		fixed4 _Color;
		half _BumpScale;
		half _Glossiness;
		half _OcclusionFromVertex;
		half _WindWeight;

		struct Input
		{
			float2 uv_MainTex;
			fixed4 color : COLOR;
		};

		void vert (inout appdata_full v)
		{
			v.vertex.xyz = FarmWindDisplace(v.vertex.xyz, v.normal, v.color, _WindWeight);
		}

		void surf (Input IN, inout SurfaceOutputStandard o)
		{
			fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
			o.Albedo = c.rgb;
			o.Normal = UnpackScaleNormal(tex2D(_BumpMap, IN.uv_MainTex), _BumpScale);
			o.Metallic = 0;
			o.Smoothness = _Glossiness;
			o.Occlusion = lerp(1.0, IN.color.r, _OcclusionFromVertex);
		}
		ENDCG
	}
	FallBack "Diffuse"
}
