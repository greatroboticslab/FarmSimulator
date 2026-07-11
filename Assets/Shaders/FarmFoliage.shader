//Two-sided cutout foliage shader for crops and weeds. The default one-sided
//shaders make leaves invisible from behind; plants need Cull Off.
Shader "FarmSim/Foliage"
{
	Properties
	{
		_MainTex ("Albedo", 2D) = "white" {}
		_Color ("Tint", Color) = (1,1,1,1)
		_Cutoff ("Alpha Cutoff", Range(0,1)) = 0.35
	}
	SubShader
	{
		Tags { "RenderType"="TransparentCutout" "Queue"="AlphaTest" }
		Cull Off

		CGPROGRAM
		#pragma surface surf Lambert alphatest:_Cutoff addshadow
		sampler2D _MainTex;
		fixed4 _Color;

		struct Input
		{
			float2 uv_MainTex;
		};

		void surf (Input IN, inout SurfaceOutput o)
		{
			fixed4 c = tex2D(_MainTex, IN.uv_MainTex) * _Color;
			o.Albedo = c.rgb;
			o.Alpha = c.a;
		}
		ENDCG
	}
	FallBack "Legacy Shaders/Transparent/Cutout/Diffuse"
}
