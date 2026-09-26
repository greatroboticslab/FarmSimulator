//Robot parts after the UV and bake pass (tools/robot_bake.py, robot_masks.py).
//
//Keeps each part's own paint: colour, metallic and smoothness come from the
//material it replaces, and so does the world-projected micro detail of
//FarmSim/TriplanarMetal. On top of that a per-part mask adds what makes a
//working machine read as used:
//  R  ambient occlusion from the bake
//  G  1 - edge wear: paint chipped off the convex edges down to bare metal
//  B  1 - grime: dirt collected where the part is occluded, and a film of dust
//White in every channel means no effect.
Shader "FarmSim/RobotSurface"
{
	Properties
	{
		_Color ("Paint", Color) = (1,1,1,1)
		_MainTex ("Albedo (optional)", 2D) = "white" {}
		_Metallic ("Metallic", Range(0,1)) = 0.3
		_Glossiness ("Smoothness", Range(0,1)) = 0.5
		_MaskTex ("Mask (R AO, G 1-wear, B 1-grime)", 2D) = "white" {}
		_AOStrength ("Occlusion", Range(0,1)) = 1
		_WearColor ("Worn Edge", Color) = (0.78,0.78,0.76,1)
		_WearMetallic ("Worn Edge Metallic", Range(0,1)) = 1
		_WearSmoothness ("Worn Edge Smoothness", Range(0,1)) = 0.55
		_WearStrength ("Edge Wear", Range(0,1)) = 0.8
		_GrimeColor ("Grime", Color) = (0.24,0.21,0.17,1)
		_GrimeStrength ("Grime Amount", Range(0,1)) = 0.6
		_DetailTex ("Detail (world projected)", 2D) = "gray" {}
		_DetailScale ("Detail Scale", Float) = 0.6
		_DetailStrength ("Detail Strength", Range(0,1)) = 0
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }

		CGPROGRAM
		#pragma surface surf Standard fullforwardshadows
		#pragma target 3.0

		sampler2D _MainTex;
		sampler2D _MaskTex;
		sampler2D _DetailTex;
		fixed4 _Color;
		half _Metallic, _Glossiness;
		half _AOStrength;
		fixed4 _WearColor;
		half _WearMetallic, _WearSmoothness, _WearStrength;
		fixed4 _GrimeColor;
		half _GrimeStrength;
		float _DetailScale;
		half _DetailStrength;

		struct Input
		{
			float2 uv_MaskTex;
			float2 uv_MainTex;
			float3 worldPos;
			float3 worldNormal;
		};

		void surf (Input IN, inout SurfaceOutputStandard o)
		{
			fixed3 mask = tex2D(_MaskTex, IN.uv_MaskTex).rgb;
			half ao = lerp(1.0, mask.r, _AOStrength);
			half wear = (1.0 - mask.g) * _WearStrength;
			half grime = (1.0 - mask.b) * _GrimeStrength;

			//the world-projected detail FarmSim/TriplanarMetal used, so parts that
			//had it keep it (0.5 is neutral)
			float3 blend = pow(abs(normalize(IN.worldNormal)), 4);
			blend /= (blend.x + blend.y + blend.z);
			fixed d = tex2D(_DetailTex, IN.worldPos.zy * _DetailScale).r * blend.x
				+ tex2D(_DetailTex, IN.worldPos.xz * _DetailScale).r * blend.y
				+ tex2D(_DetailTex, IN.worldPos.xy * _DetailScale).r * blend.z;

			fixed3 col = _Color.rgb * tex2D(_MainTex, IN.uv_MainTex).rgb * lerp(1.0, d * 2.0, _DetailStrength);
			col = lerp(col, _GrimeColor.rgb, grime);
			col = lerp(col, _WearColor.rgb, wear);

			o.Albedo = col;
			o.Metallic = lerp(lerp(_Metallic, 0.0, grime), _WearMetallic, wear);
			o.Smoothness = lerp(_Glossiness * (1.0 - 0.6 * grime) * lerp(1.0, 0.7 + 0.6 * d, _DetailStrength), _WearSmoothness, wear);
			o.Occlusion = ao;
		}
		ENDCG
	}
	FallBack "Diffuse"
}
