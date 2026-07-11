//World-projected detail shader for the robot bodies. The lab models have no
//UV maps, so this projects a wear/grain texture from the world axes instead:
//any mesh gets surface detail with zero asset changes.
Shader "FarmSim/TriplanarMetal"
{
	Properties
	{
		_Color ("Tint", Color) = (1,1,1,1)
		_DetailTex ("Detail (world projected)", 2D) = "gray" {}
		_DetailScale ("Detail Scale", Float) = 0.6
		_DetailStrength ("Detail Strength", Range(0,1)) = 0.45
		_Metallic ("Metallic", Range(0,1)) = 0.5
		_Glossiness ("Smoothness", Range(0,1)) = 0.5
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" }

		CGPROGRAM
		#pragma surface surf Standard fullforwardshadows
		#pragma target 3.0

		sampler2D _DetailTex;
		float _DetailScale;
		float _DetailStrength;
		float _Metallic;
		float _Glossiness;
		fixed4 _Color;

		struct Input
		{
			float3 worldPos;
			float3 worldNormal;
		};

		void surf (Input IN, inout SurfaceOutputStandard o)
		{
			float3 blend = abs(normalize(IN.worldNormal));
			blend = pow(blend, 4);
			blend /= (blend.x + blend.y + blend.z);

			fixed dx = tex2D(_DetailTex, IN.worldPos.zy * _DetailScale).r;
			fixed dy = tex2D(_DetailTex, IN.worldPos.xz * _DetailScale).r;
			fixed dz = tex2D(_DetailTex, IN.worldPos.xy * _DetailScale).r;
			fixed d = dx * blend.x + dy * blend.y + dz * blend.z;

			//d is 0.5 neutral: darker values read as grime, brighter as scratches
			fixed3 col = _Color.rgb * lerp(1.0, d * 2.0, _DetailStrength);

			o.Albedo = col;
			o.Metallic = _Metallic;
			//worn spots lose polish, scratches catch it
			o.Smoothness = _Glossiness * lerp(1.0, 0.7 + 0.6 * d, _DetailStrength);
		}
		ENDCG
	}
	FallBack "Diffuse"
}
