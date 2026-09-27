//Short grass blades on the lawn around the camera (GrassBlades.cs draws them with
//DrawMeshInstancedIndirect; this lives under Resources so builds include it).
//
//Per instance, from _GrassBladeData (two float4 per clump): the root position and
//yaw, then scale, dry-grass weight and a colour seed. The mesh is a clump of six
//tapered blades with vertex colour G = height along the blade (0 root, 1 tip),
//R and B = per-blade random numbers.
//
//The blades take their colour from the terrain layer texture under their root,
//so they grow out of the ground they stand on instead of sitting on it, darker
//at the root and lighter at the tip. Lighting is the grass tufts' model
//(FarmSim/Grass: Lambert, the terrain's Disney diffuse, sky sheen, translucency)
//with the sheen worked out per vertex, which is plenty for blades this small.
//
//They lean in FarmWind's wind, are pushed over by up to eight "benders" (the
//robots and the recent points of their tracks, set by GrassBlades.cs as x,
//strength, z, radius) and shrink into the ground toward the edge of the drawn
//area, so the edge never pops.
Shader "FarmSim/GrassBlades"
{
	Properties
	{
		_GroundTex ("Ground: grass layer", 2D) = "white" {}
		_DryTex ("Ground: dry grass layer", 2D) = "white" {}
		_Tint ("Tint", Color) = (1,1,1,1)
		_RootShade ("Shade At The Root", Range(0,1)) = 0.45
		_TipLight ("Lighter At The Tip", Range(0,1)) = 0.4
		_Translucency ("Translucency", Range(0,2)) = 1
		_TranslucencyColor ("Translucency Tint", Color) = (0.95,1,0.6,1)
		_Glossiness ("Sheen Smoothness", Range(0,1)) = 0.12
		_WindWeight ("Wind Weight", Range(0,6)) = 2.5
	}
	SubShader
	{
		Tags { "RenderType"="Opaque" "Queue"="Geometry+10" }
		Cull Off

		CGPROGRAM
		#pragma surface surf Blade vertex:vert addshadow noforwardadd nolightmap nodynlightmap nodirlightmap nometa
		#pragma multi_compile_instancing
		#pragma instancing_options procedural:setup
		#pragma target 4.5
		#include "../FarmWind.cginc"

		sampler2D _GroundTex;
		sampler2D _DryTex;
		//terrain layer tiling in world space: uv = world.xz * xy + zw
		float4 _GroundST;
		float4 _DryST;
		fixed4 _Tint;
		half _RootShade;
		half _TipLight;
		half _Translucency;
		fixed4 _TranslucencyColor;
		half _Glossiness;
		half _WindWeight;
		//x = drawn radius, y = length of the fade at its edge
		float4 _GrassFade;
		float4 _FarmGrassBenders[8];

	#ifdef UNITY_PROCEDURAL_INSTANCING_ENABLED
		StructuredBuffer<float4> _GrassBladeData;
	#endif

		//position, yaw and scale of this clump, as the object matrices
		void setup()
		{
		#ifdef UNITY_PROCEDURAL_INSTANCING_ENABLED
			float4 d0 = _GrassBladeData[unity_InstanceID * 2];
			float k = _GrassBladeData[unity_InstanceID * 2 + 1].x;
			float s, c;
			sincos(d0.w, s, c);
			unity_ObjectToWorld = float4x4(
				c * k, 0, s * k, d0.x,
				0,     k, 0,     d0.y,
				-s * k, 0, c * k, d0.z,
				0,     0, 0,     1);
			float ik = 1.0 / k;
			unity_WorldToObject = float4x4(
				c * ik, 0,  -s * ik, 0,
				0,      ik, 0,       0,
				s * ik, 0,  c * ik,  0,
				0,      0,  0,       1);
			unity_WorldToObject._14_24_34 = -mul((float3x3)unity_WorldToObject, d0.xyz);
		#endif
		}

		struct Input
		{
			half3 bladeColor;
			half3 sheen;
		};

		void vert (inout appdata_full v, out Input o)
		{
			UNITY_INITIALIZE_OUTPUT(Input, o);
			float3 root = float3(0, 0, 0);
			float dry = 0;
			float seed = 0.5;
		#ifdef UNITY_PROCEDURAL_INSTANCING_ENABLED
			float4 d0 = _GrassBladeData[unity_InstanceID * 2];
			float4 d1 = _GrassBladeData[unity_InstanceID * 2 + 1];
			root = d0.xyz;
			dry = d1.y;
			seed = d1.z;
		#endif
			float t = v.color.g;

			//shrink into the ground toward the edge of the drawn area
			float fade = saturate((_GrassFade.x - distance(_WorldSpaceCameraPos.xz, root.xz)) / max(_GrassFade.y, 0.01));
			v.vertex.y *= fade;

			float3 wp = mul(unity_ObjectToWorld, float4(v.vertex.xyz, 1)).xyz;
			float h = max(wp.y - root.y, 0);

			//pushed over by the robots and their recent track
			float2 push = 0;
			for (int i = 0; i < 8; i++)
			{
				float4 b = _FarmGrassBenders[i];
				float2 dv = root.xz - b.xz;
				float d = length(dv);
				push += dv / max(d, 0.001) * (saturate(1 - d / max(b.w, 0.001)) * b.y);
			}
			float amount = saturate(length(push));
			float2 away = push / max(length(push), 0.0001);
			wp.xz += away * (amount * h * 0.9);
			wp.y -= amount * h * 0.55;

			//leaning with the wind, tips most; a quicker flutter per blade on top
			float time = _Time.y * max(_FarmWind.w, 0.01);
			float phase = dot(root.xz, float2(0.37, 0.53)) + v.color.b * 6.2832;
			float sway = 0.65 + 0.35 * sin(time * 1.3 + phase * 0.4);
			float flutter = sin(time * 5.7 + phase * 3.0) * 0.25;
			float2 wind = _FarmWind.xz * _WindWeight;
			wp.xz += wind * ((sway + flutter) * h * t);

			v.vertex.xyz = mul(unity_WorldToObject, float4(wp, 1)).xyz;

			//the ground's own colour under the root, a blurred mip of it
			half3 grass = tex2Dlod(_GroundTex, float4(root.xz * _GroundST.xy + _GroundST.zw, 0, 3)).rgb;
			half3 dryC = tex2Dlod(_DryTex, float4(root.xz * _DryST.xy + _DryST.zw, 0, 3)).rgb;
			//per blade: a little lighter or darker, a little yellower or greener
			half r1 = frac(seed * 13.7 + v.color.r * 3.1);
			half r2 = frac(seed * 7.3 + v.color.r * 5.9);
			half3 c = lerp(grass, dryC, dry) * (0.85 + 0.3 * r1);
			c *= lerp(half3(0.9, 1.05, 0.95), half3(1.12, 1.04, 0.75), r2);
			o.bladeColor = c * lerp(1.0 - _RootShade, 1.0 + _TipLight, t) * _Tint.rgb;

			//the terrain's sky reflection (see FarmSim/Grass), per vertex
			float3 view = normalize(_WorldSpaceCameraPos - wp);
			half nv = saturate(view.y);
			half f = 1.0 - nv;
			half f2 = f * f;
			half fresnel = lerp(0.04, _Glossiness + 0.04, f2 * f2 * f);
			half roughness = (1.0 - _Glossiness) * (1.0 - _Glossiness);
			half3 sky = max(0, ShadeSH9(half4(reflect(-view, float3(0, 1, 0)), 1)));
			o.sheen = sky * fresnel / (roughness * roughness + 1.0) * fade;
		}

		void surf (Input IN, inout SurfaceOutput o)
		{
			o.Albedo = IN.bladeColor;
			o.Emission = IN.sheen;
			o.Alpha = 1;
		}

		//FarmSim/Grass's lighting: Lambert with the terrain's Disney diffuse, and
		//sunlight through the blade when looking toward the sun
		half4 LightingBlade (SurfaceOutput s, half3 lightDir, half3 viewDir, half atten)
		{
			half nl = max(0, dot(s.Normal, lightDir));
			half nv = saturate(dot(s.Normal, viewDir));
			half lh = saturate(dot(lightDir, normalize(lightDir + viewDir)));
			half fd90 = 0.5 + 2.0 * lh * lh * (1.0 - _Glossiness);
			half ol = 1.0 - nl, ov = 1.0 - nv;
			half diffuse = nl * (1.0 + (fd90 - 1.0) * ol * ol * ol * ol * ol) * (1.0 + (fd90 - 1.0) * ov * ov * ov * ov * ov);
			half back = saturate(dot(viewDir, -lightDir));
			back *= back;
			back *= back;
			half3 through = _TranslucencyColor.rgb * (_Translucency * back);
			half4 c;
			c.rgb = s.Albedo * _LightColor0.rgb * atten * (diffuse + through);
			c.a = 1;
			return c;
		}
		ENDCG
	}
	FallBack Off
}
