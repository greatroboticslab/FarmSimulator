Shader "Custom/WaterTrueDepthFade"
{
    Properties
    {
        _NoiseTex1 ("Noise Texture 1", 2D) = "white" {}
        _NoiseTex2 ("Noise Texture 2", 2D) = "white" {}
        _Speed1 ("Speed 1", Vector) = (0.1, 0.1, 0, 0)
        _Speed2 ("Speed 2", Vector) = (-0.1, 0.05, 0, 0)
        _RefractionStrength ("Refraction Strength", Float) = 0.05
        _DepthInfluence ("Depth Influence Multiplier", Float) = 5.0
        _DeepColor ("Deep Water Color", Color) = (0.0, 0.3, 0.6, 1.0)
    }

    SubShader
    {
        Tags { "Queue"="Transparent" "RenderType"="Transparent" }
        GrabPass { "_GrabTex" }

        Pass
        {
            ZWrite Off
            Blend SrcAlpha OneMinusSrcAlpha

            CGPROGRAM
            #pragma vertex vert
            #pragma fragment frag
            #include "UnityCG.cginc"

            sampler2D _NoiseTex1, _NoiseTex2;
            sampler2D _GrabTex;
            sampler2D _CameraDepthTexture;
            float4 _Speed1, _Speed2;
            float _RefractionStrength;
            float _DepthInfluence;
            float4 _DeepColor;
            float4 _NoiseTex1_ST, _NoiseTex2_ST;

            struct appdata
            {
                float4 vertex : POSITION;
                float2 uv : TEXCOORD0;
            };

            struct v2f
            {
                float2 uv1 : TEXCOORD0;
                float2 uv2 : TEXCOORD1;
                float4 vertex : SV_POSITION;
                float4 screenUV : TEXCOORD2;
            };

            v2f vert (appdata v)
            {
                v2f o;
                o.vertex = UnityObjectToClipPos(v.vertex);
                o.uv1 = TRANSFORM_TEX(v.uv, _NoiseTex1);
                o.uv2 = TRANSFORM_TEX(v.uv, _NoiseTex2);
                o.screenUV = ComputeGrabScreenPos(o.vertex);
                return o;
            }

            fixed4 frag (v2f i) : SV_Target
            {
                // Scroll noise distortion
                float2 offset1 = tex2D(_NoiseTex1, i.uv1 + _Time.y * _Speed1.xy).rg;
                float2 offset2 = tex2D(_NoiseTex2, i.uv2 + _Time.y * _Speed2.xy).rg;
                float2 totalOffset = (offset1 + offset2 - 1.0) * _RefractionStrength;

                float2 grabUV = i.screenUV.xy / i.screenUV.w + totalOffset;
                fixed4 sceneColor = tex2D(_GrabTex, grabUV);

                // Get background depth at current screen pixel
                float rawSceneDepth = SAMPLE_DEPTH_TEXTURE_PROJ(_CameraDepthTexture, UNITY_PROJ_COORD(i.screenUV));
                float sceneLinearDepth = LinearEyeDepth(rawSceneDepth);

                // Depth of the water surface (this fragment)
                float surfaceLinearDepth = LinearEyeDepth(i.screenUV.z / i.screenUV.w);

                // Difference = underwater depth
                float depthUnderwater = sceneLinearDepth - surfaceLinearDepth;
                depthUnderwater = max(depthUnderwater, 0); // prevent negative depth

                // Depth fade control (scaled)
                float alpha = saturate(1.0 - depthUnderwater * _DepthInfluence);

                // Final water color blending
                fixed4 finalColor = lerp(_DeepColor, sceneColor, alpha);
                finalColor.a = alpha;

                return finalColor;
            }
            ENDCG
        }
    }

    FallBack "Transparent"
}
