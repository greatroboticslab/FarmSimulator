#ifndef FARM_WIND_INCLUDED
#define FARM_WIND_INCLUDED

//Wind shared by every tree and plant that opts in (material _WindWeight > 0).
//Set every frame by FarmWind.cs: xyz = direction * strength in metres of lean at
//full flexibility, w = speed. Zero until something sets it, so nothing moves in
//a scene without the driver.
float4 _FarmWind;

//Vertex colour conventions of the generated vegetation:
//  G = flexibility, 0 at the trunk rising to 1 at leaf-cluster tips
//  B = a per-cluster phase, so neighbouring clusters do not move in lockstep
//Each object also takes a phase from where it stands.
float3 FarmWindDisplace(float3 objPos, float3 objNormal, fixed4 color, half weight)
{
	if (weight <= 0) return objPos;

	float3 origin = mul(unity_ObjectToWorld, float4(0, 0, 0, 1)).xyz;
	float t = _Time.y * max(_FarmWind.w, 0.01);
	float phase = dot(origin.xz, float2(0.071, 0.113)) + color.b * 6.2832;

	//the crown leans with the wind and rocks around that lean; leaf clusters
	//flutter faster and smaller on top of it
	float sway = 0.7 + 0.3 * (sin(t * 0.83 + phase * 0.3) * 0.65 + sin(t * 1.91 + phase * 0.7) * 0.35);
	float flutter = sin(t * 6.1 + phase * 3.0) * 0.5 + sin(t * 9.7 + phase * 5.0) * 0.5;
	float flex = color.g * color.g;

	float3 world = _FarmWind.xyz * (sway * flex * weight);
	world += mul((float3x3)unity_ObjectToWorld, objNormal) * (flutter * color.g * length(_FarmWind.xyz) * 0.12 * weight);
	return objPos + mul((float3x3)unity_WorldToObject, world);
}

#endif
