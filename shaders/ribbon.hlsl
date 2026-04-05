cbuffer SceneConstants : register(b0)
{
    float4x4 gMvp;
};

struct VSInput
{
    float3 position : POSITION;
    float2 uv : TEXCOORD0;
    float surfaceKind : TEXCOORD1;
};

struct PSInput
{
    float4 position : SV_POSITION;
    float2 uv : TEXCOORD0;
    float surfaceKind : TEXCOORD1;
    float3 worldPosition : TEXCOORD2;
};

PSInput VSMain(VSInput input)
{
    PSInput output;
    output.position = mul(float4(input.position, 1.0f), gMvp);
    output.uv = input.uv;
    output.surfaceKind = input.surfaceKind;
    output.worldPosition = input.position;
    return output;
}

float Hash21(float2 sampleUv)
{
    sampleUv = frac(sampleUv * float2(0.1031f, 0.1030f));
    sampleUv += dot(sampleUv, sampleUv.yx + 33.33f);
    return frac((sampleUv.x + sampleUv.y) * sampleUv.x);
}

float ValueNoise(float2 sampleUv)
{
    const float2 cell = floor(sampleUv);
    const float2 local = frac(sampleUv);
    const float2 smoothLocal = local * local * (3.0f - 2.0f * local);

    const float bottomLeft = Hash21(cell + float2(0.0f, 0.0f));
    const float bottomRight = Hash21(cell + float2(1.0f, 0.0f));
    const float topLeft = Hash21(cell + float2(0.0f, 1.0f));
    const float topRight = Hash21(cell + float2(1.0f, 1.0f));

    const float bottom = lerp(bottomLeft, bottomRight, smoothLocal.x);
    const float top = lerp(topLeft, topRight, smoothLocal.x);
    return lerp(bottom, top, smoothLocal.y);
}

float StripeMask(float center, float halfWidth, float coordinate)
{
    const float distanceFromCenter = abs(coordinate - center);
    const float feather = max(fwidth(coordinate) * 0.75f, 0.0025f);
    return 1.0f - smoothstep(halfWidth - feather, halfWidth + feather, distanceFromCenter);
}

float DashMask(float coordinate, float dutyCycle)
{
    const float repeated = frac(coordinate);
    const float feather = max(fwidth(coordinate) * 1.25f, 0.01f);
    const float beginMask = smoothstep(0.0f, feather, repeated);
    const float endMask = 1.0f - smoothstep(dutyCycle - feather, dutyCycle + feather, repeated);
    return beginMask * endMask;
}

float3 BaseAsphalt(float2 worldUv)
{
    const float macroNoise = ValueNoise(worldUv * 0.35f + float2(8.7f, 2.3f));
    const float midNoise = ValueNoise(worldUv * 2.4f + float2(4.1f, 9.2f));
    const float fineNoise = ValueNoise(worldUv * 10.0f + float2(15.7f, 3.4f));

    float3 asphaltColor = lerp(float3(0.09f, 0.095f, 0.10f), float3(0.17f, 0.175f, 0.18f), macroNoise);
    asphaltColor += (midNoise - 0.5f) * 0.025f;
    asphaltColor += (fineNoise - 0.5f) * 0.010f;

    const float aggregate = ValueNoise(worldUv * 20.0f + float2(1.7f, 6.8f));
    asphaltColor += (aggregate - 0.5f) * 0.006f;
    return saturate(asphaltColor);
}

float4 PSMain(PSInput input) : SV_TARGET
{
    const float2 worldUv = input.worldPosition.xz;
    const float2 roadUv = input.uv;
    const float roadV = roadUv.y * 2.5f;
    const float intersectionMask = step(0.5f, input.surfaceKind);

    const float3 asphaltColor = BaseAsphalt(worldUv);
    float3 roadColor = asphaltColor;

    const float laneWear =
        max(StripeMask(0.32f, 0.10f, roadUv.x), StripeMask(0.68f, 0.10f, roadUv.x));
    roadColor -= laneWear * 0.04f;

    const float shoulderDust = pow(abs(roadUv.x * 2.0f - 1.0f), 1.6f);
    roadColor += shoulderDust * float3(0.05f, 0.045f, 0.03f);

    float edgePaintMask =
        max(StripeMask(0.08f, 0.022f, roadUv.x), StripeMask(0.92f, 0.022f, roadUv.x));
    edgePaintMask *= 0.85f + 0.15f * ValueNoise(float2(roadV * 0.7f, roadUv.x * 13.0f));

    float centerPaintMask = StripeMask(0.50f, 0.026f, roadUv.x);
    centerPaintMask *= DashMask(roadV, 0.42f);
    centerPaintMask *= 0.55f + 0.45f * ValueNoise(float2(roadV * 1.6f, 4.0f + roadUv.x * 5.0f));

    const float seamGlow = smoothstep(0.22f, 0.0f, abs(roadUv.x - 0.5f));
    roadColor += seamGlow * 0.01f;

    const float3 edgePaintColor = float3(0.92f, 0.90f, 0.82f);
    const float3 centerPaintColor = float3(0.94f, 0.72f, 0.20f);

    roadColor = lerp(roadColor, edgePaintColor, saturate(edgePaintMask));
    roadColor = lerp(roadColor, centerPaintColor, saturate(centerPaintMask));
    roadColor = lerp(roadColor, asphaltColor, intersectionMask);

    return float4(saturate(roadColor), 1.0f);
}
