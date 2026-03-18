cbuffer SceneConstants : register(b0)
{
    float4x4 gMvp;
};

struct VSInput
{
    float3 position : POSITION;
    float2 uv : TEXCOORD;
};

struct PSInput
{
    float4 position : SV_POSITION;
    float2 uv : TEXCOORD;
};

PSInput VSMain(VSInput input)
{
    PSInput output;
    output.position = mul(float4(input.position, 1.0f), gMvp);
    output.uv = input.uv;
    return output;
}

float4 PSMain(PSInput input) : SV_TARGET
{
    const float2 tileCoord = floor(float2(input.uv.x * 8.0f, input.uv.y * 3.0f));
    const float checker = fmod(tileCoord.x + tileCoord.y, 2.0f);
    const float3 colorA = float3(0.16f, 0.18f, 0.20f);
    const float3 colorB = float3(0.88f, 0.84f, 0.72f);
    return float4(lerp(colorA, colorB, checker), 1.0f);
}
