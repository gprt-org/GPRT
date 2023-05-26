// MIT License

// Copyright (c) 2022 Nathan V. Morrical

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "sharedCode.h"


float3x3 setCamera(float3 ro, float3 ta, float cr)
{
    float3 cw = normalize(ta - ro);
    float3 cp = float3(sin(cr), cos(cr), 0.0f);
    float3 cu = normalize(cross(cw, cp));
    float3 cv = cross(cu, cw);
    return float3x3(float3(cu), float3(cv), float3(cw));
}

// A bounding box, I think.
// https://iquilezles.org/articles/boxfunctions
float2 iBox(float3 ro, float3 rd, float3 rad)
{
    float3 m = 1.0f / rd;
    float3 n = m * ro;
    float3 k = abs(m) * rad;
    float3 t1 = (-n) - k;
    float3 t2 = (-n) + k;
    return float2(max(max(t1.x, t1.y), t1.z), min(min(t2.x, t2.y), t2.z));
}

//HSV (hue, saturation, value) to RGB.
//Sources: https://gist.github.com/yiwenl/745bfea7f04c456e0101, https://gist.github.com/sugi-cho/6a01cae436acddd72bdf
float3 hsv2rgb(float3 c)
{
    float4 K = float4(1.0f, 2./3., 1./3., 3.0f);
    return lerp(K.xxx, clamp(abs((frac(c.x.xxx + K.xyz) * 6.0f) - K.w.xxx) - K.x.xxx, 0.0f.xxx, 1.0f.xxx), c.y.xxx) * c.z;
}

//RGB to HSV.
//Source: https://gist.github.com/yiwenl/745bfea7f04c456e0101
float3 rgb2hsv(float3 c)
{
    float cMax = max(max(c.x, c.y), c.z);
    float cMin = min(min(c.x, c.y), c.z);
    float delta = cMax - cMin;
    float3 hsv = float3(0.0f, 0.0f, cMax);
    if (cMax > cMin)
    {
        hsv.y = delta / cMax;
        if (c.x == cMax)
        {
            hsv.x = (c.y - c.z) / delta;
        }
        else
        {
            if (c.y == cMax)
            {
                hsv.x = 2.0f + ((c.z - c.x) / delta);
            }
            else
            {
                hsv.x = 4.0f + ((c.x - c.y) / delta);
            }
        }
        hsv.x = frac(hsv.x / 6.0f);
    }
    return hsv;
}

// This function adds a bit more red instead of direct interpolation
float3 hsvColorscale(float x)
{
    float3 color1 = float3(1.f, 1.f, .28);
    float3 color2 = float3(.03, .02, .2);

    float3 c1 = rgb2hsv(1.f - color1);
    float3 c2 = rgb2hsv(1.f - color2);
    return 1.0f - hsv2rgb(lerp(c1, c2, x));
}

//------------------------------------------------------------------
float sdTorus(float3 p, float2 t)
{
    return length(float2(length(p.xz) - t.x, p.y)) - t.y;
}

float sdPlane(float3 p)
{
    return p.y;
}

//------------------------------------------------------------------

float2 opU( float2 d1, float2 d2 ) {
	return (d1.x<d2.x) ? d1 : d2;
}

float2 opD(float2 d1, float2 d2) {
    return (d1.x>=d2.x) ? d1 : d2;
}

// exponential smooth min (K = 32):
float smin(float a, float b, float k)
{
    float res = exp2((-k) * a) + exp2((-k) * b);
    return (-log2(res)) / k;
}

// The boundary function
float f(float3 pos)
{
    return float(frac((pos.x * 3.66) + 0.25f) > 0.5f);
}

// This function returns the negative distance, which we then negate after the fact.
// We do this because the important difference is that the `map` function returns the
// exterior difference *without the cutting plane*.
float2 interiorMap(float3 pos)
{
    float r1 = 0.4;
    float r2 = 0.19;
    float k = 16.0f;
    float3 param = (pos - float3(r1 + (r2 * 0.8), r1 + r2, 0.0f)).yzx;
    float2 param_1 = float2(r1, r2);
    float3 param_2 = pos - float3((-r1) - (r2 * 0.8), r1 + r2, 0.0f);
    float2 param_3 = float2(r1, r2);
    float param_4 = sdTorus(param, param_1);
    float param_5 = sdTorus(param_2, param_3);
    float param_6 = k;
    float3 param_7 = pos;
    return float2(-smin(param_4, param_5, param_6), f(param_7));
}

float2 map(float3 pos, float cuttingPlane)
{
    float2 res = float2(-interiorMap(pos).x, 2.0f);
    res = opD(res, float2(-sdPlane(pos.xzx - cuttingPlane.xxx), 3.0f));
    return res;
}

float2 castRay(float3 ro, float3 rd, float cuttingPlane)
{
    float2 res = (-1.0f).xx;

    float tmin = 1.0f;
    float tmax = 20.0f;
    
    // raytrace floor plane
    float tp1 = (0.0f - ro.y) / rd.y;
    if (tp1 > 0.0f)
    {
        tmax = min(tmax, tp1);
        res = float2(tp1, 1.0f);
    }

    // raymarch primitives
    float2 tb = iBox(ro - float3(0.0f, 0.6, 0.0f), rd, float3(1.2, 0.6, 0.6));
    if (tb.x < tb.y && tb.y > 0.f && tb.x < tmax) {
        tmin = max(tb.x, tmin);
        tmax = min(tb.y, tmax);

        float t = tmin;
        for (int i = 0; (i < 80) && (t < tmax); i++) {
            float2 h = map(ro + (rd * t), cuttingPlane);
            if (abs(h.x) < (0.00001 * t)) {
                res = float2(t, h.y);
                break;
            }
            t += h.x;
        }
    }
    return res;
}

// https://iquilezles.org/articles/normalsSDF
float3 calcNormal(float3 pos, float cuttingPlane)
{
    // inspired by tdhooper and klems - a way to prevent the compiler from inlining map() 4 times
    float3 n = 0.0f.xxx;
    for (int i = 0; i < 4; i++)
    {
        float3 e = ((float3(float(((i + 3) >> 1) & 1), float((i >> 1) & 1), float(i & 1)) * 2.0f) - 1.0f.xxx) * 0.577300012111663818359375f;
        float3 param = pos + (e * 0.0005000000237487256526947021484375f);
        float param_1 = cuttingPlane;
        n += (e * map(param, param_1).x);
    }
    return normalize(n);
}

float3 colorscale(float x)
{
    return lerp(float3(1.0f, 1.0f, 0.28), float3(0.02999999932944774627685546875f, 0.0199999995529651641845703125f, 0.20000000298023223876953125f), x.xxx);
}

float calcAO(float3 pos, float3 nor, float cuttingPlane)
{
    float occ = 0.0f;
    float sca = 1.0f;
    for (int i = 0; i < 5; i++)
    {
        float hr = 0.00999999977648258209228515625f + ((0.119999997317790985107421875f * float(i)) / 4.0f);
        float3 aopos = (nor * hr) + pos;
        float3 param = aopos;
        float param_1 = cuttingPlane;
        float dd = map(param, param_1).x;
        occ += ((-(dd - hr)) * sca);
        sca *= 0.949999988079071044921875f;
    }
    return clamp(1.0f - (3.0f * occ), 0.0f, 1.0f) * (0.5f + (0.5f * nor.y));
}

// https://iquilezles.org/articles/rmshadows
float calcSoftshadow(float3 ro, float3 rd, float mint, float tmax, float cuttingPlane)
{
    // This affects the shadowing, I think.
    float maxHei = 2.2;
    float tp = (maxHei - ro.y) / rd.y;
    if (tp > 0.0f)
    {
        tmax = min(tmax, tp);
    }
    float res = 1.0f;
    float t = mint;
    for (int i = 0; i < 16; i++)
    {
        float3 param = ro + (rd * t);
        float param_1 = cuttingPlane;
        float h = map(param, param_1).x;
        float s = clamp((4.0f * h) / t, 0.0f, 1.0f);
        res = min(res, (s * s) * (3.0f - (2.0f * s)));
        t += clamp(h, 0.0199999995529651641845703125f, 0.100000001490116119384765625f);
        if ((res < 0.004999999888241291046142578125f) || (t > tmax))
        {
            break;
        }
    }
    return clamp(res, 0.0f, 1.0f);
}


float3 render(float3 ro, float3 rd, float cuttingPlane, inout bool isInterior, inout float3 pos)
{
    float3 col = 0.9;
    float2 res = castRay(ro, rd, cuttingPlane);
    float t = res.x;
    float m = res.y;
    if (m > 0.5f) {
        pos = ro + (rd * t);
        float3 nor = (m < 1.5) ? float3(0, 1, 0) : calcNormal(pos, cuttingPlane);

        // material
        if (m > 2.5f) {
            isInterior = true;
        } else if (m > 1.5f) {
                col = colorscale(f(pos));
        } else {
            isInterior = false;
        }

        // lighting
        float occ = 0.5 + 0.5 * calcAO(pos, nor, cuttingPlane);
		float3 lig = normalize(float3(-0.2, 0.4, -0.3));
        float3 hal = normalize(lig - rd);
		float amb = sqrt(clamp(0.5 + 0.5 * nor.y, 0.0, 1.0));
        float dif = clamp(dot(nor, lig), 0.0, 1.0);

        dif *= 0.2 + 0.8 * calcSoftshadow(pos, lig, 0.2, 2.0, cuttingPlane);

		float spe = pow(clamp(dot(nor, hal), 0.0, 1.0), 16.0)*
                    dif * (0.04 + 0.96 * pow(clamp(1.0 + dot(hal, rd), 0.0, 1.0), 5.0));

        float3 lin = 0.0f;
        lin += 0.25f * dif;
        lin += (1.0f * amb) * occ;
        col *= lin;
        col += 6.0f * spe;
    }
    return col;
}


// --------------------------------------
// oldschool rand() from Visual Studio
// --------------------------------------
static int seed;
void srand(int s) {
    seed = s;
}

int rand() {
    seed = (seed * 214013) + 2531011;
    return (seed >> 16) & 32767;
}
// --------------------------------------

// --------------------------------------
// hash to initialize the random sequence (copied from Hugo Elias)
// --------------------------------------
int hash(int n)
{
    n = (n << 13) ^ n;
    return (n * (((n * n) * 15731) + 789221)) + 1376312589;
}

// --------------------------------------

float3 randomOnSphere()
{
    float theta = (6.283185 / 32767.0) * float(rand());
    float u = (2.0 / 32767.0) * float(rand()) - 1.0;
    return float3(float2(cos(theta), sin(theta)) * sqrt(max(0.0f, 1.0f - (u * u))), u);
}

// WoS! Walk-on-spheres. This the heart of the whole thing. Remarkably concise.
float march(float3 p)
{
    float2 h = 0.0f.xx;
    for (int i = 0; i < 32; i++) {
        float3 param = p;
        h = interiorMap(param);
        if (h.x < 0.001) {
            break;
        }
        p = p + h.x * randomOnSphere();
    }
    return h.y;
}


GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record)) {
    uint2 pixelID = DispatchRaysIndex().xy;
    uint2 fbSize = DispatchRaysDimensions().xy;
    float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);
    RayDesc rayDesc;
    rayDesc.Origin = record.camera.pos;
    rayDesc.Direction =
        normalize(record.camera.dir_00 + screen.x * record.camera.dir_du + screen.y * record.camera.dir_dv);
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 10000.0;

    float2 fragCoord = pixelID;
    float2 iResolution = fbSize;

    float cuttingPlane = 0.6 * cos(record.iTime);
    
    // init randoms
    int2 q = int2(fragCoord);
    srand(hash(q.x + hash(q.y + hash(record.iFrame))));
    
    // render
    bool isInterior = false;
    float3 pos;
    float3 col = render(rayDesc.Origin, rayDesc.Direction, cuttingPlane, isInterior, pos);
    if (isInterior)
    {
        float sum = 0.0f;
        for (int i = 0; i < 100; i++) {
            sum += march(pos);
        }
        sum /= 100.0f;
        col = hsvColorscale(sum);
    }
    bool redraw = true;

    col = pow(col, 0.4545);

    const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
    gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(float4(col, 1.0f)));
}
