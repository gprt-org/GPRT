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

#include "rng.h"

[[vk::push_constant]] PushConstants pc;

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
    return float(frac(((pos.z + pos.x) * 3.0) + 0.25f) > 0.6f);
}

float dot2( in float3 v ) { return dot(v,v); }
float maxcomp( in float2 v ) { return max(v.x,v.y); }

float3 closestTriangle( in float3 v0, in float3 v1, in float3 v2, in float3 p )
{
    float3 v10 = v1 - v0; float3 p0 = p - v0;
    float3 v21 = v2 - v1; float3 p1 = p - v1;
    float3 v02 = v0 - v2; float3 p2 = p - v2;
    float3 nor = cross( v10, v02 );

#if 0
    // method 1, in 3D space
    if( dot(cross(v10,nor),p0)<0.0 ) return v0 + v10*clamp( dot(p0,v10)/dot2(v10), 0.0, 1.0 );
    if( dot(cross(v21,nor),p1)<0.0 ) return v1 + v21*clamp( dot(p1,v21)/dot2(v21), 0.0, 1.0 );
    if( dot(cross(v02,nor),p2)<0.0 ) return v2 + v02*clamp( dot(p2,v02)/dot2(v02), 0.0, 1.0 );
    return p - nor*dot(nor,p0)/dot2(nor);
    
#else    
    // method 2, in barycentric space
    float3  q = cross( nor, p0 );
    float d = 1.0/dot2(nor);
    float u = d*dot( q, v02 );
    float v = d*dot( q, v10 );
    float w = 1.0-u-v;
    
         if( u<0.0 ) { w = clamp( dot(p2,v02)/dot2(v02), 0.0, 1.0 ); u = 0.0; v = 1.0-w; }
    else if( v<0.0 ) { u = clamp( dot(p0,v10)/dot2(v10), 0.0, 1.0 ); v = 0.0; w = 1.0-u; }
	else if( w<0.0 ) { v = clamp( dot(p1,v21)/dot2(v21), 0.0, 1.0 ); w = 0.0; u = 1.0-v; }
    
    return u*v1 + v*v2 + w*v0;
#endif    
}


float udBox2(float3 aabbMin, float3 aabbMax, float3 p)
{
    float3 d = max(max(aabbMin - p, float3(0.f, 0.f, 0.f)), p - aabbMax);
    return dot(d,d);
}

float udBox(float3 aabbMin, float3 aabbMax, float3 p)
{
    return sqrt(udBox2(aabbMin, aabbMax, p));
}

// This function returns the negative distance, which we then negate after the fact.
// We do this because the important difference is that the `map` function returns the
// exterior difference *without the cutting plane*.
float2 interiorMap(in LBVHData lbvh, float3 pos)
{
    // brute force thing for now
    // float3 aabbMin = gprt::load<float3>(lbvh.aabbs, 0);
    // float3 aabbMax = gprt::load<float3>(lbvh.aabbs, 1);

    // float3 p = (pos - aabbMin) / (aabbMax - aabbMin);

    float dist = 1e38f;
    float s = -1.f;

    uint stack[32];
    uint stackPtr = 0;
    stackPtr ++;
    stack[stackPtr] = 0; // root
    while (stackPtr > 0)
    {
        bool gotoNext = false;
        int4 node = gprt::load<int4>(lbvh.nodes, stack[stackPtr]);
        stackPtr = stackPtr - 1;

        // while left and right contain children
        while (node.x != -1 && node.y != -1) {
            int4 children[2] = { 
                gprt::load<int4>(lbvh.nodes, node.x), 
                gprt::load<int4>(lbvh.nodes, node.y) 
            };

            float3 leftAABB[2] = {
                gprt::load<float3>(lbvh.aabbs, 2 * node.x + 0), 
                gprt::load<float3>(lbvh.aabbs, 2 * node.x + 1), 
            };

            float3 rightAABB[2] = {
                gprt::load<float3>(lbvh.aabbs, 2 * node.y + 0), 
                gprt::load<float3>(lbvh.aabbs, 2 * node.y + 1), 
            };

            float distL = udBox(leftAABB[0], leftAABB[1], pos); // distance(*((AABB*)(&children[0].bbox)),p);
            float distR = udBox(rightAABB[0], rightAABB[1], pos); //distance(*((AABB*)(&children[1].bbox)),p);

            if (distL < dist && distR < dist)
            {
                if (distL < distR)
                {
                    // if (lbvh.tmp) {
                    //     printf("Left closer, traversing %d, adding %d to stack, stack ptr %d\n", node.w, stackPtr);
                    // }
                    stackPtr++;
                    stack[stackPtr] = node.y;
                    node = children[0];
                }
                else
                {
                    stackPtr++;
                    stack[stackPtr] = node.x;
                    node = children[1];
                }
            }
            else if (distL < dist)
                node = children[0];
            else if (distR < dist)
                node = children[1];
            else
            {
                gotoNext = true;
                break;
            }
        }
        if (gotoNext) continue;

        // if (lbvh.tmp) {
        //     printf("Traversing %d, Stack ptr %d\n", node.w, stackPtr);
        // }
        
        // Traverse leaf
        int i = node.w;
        int3 tri = gprt::load<int3>(lbvh.triangles, i);
        float3 p0 = gprt::load<float3>(lbvh.positions, tri.x); 
        float3 p1 = gprt::load<float3>(lbvh.positions, tri.y); 
        float3 p2 = gprt::load<float3>(lbvh.positions, tri.z); 

        float3 pClosest = closestTriangle(p0, p1, p2, pos.xyz);
        
        float d = distance(pos, pClosest);
        if (d < dist) {
            dist = d;

            // compute sign
            float3 N = -cross(normalize(p1 - p0), normalize(p2 - p0));
            s = sign(dot(N, normalize(pClosest - pos)));
        }
    }
    return float2(-dist * s, f(pos));


    // for (uint i = 0; i < lbvh.numPrims; ++i) {
    //     int3 tri = gprt::load<int3>(lbvh.triangles, i);
    //     float3 p0 = .5f * gprt::load<float3>(lbvh.positions, tri.x).xzy + float3(0.f, 0.5f, .0f); 
    //     float3 p1 = .5f * gprt::load<float3>(lbvh.positions, tri.y).xzy + float3(0.f, 0.5f, .0f); 
    //     float3 p2 = .5f * gprt::load<float3>(lbvh.positions, tri.z).xzy + float3(0.f, 0.5f, .0f); 

    //     float3 pClosest = closestTriangle(p0, p1, p2, pos.xyz);
    //     float d = distance(pos, pClosest);

    //     if (d < dist) {
    //         dist = d;

    //         // compute sign
    //         float3 N = cross(normalize(p1 - p0), normalize(p2 - p0));
    //         s = sign(dot(N, normalize(pClosest - pos)));
    //     }
    // }
    // return float2(-dist * s, f(pos));


    // float r1 = 0.4;
    // float r2 = 0.19;
    // float k = 16.0f;
    // return float2(-smin(
    //     sdTorus((pos - float3(r1 + (r2 * 0.8), r1 + r2, 0.0f)).yzx, float2(r1, r2)), 
    //     sdTorus(pos - float3((-r1) - (r2 * 0.8), r1 + r2, 0.0f), float2(r1, r2)), k), 
    //     f(pos));
}

// x is the distance, y is the material type
float2 map(in LBVHData lbvh, float3 pos, float cuttingPlane)
{
    // 
    float2 res = float2(-interiorMap(lbvh, pos).x, 2.0f);

    // This clips the above result to the given plane
    res = opD(res, float2(-sdPlane(pos.xzx - cuttingPlane.xxx), 3.0f));
    return res;
}

float2 castRay(in LBVHData lbvh, float3 ro, float3 rd, float cuttingPlane)
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
    float3 aabbMin = gprt::load<float3>(lbvh.aabbs, 0);
    float3 aabbMax = gprt::load<float3>(lbvh.aabbs, 1);
    float2 tb = iBox(ro - float3(0.0f, 1.0, 0.0f), rd, float3(2.0, 1.0, 1.0));
    if (tb.x < tb.y && tb.y > 0.f && tb.x < tmax) {
        tmin = max(tb.x, tmin);
        tmax = min(tb.y, tmax);

        float t = tmin;
        for (int i = 0; (i < 80) && (t < tmax); i++) {
            float2 h = map(lbvh, ro + (rd * t), cuttingPlane);
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
float3 calcNormal(in LBVHData lbvh, float3 pos, float cuttingPlane)
{
    // inspired by tdhooper and klems - a way to prevent the compiler from inlining map() 4 times
    float3 n = 0.0f.xxx;
    for (int i = 0; i < 4; i++)
    {
        float3 e = ((float3(float(((i + 3) >> 1) & 1), float((i >> 1) & 1), float(i & 1)) * 2.0f) - 1.0f.xxx) * 0.577300012111663818359375f;
        float3 param = pos + (e * 0.0005000000237487256526947021484375f);
        float param_1 = cuttingPlane;
        n += (e * map(lbvh, param, param_1).x);
    }
    return normalize(n);
}

float3 colorscale(float x)
{
    return lerp(float3(1.0f, 1.0f, 0.28), float3(0.02999999932944774627685546875f, 0.0199999995529651641845703125f, 0.20000000298023223876953125f), x.xxx);
}

float calcAO(in LBVHData lbvh, float3 pos, float3 nor, float cuttingPlane)
{
    float occ = 0.0f;
    float sca = 1.0f;
    for (int i = 0; i < 5; i++)
    {
        float hr = 0.00999999977648258209228515625f + ((0.119999997317790985107421875f * float(i)) / 4.0f);
        float3 aopos = (nor * hr) + pos;
        float3 param = aopos;
        float param_1 = cuttingPlane;
        float dd = map(lbvh, param, param_1).x;
        occ += ((-(dd - hr)) * sca);
        sca *= 0.949999988079071044921875f;
    }
    return clamp(1.0f - (3.0f * occ), 0.0f, 1.0f) * (0.5f + (0.5f * nor.y));
}

// https://iquilezles.org/articles/rmshadows
float calcSoftshadow(in LBVHData lbvh, float3 ro, float3 rd, float mint, float tmax, float cuttingPlane)
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
        float h = map(lbvh, param, param_1).x;
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


float3 render(in LBVHData lbvh, float3 ro, float3 rd, float cuttingPlane, inout bool isInterior, inout float3 pos)
{
    float3 col = 0.9;
    float2 res = castRay(lbvh, ro, rd, cuttingPlane);
    float t = res.x;
    float m = res.y;
    if (m > 0.5f) {
        pos = ro + (rd * t);
        float3 nor = (m < 1.5) ? float3(0, 1, 0) : calcNormal(lbvh, pos, cuttingPlane);

        // material
        if (m > 2.5f) {
            isInterior = true;
        } else if (m > 1.5f) {
            col = colorscale(f(pos));
        } else {
            isInterior = false;
        }

        // lighting
        float occ = 0.5 + 0.5 * calcAO(lbvh, pos, nor, cuttingPlane);
		float3 lig = normalize(float3(-0.2, 0.4, -0.3));
        float3 hal = normalize(lig - rd);
		float amb = sqrt(clamp(0.5 + 0.5 * nor.y, 0.0, 1.0));
        float dif = clamp(dot(nor, lig), 0.0, 1.0);

        dif *= 0.2 + 0.8 * calcSoftshadow(lbvh, pos, lig, 0.2, 2.0, cuttingPlane);

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

float3 randomOnSphere(inout LCGRand rng)
{
    float theta = 6.283185 * lcg_randomf(rng);
    float u = 2.0 * lcg_randomf(rng) - 1.0;
    return float3(float2(cos(theta), sin(theta)) * sqrt(max(0.0f, 1.0f - (u * u))), u);
}

// WoS! Walk-on-spheres. This the heart of the whole thing. Remarkably concise.
float march(in LBVHData lbvh, inout LCGRand rng, float3 p)
{
    float2 h = 0.0f.xx;
    for (int i = 0; i < 8; i++) {
        float3 param = p;
        h = interiorMap(lbvh, param);
        h.x = abs(h.x);
        if (h.x < 0.001) {
            break;
        }
        p = p + h.x * randomOnSphere(rng);
    }
    return h.y;
}


GPRT_RAYGEN_PROGRAM(simpleRayGen, (RayGenData, record)) {
    uint2 pixelID = DispatchRaysIndex().xy;
    uint2 fbSize = DispatchRaysDimensions().xy;

    LCGRand rng = get_rng(pc.iFrame, pixelID, fbSize);

    // if (all(pixelID == uint2(fbSize / 2))) {
    //     // printf("seed %d\n", seed);
    // }
    
    // printf("ID %d %d Frame %d\n", pixelID.x, pixelID.y, pc.iFrame);

    LBVHData lbvh = record.lbvh;
    if (all(pixelID == (fbSize / 2))) lbvh.tmp = 1;
    else lbvh.tmp = 0;

    float2 screen = (float2(pixelID) + float2(.5f, .5f)) / float2(fbSize);
    RayDesc rayDesc;
    rayDesc.Origin = pc.camera.pos;
    rayDesc.Direction =
        normalize(pc.camera.dir_00 + screen.x * pc.camera.dir_du + screen.y * pc.camera.dir_dv);
    rayDesc.TMin = 0.0;
    rayDesc.TMax = 10000.0;

    float2 fragCoord = pixelID;
    float2 iResolution = fbSize;

    float cuttingPlane = pc.cuttingPlane;

    // render
    bool isInterior = false;
    float3 pos;
    float3 col = render(lbvh, rayDesc.Origin, rayDesc.Direction, cuttingPlane, isInterior, pos);
    if (isInterior)
    {
        float sum = 0.f;
        for (int i = 0; i < 1; ++i)
          sum += march(lbvh, rng, pos);
        sum /= 1.f;
        col = hsvColorscale(sum);
    }
    bool redraw = true;

    col = pow(col, 0.4545);

    int frame = pc.iFrame;
    const int fbOfs = pixelID.x + fbSize.x * pixelID.y;
    float4 prev = gprt::load<float4>(record.accumBuffer, fbOfs);
    float4 newCol = prev * (frame / (frame + 1.f)) + float4(col, 1.0f) * (1.f / (frame + 1.f));
    gprt::store<float4>(record.accumBuffer, fbOfs, newCol);

    gprt::store(record.frameBuffer, fbOfs, gprt::make_bgra(newCol));
}
