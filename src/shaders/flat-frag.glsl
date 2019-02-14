#version 300 es
precision highp float;

uniform vec3 u_Eye, u_Ref, u_Up;
uniform vec2 u_Dimensions;
uniform float u_Time;

in vec2 fs_Pos;
out vec4 out_Col;

float EPSILON = 1e-6;
float STEPSIZE = 0.5f;
float MIN_MARCH_LEN = 1e-2;
float MAX_MARCH_LEN = 1e2;
int   MAX_MARCH_ITER = 64;

vec3 lightVec = normalize(vec3(2, 0.7, 0.0));
vec3 lightCol = vec3(1.0, 1.0, 1.15);
vec3 skyLitCol = vec3(0.16, 0.20, 0.28) / 2.0;
vec3 indLitCol = vec3(0.3, 0.28, 0.2) / 2.0;
vec3 lanternPos = vec3(0.0, 9.0, 0.0);
vec3 lanternCol = vec3(1.0, 0.0, 0.0);
float ambientLit = 0.05;
vec3 ambientCol = vec3(1);

// Noise
///////////////////////////////////////////////////////
vec3 c_seed = vec3(0);
float PI = 3.14159265;
float PI_2 = 6.2831853;
float random1( vec2 p , vec2 seed) { return fract(sin(dot(p + seed, vec2(127.1, 311.7))) * 43758.5453); }
float random1( vec3 p , vec3 seed) { return fract(sin(dot(p + seed, vec3(987.654, 123.456, 531.975))) * 85734.3545); }
vec2 random2( vec2 p , vec2 seed) { return fract(sin(vec2(dot(p + seed, vec2(311.7, 127.1)), dot(p + seed, vec2(269.5, 183.3)))) * 85734.3545); }

float falloff(float t) {
  t = t * t * t * (t * (t * 6. - 15.) + 10.);
  return t;
}

vec2 randGrad(vec2 p, vec2 seed) {
  float randDeg = random1(p, seed) * PI_2;
  return vec2(cos(randDeg), sin(randDeg));
}

float PerlinNoise(vec2 p, float s) {
    p /= s;
    vec2 pCell = floor(p);
    p -= pCell;
    float dotGrad00 = dot(randGrad(pCell + vec2(0., 0.), c_seed.xz), p - vec2(0., 0.));
    float dotGrad01 = dot(randGrad(pCell + vec2(0., 1.), c_seed.xz), p - vec2(0., 1.));
    float dotGrad10 = dot(randGrad(pCell + vec2(1., 0.), c_seed.xz), p - vec2(1., 0.));
    float dotGrad11 = dot(randGrad(pCell + vec2(1., 1.), c_seed.xz), p - vec2(1., 1.));

    return mix(mix(dotGrad00, dotGrad10, falloff(p.x)), mix(dotGrad01, dotGrad11, falloff(p.x)), falloff(p.y)) * .5 + .5;
}

float FBMPerlin(vec2 p, float minCell, int maxIter) {
  float sum = 0.;
  float noise = 0.;
  for (int i = 0; i < maxIter; i++) {
      noise += PerlinNoise(p, minCell * pow(2., float(i))) / pow(2., float(maxIter - i));
      sum += 1. / pow(2., float(maxIter - i));
  }
  noise /= sum;
  return noise;
}

vec2 sampleInCell(vec2 p) {
  return random2(p, c_seed.xz) + p;
}

float worleyNoise(vec2 p, float s) {
    // Which cell p belongs to
    p /= s;
    vec2 pCell = floor(p);

    float min_dist = 1.;
    for (int i = -1; i <= 1 ; i++) {
        for (int j = -1; j <= 1; j++) {
          vec2 sampleNoise = sampleInCell(pCell + vec2(i, j));
          min_dist = min(min_dist, distance(sampleNoise, p));
        }
    }
    float noise = clamp(min_dist, 0., 1.);
    return noise;
}

float worleyNoiseOutline(vec2 p, float s) {
    // Which cell p belongs to
    p /= s;
    vec2 pCell = floor(p);

    float min_dist = 1.;
    float sec_dist = 1.;
    for (int i = -1; i <= 1 ; i++) {
        for (int j = -1; j <= 1; j++) {
          vec2 sampleNoise = sampleInCell(pCell + vec2(i, j));
          float dist = distance(sampleNoise, p);
          if (dist < min_dist) { sec_dist = min_dist; min_dist = dist; }
          else if (dist < sec_dist) { sec_dist = dist; }
        }
    }
    float noise = clamp(sec_dist - min_dist, 0., 1.);
    return noise;
}

// Transform Function
///////////////////////////////////////////////////////
vec3 rotX(vec3 p, float x) {
  x = radians(x);
  float c = cos(x);
  float s = sin(x);
  return vec3(p.x, c*p.y-s*p.z, s*p.y+c*p.z);
}

vec3 rotY(vec3 p, float y) {
  y = radians(y);
  float c = cos(y);
  float s = sin(y);
  return vec3(c*p.x+s*p.z, p.y, -s*p.x+c*p.z);
}

vec3 rotZ(vec3 p, float z) {
  z = radians(z);
  float c = cos(z);
  float s = sin(z);
  return vec3(c*p.x-s*p.y, s*p.x+c*p.y, p.z);
}

// SDF
///////////////////////////////////////////////////////
float opExtrussion(vec3 p, float sdf, float h )
{
  vec2 w = vec2( sdf, abs(p.z) - h );
  return min(max(w.x,w.y),0.0) + length(max(w,0.0));
}

float opSubtraction( float d1, float d2 ) { return max(-d1,d2); }
float opIntersection( float d1, float d2 ) { return max(d1,d2); }

vec3 opCheapBendX(vec3 p, float w)
{
  float c = cos(w*p.y);
  float s = sin(w*p.y);
  mat2  m = mat2(c, s, -s, c);
  return vec3(p.x, m * p.yz);
}

float smin( float a, float b, float k )
{
    float res = exp2( -k*a ) + exp2( -k*b );
    return -log2( res )/k;
}

float sdBox( vec3 p, vec3 b )
{
  vec3 d = abs(p) - b;
  return length(max(d,0.0))
         + min(max(d.x,max(d.y,d.z)),0.0);
}

float sdCylinder( vec3 p, vec2 h )
{
  vec2 d = abs(vec2(length(p.xz),p.y)) - h;
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float sdTriangleIsosceles( vec2 p, vec2 q )
{
    p.x = abs(p.x);
    
    vec2 a = p - q*clamp( dot(p,q)/dot(q,q), 0.0, 1.0 );
    vec2 b = p - q*vec2( clamp( p.x/q.x, 0.0, 1.0 ), 1.0 );
    float s = -sign( q.y );
    vec2 d = min( vec2( dot(a,a), s*(p.x*q.y-p.y*q.x) ),
                  vec2( dot(b,b), s*(p.y-q.y)  ));

    return -sqrt(d.x)*sign(d.y);
}

float dot2( vec2 v ) { return dot(v,v); }
float sdCappedCone( vec3 p, float h, float r1, float r2 )
{
  vec2 q = vec2( length(p.xz), p.y );
  
  vec2 k1 = vec2(r2,h);
  vec2 k2 = vec2(r2-r1,2.0*h);
  vec2 ca = vec2(q.x-min(q.x,(q.y < 0.0)?r1:r2), abs(q.y)-h);
  vec2 cb = q - k1 + k2*clamp( dot(k1-q,k2)/dot2(k2), 0.0, 1.0 );
  float s = (cb.x < 0.0 && ca.y < 0.0) ? -1.0 : 1.0;
  return s*sqrt( min(dot2(ca),dot2(cb)) );
}

float sdRoundCone(vec3 p, float h, float r1, float r2)
{
  vec2 q = vec2( length(p.xz), p.y );
  
  float b = (r1-r2)/h;
  float a = sqrt(1.0-b*b);
  float k = dot(q,vec2(-b,a));
  
  if( k < 0.0 ) return length(q) - r1;
  if( k > a*h ) return length(q-vec2(0.0,h)) - r2;
      
  return dot(q, vec2(a,b) ) - r1;
}

float sdHexPrism( vec3 p, vec2 h )
{
  const vec3 k = vec3(-0.8660254, 0.5, 0.57735);
  p = abs(p);
  p.xy -= 2.0*min(dot(k.xy, p.xy), 0.0)*k.xy;
  vec2 d = vec2(length(p.xy-vec2(clamp(p.x,-k.z*h.x,k.z*h.x), h.x))*sign(p.y-h.x), p.z-h.y );
  return min(max(d.x,d.y),0.0) + length(max(d,0.0));
}

float sdSphere( vec3 p, float s )
{
  return length(p)-s;
}

// UV Map
///////////////////////////////////////////////////////
vec3 uvCylinder(vec3 p, vec2 h) {
  if (p.y > -h.y && p.y < h.y) {
    return vec3(degrees(atan(p.z, p.x)) / 180.0, p.y / h.y, 0.5);
  }
  else {
    return vec3(p.x / h.x, p.z / h.x, 0.0);
  }
}

vec3 uvHexPrism( vec3 p, vec2 h ) {
  if (p.z > -h.y && p.z < h.y) {
    float a = degrees(atan(p.y, p.x));
    vec2 _p = rotZ(vec3(p.xy, 0.0), -floor((a+120.0)/60.0) * 60.0).xy;
    return vec3(1.732051 * _p.x / _p.y, p.z / h.y, 0.5);
  }
  else {
    return vec3(p.x/h.x, p.y/h.x, 0.0);
  }
}

// Map Function
////////////////////////////////////////////////////////
float mapCeiling(vec3 pos, out vec3 uvflag) {
  float r = 0.3;
  float l = 4.5;
  float r0 = 0.25;
  float l0 = 4.0;
  float d = MAX_MARCH_LEN;
  float halfWidth = 4.0;
  float angle = degrees(asin(clamp(halfWidth / l * 0.866025, -1.0, 1.0)));

  vec3 po = pos;
  vec3 pCol;
  int flag = -1;
  for (int i = 0; i < 6; i++) {
    vec3 p = rotY(po, float(i) * 60.0);
    vec3 p0 = rotX(p, -angle) + vec3(0.0, l, 0.0);
    vec3 p1 = rotZ(p0 + vec3(0.0, -l, r), 180.0);
    vec3 p2 = p1 - vec3(0.0, 0.0, r + 0.2);
    float d2 = opExtrussion(p2, sdTriangleIsosceles(p2.xy, vec2(halfWidth, l * 2.0)), 0.2);
    float dis = abs(cos(p2.x * 5.0)) * 0.1;
    if (p2.z < 0.0) { d2 = d2 - dis; }
    if (d > d2) { d = d2; flag = 1; }

    float len = sqrt(halfWidth * halfWidth * 0.25 + l * l);
    vec3 p3 = rotX(rotY(p, 30.0), -angle * 1.1) + vec3(0.0, len * 1.9, 0.0);
    float d3 = sdRoundCone(p3, len * 1.9, 0.8 * r, r);
    d3 = smin(d3, sdRoundCone(opCheapBendX(rotX(p3 + vec3(0.0, r, 0.0), -30.0), 0.5) + vec3(0.0, 2.0, 0.0), 2.0, 0.0, 0.8 * r), 15.0);
    if (d > d3) { d = d3; flag = 2; }

    vec3 p4 = rotY( po + vec3(0.0, 9.0, 0.0), 30.0 + float(i) * 60.0) + vec3(0.0, 0.0, 6.0);
    float d4 = sdCylinder(p4, vec2(r0, l0));
    if (d > d4) { d=d4; pCol=p4; flag=3; }
  }

  switch(flag) {
  case -1: break;
  case 0: break;
  case 1: uvflag = vec3(pos.x, pos.y, 1.0); break;
  case 2: uvflag = vec3(pos.x, pos.y, 2.0); break;
  case 3: uvflag = uvCylinder(pCol, vec2(r, l)) + vec3(0.0, 0.0, 3.0); break;
  }
  return d;
}

float mapBase(vec3 pos, out vec3 uvflag) {
  int flag = -1;
  float d = MAX_MARCH_LEN;

  vec3 p0 = rotX(pos, 90.0);
  float r0 = 6.0;
  float l0 = 1.0;
  float d0 = sdHexPrism(p0, vec2(r0, l0));
  if (d > d0) { d=d0; flag=0; }

  switch(flag) {
  case -1: break;
  case 0: uvflag = uvHexPrism(p0, vec2(r0, l0)) + vec3(0.0, 0.0, 4.0);  break;
  }
  return d;
}

float mapWater(vec3 pos, out vec3 uvflag) {
  vec2 q = vec2(FBMPerlin(pos.xz + vec2(u_Time * 0.05,0.), 0.5, 4) + u_Time * 0.01,
                FBMPerlin(pos.xz + vec2(20., 5.), 0.5, 4));
  float n = pow(FBMPerlin(pos.xz + 5.0 * q, 0.5, 4), 2.0);
  uvflag = vec3(pos.x, pos.z, 5.0 + n);
  return pos.y - n * 0.7;
}

float map(vec3 pos, bool lightVisible, out vec3 uvflag)
{
  vec3 p = pos;
  vec3 uv;
  float d = MAX_MARCH_LEN;
  float d0 = mapCeiling(p - vec3(0.0, 13.0, 0.0), uv);
  float ds = sdCylinder(p - vec3(0.0, 17.0, 0.0), vec2(10.0, 7.0));
  d0 = opSubtraction(ds, d0);
  if(d > d0) { d=d0; uvflag=uv; }
  float d1 = opIntersection(ds, mapCeiling((p - vec3(0.0, 15.5, 0.0)) / 0.7, uv) * 0.7);
  if(d > d1) { d=d1; uvflag=uv; }
  float d2 = mapBase(p, uv);
  if(d > d2) { d=d2; uvflag=uv; }
  float d3 = mapWater(pos + vec3(0.0, 1.0, 0.0), uv);
  if(d > d3) { d=d3; uvflag=uv; }
  if (lightVisible) {
    float d4 = sdSphere(pos - lanternPos, 1.0);
    if(d > d4) { d=d4; uvflag=vec3(0.0, 0.0, 7.0); }
  }
  return d;
}

// Normal Calculation
// Reference from http://iquilezles.org/www/articles/normalsSDF/normalsSDF.htm
////////////////////////////////////////////////////////
vec3 calcNormal(vec3 pos, out vec3 uvflag)
{
    const float ep = 0.0001;
    vec2 e = vec2(1.0,-1.0)*0.5773;
    return normalize( e.xyy*map( pos + e.xyy*ep, false, uvflag) + 
					            e.yyx*map( pos + e.yyx*ep, false, uvflag) + 
					            e.yxy*map( pos + e.yxy*ep, false, uvflag) + 
					            e.xxx*map( pos + e.xxx*ep, false, uvflag) );
}

// Ray Marching Funtion
// Reference from https://www.shadertoy.com/view/4lyfzw
////////////////////////////////////////////////////////
bool rayMarch(in vec3 ori, in vec3 dir, out vec3 uvflag, out vec3 p, out float d) {
  float t = 0.0;
  for (int i = 0; i < MAX_MARCH_ITER; i++) {
    p = ori + t * dir;
    d = map(p, true, uvflag);
    if (t > MAX_MARCH_LEN || d < MIN_MARCH_LEN * t) {
      break;
    }
    t += d * STEPSIZE;
  }
  return d < MIN_MARCH_LEN * t;
}

// Visual Effect
// Reference from http://iquilezles.org/www/articles/rmshadows/rmshadows.htm
////////////////////////////////////////////////////////
float softshadow(vec3 ro, vec3 rd, float mint, float maxt, float k )
{
  float res = 1.0;
  vec3 uv;
  for( float t=mint; t < maxt; )
  {
    float h = map(ro + rd*t, false, uv);
    if( h < MIN_MARCH_LEN * t )
      return 0.0;
    res = min( res, k*h/t );
    t += h * STEPSIZE;
  }
  return res;
}

// Render
///////////////////////////////////////////////////////
vec3 render(vec3 pos, vec3 uvflag, out vec3 norm) {
  vec3 color;
  if (uvflag.z < -0.0 + EPSILON) {
    return vec3(0.0);
  }
  else if(uvflag.z < 1.0 + EPSILON) {
    color = vec3(69.0, 88.0, 82.0) / 255.0;
  }
  else if(uvflag.z < 2.0 + EPSILON) {
    color = vec3(98.0, 113.0, 102.0) / 255.0;
  }
  else if(uvflag.z < 3.5 + EPSILON) {
    color = vec3(174.0, 14.0, 14.0) / 255.0;
  }
  else if(uvflag.z < 4.0 + EPSILON) {
    vec2 q = vec2(FBMPerlin(uvflag.xy + vec2(0.,0.), 0.05, 4),
                  FBMPerlin(uvflag.xy + vec2(2.,5.), 0.05, 4));
    float n = FBMPerlin(uvflag.xy + 5.0 * q, 0.05, 4);
    vec3 c0 = vec3(200.0, 200.0, 216.0) / 300.0;
    vec3 c1 = vec3(210.0, 202.0, 191.0) / 330.0;
    vec3 c2 = vec3(230.0, 238.0, 250.0) / 300.0;
    vec3 col;
    if (n < 0.3) { col = mix(c0, c1, n / 0.3); }
    else if (n < 0.7) { col = mix(c1, c2, (n - 0.3) / 0.4); }
    else { col = mix( c2, c0, (n - 0.7) / 0.3); }
    color = col;
  }
  else if(uvflag.z < 4.5 + EPSILON) {
    vec3 c0 = vec3(161.0, 164.0, 154.0) / 550.0;
    vec3 c1 = vec3(200.0, 200.0, 216.0) / 300.0;
    vec3 col = c1;
    if (abs(uvflag.y) < 0.8) {
      float n = smoothstep(0.0, 1.0, worleyNoiseOutline(uvflag.xy * vec2(3.0, 1.0), 0.4)) * 0.5 + 0.5;
      col = mix(c0, c1, n);
      if (abs(uvflag.y) > 0.5) {
        col *= mix(1.0, 0.7, (abs(uvflag.y) - 0.5) / 0.3);
      }
    }
    color = col;
  }
  else if(uvflag.z < 6.0 + EPSILON) {
    color = mix(vec3(3., 7., 15.) / 255., vec3(12., 19., 36.) / 255., uvflag.z - 5.0);
  }
  else if(uvflag.z < 7.0 + EPSILON) {
    color = lanternCol;
    return color;
  }

  vec3 uv;
  norm = calcNormal(pos, uv);
  // ambient light
  vec3 assembleCol = ambientLit * ambientCol;

  // sun light
  float sunTerm = clamp(dot(norm, lightVec), 0.0, 1.0);
  vec3 shadowRay = lightVec;
  float shadowTerm = softshadow(pos, shadowRay, MIN_MARCH_LEN + EPSILON, MAX_MARCH_LEN, 16.0 );
  assembleCol += pow(vec3(shadowTerm), vec3(1.5, 1.2, 1.0)) * sunTerm * lightCol;

  // sky light
  float skyTerm = clamp(0.5 + 0.5 * norm.y, 0.0, 1.0);
  assembleCol += skyTerm * skyLitCol;

  // indirect light
  float indTerm = clamp(dot(norm, lightVec * vec3(-1.0, 0.0, -1.0)), 0.0, 1.0);
  assembleCol += indTerm * indLitCol;

  // lantern
  vec3 lanternRay = lanternPos - pos;
  float le = length(lanternRay);
  float lanternTerm = pow(1.0 / le, 2.0) * smoothstep(-0.8, 1.0, dot(norm, normalize(lanternRay)));
  lanternRay = normalize(lanternRay);
  float lanternShadowTerm = softshadow(pos, lanternRay, MIN_MARCH_LEN + EPSILON, le - EPSILON, 32.0 );
  assembleCol += lanternShadowTerm * clamp(lanternTerm * 200.0, 0.0, 1.0) * lanternCol;

  return assembleCol * color;
}

// Ray Casting Funtion
///////////////////////////////////////////////////////
vec3 rayCast(vec2 p, vec3 eye, vec3 ref, vec3 up) {
  vec3 focusVector = ref - eye;
  vec3 Right = normalize(cross(focusVector, up)) * u_Dimensions.x / u_Dimensions.y;
  float len = length(focusVector) * 0.7;
  return normalize(focusVector + len * p.x * Right + len * p.y * up);
}

// Schlick's Approximation for Fresnel Reflection
///////////////////////////////////////////////////////
float fresnelReflect(float n1, float n2, float cosTheta) {
  float R0 = pow((n1 - n2) / (n1 + n2), 2.0);
  return R0 + (1.0 - R0) * pow(1.0 - cosTheta, 2.5); // 5.0 for origin equation
}

// Sky Ramp
///////////////////////////////////////////////////////
vec3 skyRamp(vec3 dir) {
  float cosUp = dir.y;
  vec3 col1 = vec3(49., 58., 83.)/255.;
  vec3 col2 = vec3(20., 34., 77.)/255.;
  vec3 col3 = vec3(12., 19., 36.)/255.;
  vec3 col4 = vec3(3., 7., 15.)/255.;

  vec3 col;
  if (cosUp < 0.0) {
    col = col1;
  }
  else if (cosUp < 0.2) {
    col = mix(col1, col2, cosUp / 0.2);
  }
  else if (cosUp < 0.5) {
    col = mix(col2, col3, (cosUp - 0.2)/0.3);
  }
  else {
    col = mix(col3, col4, (cosUp - 0.5)/0.5);
  }

  float dotMoon = dot(dir, lightVec);
  if (dotMoon > 0.988) {
    col = vec3(1);
  }
  else {
    col = mix(col, vec3(1), smoothstep(0.97, 0.995, dotMoon));
  }

  return col;
}

// fog
///////////////////////////////////////////////////////
vec3 applyFog(vec3 pos, vec3 rgb, float distance )
{
    float fogAmount = 1.0 - exp( -distance * 3.0);
    vec3  fogColor  = vec3(53., 68., 90.)/255.;
    return mix(rgb, fogColor, fogAmount * (1.0 - smoothstep(0.0, 35.0, pos.y)));
}

void main() {
  float speed = 0.01;
  float r = 14.0 + 10.0 * smoothstep(-0.5, 0.5, -sin(speed * u_Time));
  vec3 eye = vec3(r * sin(speed * u_Time), 4.8, r * cos(speed * u_Time));
  vec3 ref = vec3(0.0, 2.8, 0.0);
  vec3 forward = normalize(ref - eye);
  vec3 up = normalize(vec3(0.0, 1.0, 0.0)-forward.y * forward);

  vec3 dir = rayCast(fs_Pos, eye, ref, up);
  vec3 skyCol = skyRamp(dir);
  vec3 uvflag;
  vec3 pos;
  vec3 col;
  vec3 norm;
  float d;
  if(rayMarch(eye, dir, uvflag, pos, d)) {
    col = render(pos, uvflag, norm);
    if (uvflag.z > 4.5 + EPSILON && uvflag.z < 6.0 + EPSILON) {
      dir = -2.0 * norm * dot(norm, dir) + dir;
      vec3 ori = pos;
      float R = fresnelReflect(1.0, 1.33, dot(norm, dir));
      float dd;
      if (rayMarch(ori, dir, uvflag, pos, dd)) {
        col = mix(col, render(pos, uvflag, norm), R);
      }
      else {
        col = mix(col, skyRamp(dir), R);
      }
    }
  }
  else {
    d = MAX_MARCH_LEN;
    col = skyCol;
  }
  col = applyFog(eye + d * dir, col, d);
  out_Col = vec4(col, 1.0);
}
