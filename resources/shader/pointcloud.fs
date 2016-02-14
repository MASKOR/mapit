#version 400
in mediump vec4 color_frag;
in highp vec3 viewnormal;
in highp vec3 tang;
in highp vec3 bitang;
in mediump vec3 viewPoint;
noperspective in highp vec4 Ap;
noperspective in highp vec4 Bp;
noperspective in highp vec4 Cp;
noperspective in highp vec4 Dp;
out vec4 fragColor;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectionmatrix;
uniform mediump mat4 projectioninvmatrix;
uniform highp vec4 viewport; //TODO: wz! and: do not use, incorrect for oculus!

/*
vec3 unproj(vec3 p, vec3 n)
{
    vec2 xy = gl_FragCoord.xyz/viewport.xy;
    vec4 v_screen = vec4(xy, 0.0, 1.0 );
    vec4 v_homo = inverse(gl_ProjectionMatrix) * 2.0*(v_screen-vec4(0.5));
    vec3 v_eye = v_homo.xyz / v_homo.w; //transfer from homogeneous coordinates
    return v_eye;
}*/

void main(void)
{
/*    vec3 normal = normalize(viewnormal);
    if(normal.z<0.0)
    {
        normal = -normal;
    }*/
/*    //normal.y = -normal.y;

    // project in orthogonal coordinate system. This fits good for points near screen center and is bad at edges.

    vec2 pos = (gl_PointCoord.xy - vec2(0.5))*2.0;
    pos = (-pos.x*normalize(tang.xy))+pos.y*normalize(bitang.xy);
    pos *= pos;

    //float radSmall = min(1.0,viewnormal.z/length(viewnormal.xy));
    float radSmall = min(1.0,dot(normalize(viewPoint), viewnormal));

    //float dist = dot(pos, 1.0 / (radii * radii));
    float dist = pos.x + pos.y*(1.0/(radSmall*radSmall));
    float innerDelta = fwidth(dist);

    float innerAlpha = 1.0-smoothstep(1.0 - innerDelta, 1.0 + innerDelta, dist);

    //float impl = res.a*p.x*p.x + res.b*p.y*p.y + res.c*p.x*p.y + res.d*p.x + res.e*p.y + res.f;
    fragColor = color_frag;
    fragColor.a = innerAlpha;
    if(fragColor.a < 0.01) discard;*/

    highp vec2 p=(gl_FragCoord.xy/viewport.xy)*2.0-1.0;
    highp vec2 uv, uv2;
    float eps = 1e-4, d;
    vec4 o;
    highp vec3 A=Ap.xyz, B=Bp.xyz, C=Cp.xyz, D=Dp.xyz;

    // eliminates Z, sys2x2: L12 -= (xe,ye).L3
    A.xy = (A.xy+vec2(1.0))*viewport.xy*0.5;
    B.xy = (B.xy+vec2(1.0))*viewport.xy*0.5;
    C.xy = (C.xy+vec2(1.0))*viewport.xy*0.5;
    D.xy = (D.xy+vec2(1.0))*viewport.xy*0.5;
    A.xy -= gl_FragCoord.xy;
    B.xy -= gl_FragCoord.xy;
    C.xy -= gl_FragCoord.xy;
    D.xy -= gl_FragCoord.xy;
//    fragColor = vec4(length(A.xy)<5.1, length(B.xy)<5.1, length(C.xy)<5.1, length(D.xy) );
//return;
    highp vec3 AB = B-A, AC = C-A, CD = D-C, ABCD = CD-AB; // bilin = A + u.AB + v.AC +uv.ABCD = 0

    bool deg, swap, lin;
    if (lin = length(ABCD.xy) < eps) { // no uv: the system is indeed linear !
            A.z  = cross(A ,AC).z; // eliminates v -> gives u
        AB.z = cross(AB,AC).z;
        uv.x = -A.z/AB.z;
        uv.y = -A.y/AC.y -AB.y/AC.y*uv.x; // inject u in L2 -> gives v
        uv2 = uv;
    }
    else {   // full bilinear system.  eliminates uv -> sys1: Az + u.ABz + v.ACz = 0
        A.z  = cross(A ,ABCD).z;
        AB.z = cross(AB,ABCD).z;
        AC.z = cross(AC,ABCD).z;

        if (deg = abs(AC.z)<eps) { // v eliminated as well ! -> gives u
            //o-=o++; return; // <><><> does this case exist ?
            //uv.x = -A.z/AB.z;
            //uv.y = -A.y/AC.y -AB.y/AC.y*uv.x; // inject u in L2 -> gives v
            //uv2 = uv;
            //if (abs(AC.y)<eps) d=-1.; // really unlucky
            discard;
        }
        else { // full normal bilinear system.
                highp float e = -A.z/AC.z, f = -AB.z/AC.z, // ->  v = e + u.f
                // inject v in L2 -> P2(u): a.u^2 + b.u + c = 0    -> solve P2(u) then v
                    a = ABCD.y*f, b = ABCD.y*e + AC.y*f + AB.y, c = AC.y*e + A.y;
                    d = b*b-4.*a*c;
                if (lin = abs(a)<eps)  // <><><> better to use bigger eps: near-lin is unstable
                uv2.x = uv.x  = -c/b; // no parabolic term
            else {
                            uv.x  = (-b+sqrt(d))/a/2.;
                        uv2.x = (-b-sqrt(d))/a/2.;
                }
                uv.y  = e + f*uv.x;
                uv2.y = e + f*uv2.x;
        }
    }

    // --- select valid solution and display

    uv  = 2.*uv -1.;
    uv2 = 2.*uv2-1.;
    if ( swap = abs(uv.x)>1. || abs(uv.y)>1.) uv = uv2;
    highp float l = length(uv);

    // o = texture2D(iChannel0,.5+.5*uv); return;

    float alpha;
    if  (d<0.) alpha = 0.0; // red: ray didn't intersect the support twisted surface
    else if ( abs(uv.x)>1. || abs(uv.y)>1.) alpha = 0.0; // out of patch bounds
    else {
             alpha = step(l,1.);                        // circles in patch coords
    }

    fragColor.rgb = color_frag.rgb;
    fragColor.a = alpha;
    //fragColor.a = innerAlpha;
    if(fragColor.a < 0.1) discard;
}
