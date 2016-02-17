#version 400
in mediump vec4 color_frag;
in highp vec3 viewnormal;
in highp vec3 tang;
in highp vec3 bitang;
in mediump vec3 viewPoint;
in highp vec4 Ap;
in highp vec4 Bp;
in highp vec4 Cp;
in highp vec4 Dp;
uniform mediump float pointsize;
uniform bool discrender;
in mediump float pointSize_frag;
out vec4 fragColor;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectionmatrix;
uniform mediump mat4 projectioninvmatrix;
uniform highp vec4 viewport; //TODO: wz! and: do not use, incorrect for oculus!

/*
{
    vec2 xy = gl_FragCoord.xy/viewport.xy;
    vec4 v_screen = vec4(xy, 0.0, 1.0 );
    vec4 v_homo = inverse(gl_ProjectionMatrix) * 2.0*(v_screen-vec4(0.5));
    vec3 v_eye = v_homo.xyz / v_homo.w; //transfer from homogeneous coordinates
    return v_eye;
}*/

void main(void)
{

//    vec3 eyePos = vec3(viewPoint.xy+(gl_PointCoord.xy - vec2(0.5))*2.0, viewPoint.z); //< this is not enough. perspective division was skipped?
// unproject
    vec3 ndcPos;
    ndcPos.xy = gl_FragCoord.xy / viewport.xy;
    ndcPos.z = viewPoint.z;
    ndcPos -= 0.5;
    ndcPos *= 2.0;
    vec4 clipPos;
    clipPos.w = (2.0 * viewport.z * viewport.w / ((viewport.z + viewport.w) + ndcPos.z * (viewport.z - viewport.w)));
    clipPos.xyz = ndcPos * clipPos.w;
    vec4 eyePos = projectioninvmatrix * clipPos;

// For orthogonal projection this would be used and ray/plane intersection would have to be tweaked (ray does not start in origin then).
/*    vec3 ndcPos2;
    ndcPos2.xy = gl_FragCoord.xy / viewport.xy;
    ndcPos2.z = viewport.w;
    ndcPos2 -= 0.5;
    ndcPos2 *= 2.0;
    vec4 clipPos2;
    clipPos2.w = (2.0 * viewport.z * viewport.w / ((viewport.z + viewport.w) + ndcPos2.z * (viewport.z - viewport.w)));
    clipPos2.xyz = ndcPos2 * clipPos2.w;
    vec4 eyePos2 = projectioninvmatrix * clipPos2;*/
    vec3 normal = normalize(viewnormal);
    float alpha;
    if(discrender)
    {
    // line plane intersection
    // t = ( dot(N,O) + d ) / ( dot(N,D) )
    // p = O + t*D
    // vec3 O = vec3(0.0);
    vec4 ray_dir = normalize(eyePos);
    float d = dot(normal, viewPoint);
    float t = d / dot(normal, ray_dir.xyz);
    vec4 p = t * ray_dir;
    float distSphere = length(cross(ray_dir.xyz, viewPoint)); //< sphere
    float distDisc = distance(p.xyz,viewPoint); //< disc
    float dist = distDisc;//mix(distSphere, distDisc, discrender);
    //dist *= 1000.0; //< why?
    alpha = pointsize-dist;
    }
    else
    {
        vec2 pos = (gl_PointCoord.xy - vec2(0.5))*2.0;
        alpha = float(length(pos)<1.0);
        //float radSmall = min(1.0,dot(normalize(viewPoint), normal));
        //vec2 pos = (gl_PointCoord.xy - vec2(0.5))*2.0;
        //pos = (-pos.x*normalize(tang.xy))+pos.y*normalize(bitang.xy);
        //pos *= pos;
        //float dist = pos.x + pos.y*(1.0/(radSmall*radSmall));
        //alpha = 1.0-dist;

//FOR SQUARES
        //alpha = 1.0;
    }

    fragColor.rgb = color_frag.rgb;
    fragColor.a = alpha;
    if(fragColor.a <= 0.0) discard;
    // todo: fix
    //vec4 reprojeced_ndc_pos = projectionmatrix * -p;
    //float ndc_depth = reprojeced_ndc_pos.z / reprojeced_ndc_pos.w;

    //float depth = (((viewport.w-viewport.z) * ndc_depth) + viewport.z + viewport.w) / 2.0;
    //gl_FragDepth = depth;
}
