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
uniform mediump float pointsize;
in mediump float pointSize_frag;
out vec4 fragColor;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectionmatrix;
uniform mediump mat4 projectioninvmatrix;
uniform highp vec4 viewport; //TODO: wz! and: do not use, incorrect for oculus!

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



    // line plane intersection
    // t = ( dot(N,O) + d ) / ( dot(N,D) )
    // p = O + t*D
    // vec3 O = vec3(0.0);
    vec3 normal = normalize(viewnormal);
    vec3 ray_dir = normalize(eyePos.xyz);
    float d = dot(normal, viewPoint);
    float t = d / dot(normal, ray_dir);
    vec3 p = t * ray_dir;
    //float dist = length(cross(ray_dir, viewPoint)); //< sphere
    float dist = distance(p,viewPoint); //< disc
    float alpha = pointsize-dist*1000.0;

    fragColor.rgb = color_frag.rgb;
    fragColor.a = alpha;
    if(fragColor.a < 0.1) discard;
}
