#version 400
in mediump vec4 color_frag;
in mediump vec3 viewnormal;
in mediump vec3 viewPoint;
out vec4 fragColor;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectioninvmatrix;
uniform mediump vec4 viewport; //TODO: wz! and: do not use, incorrect for oculus!

vec3 unproj(vec3 p, vec3 n)
{
    vec2 xy = gl_FragCoord.xyz/viewport.xy;
    vec4 v_screen = vec4(xy, 0.0, 1.0 );
    vec4 v_homo = inverse(gl_ProjectionMatrix) * 2.0*(v_screen-vec4(0.5));
    vec3 v_eye = v_homo.xyz / v_homo.w; //transfer from homogeneous coordinates
    return v_eye;
}

void main(void)
{
    vec3 normal = normalize(viewnormal);
    if(normal.z>0.0)
    {
        normal = -normal;
    }
    vec2 pos = (gl_PointCoord.xy - vec2(0.5))*2.0;

    vec3 tang = normalize(cross(normal, vec3(0.0,0.0,1.0))); //longest radius
    vec3 bitang = normalize(cross(tang, normal)); //smallest radius

    pos = (pos.x*normalize(tang.xy))+pos.y*normalize(bitang.xy);
    pos *= pos;

    //float radSmall = min(1.0,normal.z/length(normal.xy));
    float radSmall = min(1.0,dot(normalize(viewPoint), normal));

    //float dist = dot(pos, 1.0 / (radii * radii));
    float dist = pos.x + pos.y*(1.0/(radSmall*radSmall));
    float innerDelta = fwidth(dist);

    float innerAlpha = 1.0-smoothstep(1.0 - innerDelta, 1.0 + innerDelta, dist);

    //float impl = res.a*p.x*p.x + res.b*p.y*p.y + res.c*p.x*p.y + res.d*p.x + res.e*p.y + res.f;
    fragColor = color_frag;
    fragColor.a = innerAlpha;
    if(fragColor.a < 0.01) discard;
}
