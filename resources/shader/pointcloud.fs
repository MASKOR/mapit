#version 400
in mediump vec4 color_frag;
in mediump vec3 viewnormal;
in mediump vec3 viewPoint;
out vec4 fragColor;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectioninvmatrix;
uniform mediump vec2 viewport; //TODO: wz!

void main(void)
{
///    vec4 p_ndc = vec4(2.0 * (gl_FragCoord.xy - viewport.xy)
///        / (viewport.zw) - 1.0, -1.0, 1.0);
///    vec4 p_eye = projection_matrix_inv * p_ndc;
///    vec3 qn = p_eye.xyz / p_eye.w;
///    vec3 proju = cross(viewnormal, vec3(0.0,0.0,1.0)); // on screen TODO: To camera
///    vec3 projv = cross(viewnormal, proju); // most compressed axis
///    vec2 coord = gl_PointCoord - vec2(0.5);
///    coord = proju.xy*coord.x+projv.xy*coord.y;
///    coord *= coord;
///    float dist = dot(coord, vec2(4.0));
    //float dist = distance(gl_PointCoord, vec2(0.5));

    vec3 normal = normalize(viewnormal);
    if(normal.z<0.0)
    {
        normal = -normal;
    }
    vec2 pos = (gl_PointCoord.xy - vec2(0.5))*2.0;

    vec3 tang = normalize(cross(normal, vec3(0.0,0.0,1.0))); //longest radius
    vec3 bitang = normalize(cross(tang, normal)); //smallest radius

    vec2 pos3d = (pos.x*normalize(tang.xy))+pos.y*normalize(bitang.xy);
    pos = pos3d.xy;
    pos *= pos;

    float radSmall = min(1.0,normal.z/length(normal.xy));

    float dist = pos.x + pos.y*(1.0/(radSmall*radSmall));
    //float innerDist = dot(pos, 1.0 / (ab * ab));
    float innerDelta = fwidth(dist) * 0.8;

    float innerAlpha = smoothstep(1.0 - innerDelta, 1.0 + innerDelta, dist);

    if(dist > 0.5) discard;
    //float impl = res.a*p.x*p.x + res.b*p.y*p.y + res.c*p.x*p.y + res.d*p.x + res.e*p.y + res.f;
    fragColor = color_frag;
}
