#version 150
in mediump vec4 color_frag;
in mediump vec3 viewnormal;
in mediump float pointsize;
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
    vec3 proju = cross(viewnormal, vec3(0.0,0.0,1.0)); // on screen TODO: To camera
    vec3 projv = cross(viewnormal, proju); // most compressed axis
    vec2 coord = gl_PointCoord - vec2(0.5);
    coord = proju.xy*coord.x+projv.xy*coord.y;
    coord *= coord;
    float dist = dot(coord, vec2(4.0));
    //float dist = distance(gl_PointCoord, vec2(0.5));
    if(dist > 0.5) discard;
    //float impl = res.a*p.x*p.x + res.b*p.y*p.y + res.c*p.x*p.y + res.d*p.x + res.e*p.y + res.f;
    fragColor = vec4(viewnormal, 1.0);//color_frag;
}
