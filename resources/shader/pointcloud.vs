#version 150
in highp vec4 vertex;
in mediump vec3 normal;
in mediump vec3 color;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectionmatrix;
uniform mediump mat3 modelviewnormalmatrix;
out mediump vec4 color_frag;
out mediump vec3 viewnormal;
out mediump float pointsize;
out mediump vec3 viewPoint;
const float pointSizeMultiplier = 156.0;
/*
out float a, b, c, d, e, f;
function projectSphere( vec3 o, float r, float fov,
    out float a,
    out float b,
    out float c,
    out float d,
    out float e,
    out float f)
{
    float r2 = r;
    float z2 = o.z*o.z;
    float l2 = dot(o,o);

    a = r2 - o.y*o.y - z2;
    b = r2 - o.x*o.x - z2;
    c = 2.0*o.x*o.y;
    d = 2.0*o.x*o.z*fov;
    e = 2.0*o.y*o.z*fov;
    f = (r2-l2+z2)*fov*fov;
}*/

void main(void)
{
    color_frag.rgb = clamp(color, 0.0, 1.0);
    color_frag.a = 1.0;
    vec4 viewSpacePos = modelviewmatrix * vertex;
    viewPoint = viewSpacePos.xyz;
    viewnormal = modelviewnormalmatrix * normal;
    gl_Position = projectionmatrix * viewSpacePos;
    float dist = length(viewSpacePos.xyz); // camera in viewspace is at (0,0,0)
    pointsize = pointSizeMultiplier / dist;
    gl_PointSize = pointsize;
}
