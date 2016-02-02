#version 400
in highp vec4 vertex;
in mediump vec3 normal;
in mediump vec3 color;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectionmatrix;
uniform mediump mat3 modelviewnormalmatrix;
out mediump vec4 color_frag;
out mediump vec3 viewnormal;
out mediump vec3 viewPoint;
uniform mediump float pointsize;
uniform mediump float distanceDetail;
//in int gl_VertexID;
void main(void)
{
    color_frag.rgb = clamp(color, 0.0, 1.0);
    float litMul = smoothstep(1.59, 1.73, length(color));
    float lit = dot(normal, normalize(vec3(0.1,-0.5, 0.1)));
    lit = smoothstep(-1.0, 1.0, lit);
    color_frag.rgb *= mix(1.0, lit, litMul);
    //color_frag.a = 1.0;
    //color_frag = vec4(gl_VertexID, float(gl_VertexID)*0.01, float(gl_VertexID)*0.001, 1.0);
    vec4 viewSpacePos = modelviewmatrix * vertex;
    viewPoint = viewSpacePos.xyz;
    viewnormal = modelviewnormalmatrix * (normal*vec3(1.0,1.0,1.0));
    gl_Position = projectionmatrix * viewSpacePos;
    float dist = gl_Position.w;//length(viewSpacePos.xyz); // camera in viewspace is at (0,0,0)

    float d3 = max(0.0, dist-20.0);
    int nthPoint = max(1,int(pow(d3,2.0)*0.02*distanceDetail));
    float finalSize = mix(0.0, pointsize / dist, float(mod(gl_VertexID, nthPoint)==0));
    gl_PointSize = finalSize;
}
