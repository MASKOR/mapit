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

//TODO:
// 1) Go "finalSize" to right in viewspace -> project.
// 2) Go "finalSize" to left  in viewspace -> project.
// 3) Go "finalSize" to top/bottom  in viewspace -> project.
// 4) Maximum is gl_PointSize
// 5) Frag: Unproject pixel to surfel coords. (ray? bring ray from projection to viewspace)
// 6) test radius

//vec2 project(vec3)

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
    float finalSize = pointsize / dist;

    //vec2 pos2d = gl_Position.xy/gl_Position.w;
    //pos2d *= vec2(1280/2, 800); // DK1 vec2(1920/2, 1080); // DK2

    finalSize = mix(0.0, finalSize, float(mod(gl_VertexID, nthPoint)==0));
    gl_PointSize = finalSize;

    //int winX = (int) Math.round((( point3D.getX() + 1 ) / 2.0) *
    //                                   width );
    //      //we calculate -point3D.getY() because the screen Y axis is
    //      //oriented top->down
    //      int winY = (int) Math.round((( 1 - point3D.getY() ) / 2.0) *
    //                                   height );
}
