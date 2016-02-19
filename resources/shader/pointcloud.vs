#version 400
in highp vec4 vertex;
in mediump vec3 normal;
in mediump vec3 color;
uniform mediump mat4 modelviewmatrix;
uniform mediump mat4 projectionmatrix;
uniform mediump mat3 modelviewnormalmatrix;
out mediump vec4 color_frag;
out highp vec3 viewnormal;
out highp vec3 tang;
out highp vec3 bitang;
out mediump vec3 viewPoint;
out mediump float pointSize_frag;
uniform int discrender;
out highp vec4 Ap;
out highp vec4 Bp;
out highp vec4 Cp;
out highp vec4 Dp;
uniform mediump float pointsize;
uniform mediump float distanceDetail;
uniform mediump float heightOfNearPlaneInv;
uniform highp vec4 viewport; //TODO: wz! and: do not use, incorrect for oculus!
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
    //float litMul = smoothstep(1.59, 1.73, length(color));
    //float lit = dot(normal, normalize(vec3(0.1,-0.5, 0.1)));
    //lit = smoothstep(-1.0, 1.0, lit);
    //color_frag.rgb *= mix(1.0, lit, litMul);
    //color_frag.a = 1.0;
    //color_frag = vec4(gl_VertexID, float(gl_VertexID)*0.01, float(gl_VertexID)*0.001, 1.0);
    highp vec4 viewSpacePos = modelviewmatrix * vertex;
    viewPoint = viewSpacePos.xyz;
    viewnormal = modelviewnormalmatrix * normal;
    tang = normalize(cross(viewnormal, vec3(0.0,0.0,1.0))); //longest radius
    bitang = normalize(cross(tang, viewnormal)); //smallest radius
    gl_Position = projectionmatrix * viewSpacePos;
    Ap = projectionmatrix * vec4(viewSpacePos.xyz - (tang - bitang)*0.1, 1.0);
    Bp = projectionmatrix * vec4(viewSpacePos.xyz - (tang + bitang)*0.1, 1.0);
    Cp = projectionmatrix * vec4(viewSpacePos.xyz + (tang + bitang)*0.1, 1.0);
    Dp = projectionmatrix * vec4(viewSpacePos.xyz + (tang - bitang)*0.1, 1.0);
    Ap /= Ap.w;
    Bp /= Bp.w;
    Cp /= Cp.w;
    Dp /= Dp.w;
    // VERY SLOW
    //vec2 csize = max(max(max(max(max(abs(Ap.xy-Bp.xy), abs(Ap.xy-Cp.xy)), abs(Ap.xy-Dp.xy)), abs(Bp.xy-Cp.xy)), abs(Bp.xy-Dp.xy)), abs(Cp.xy-Dp.xy));
    /*Ap.xy = Ap.xy*0.5+0.5;
    Bp.xy = Bp.xy*0.5+0.5;
    Cp.xy = Cp.xy*0.5+0.5;
    Dp.xy = Dp.xy*0.5+0.5;*/

    float dist = -viewSpacePos.z; // == gl_Position.w

//float fovy = fov; // degrees
//float heightOfNearPlaneInv = viewport.y / (2.0*tan(0.5*fovy*3.14159265/180.0));



    float d3 = max(0.0, dist-20.0);
    int nthPoint = max(1,int(pow(d3,2.0)*0.02*distanceDetail));
    nthPoint += 10;
    float finalSize = heightOfNearPlaneInv * pointsize * 2.0 / dist; // < todo: heightOfNearPlaneInv has a wrong name it is not what it is named after!

    //vec2 pos2d = gl_Position.xy/gl_Position.w;
    //pos2d *= vec2(1280/2, 800); // DK1 vec2(1920/2, 1080); // DK2


    if(discrender == 0) finalSize += max(0.0,length(gl_Position.xy/gl_Position.w)*20.0*(7.0-gl_Position.w));
    if(discrender > 3) finalSize = min(5, pointsize*200.0);
    finalSize = mix(0.0, finalSize, float(mod(gl_VertexID, nthPoint)==0));
    gl_PointSize = finalSize;

    if(distance(color.rgb, vec3(1.0)) < 0.001) gl_PointSize = 0.0;


    //int winX = (int) Math.round((( point3D.getX() + 1 ) / 2.0) *
    //                                   width );
    //      //we calculate -point3D.getY() because the screen Y axis is
    //      //oriented top->down
    //      int winY = (int) Math.round((( 1 - point3D.getY() ) / 2.0) *
    //                                   height );
}
