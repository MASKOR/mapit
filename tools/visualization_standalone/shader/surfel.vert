#version 150

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;
uniform mat4 projectionMatrix;
uniform mat4 viewportMatrix;
uniform float fieldOfView;
uniform float fieldOfViewVertical;

out vec3 viewspacePosition;
out vec3 viewspaceNormal;
out vec3 viewspaceTang;
out vec3 viewspaceBitang;
out vec3 color;

uniform float pointSize;

void main(void)
{
    vec4 viewSpacePos = modelView * vec4(vertexPosition, 1.0);
    viewspacePosition = viewSpacePos.xyz;
    viewspaceNormal = modelViewNormal * vertexNormal;
    viewspaceTang = normalize(cross(viewspaceNormal, vec3(0.0,0.0,1.0))); //longest radius
    viewspaceBitang = normalize(cross(viewspaceTang, viewspaceNormal)); //smallest radius
    gl_Position = projectionMatrix * viewSpacePos;

    // VERY SLOW
    //vec2 csize = max(max(max(max(max(abs(Ap.xy-Bp.xy), abs(Ap.xy-Cp.xy)), abs(Ap.xy-Dp.xy)), abs(Bp.xy-Cp.xy)), abs(Bp.xy-Dp.xy)), abs(Cp.xy-Dp.xy));

    float dist = -viewSpacePos.z; // == gl_Position.w

//    finalSize = mix(0.0, finalSize, float(mod(gl_VertexID, nthPoint)==0));
    gl_PointSize = viewportMatrix[1][1] * projectionMatrix[1][1] * pointSize / gl_Position.w;
    color = vertexPosition * 0.1;
}
