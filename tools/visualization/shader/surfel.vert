#version 130

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

out vec3 viewspacePosition;
out vec3 viewspaceNormal;
out vec3 viewspaceTang;
out vec3 viewspaceBitang;
out vec3 color;

uniform mat4 modelMatrix;
uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;
uniform mat4 projectionMatrix;
uniform mat4 viewportMatrix;
uniform float fieldOfView;
uniform float fieldOfViewVertical;


uniform int colorize; // choose parameter to use for color
uniform int colorMode; // choose style for coloring

uniform float colorscale;

uniform bool constantSize;
uniform bool yPointsUp;

uniform float pointSize;

uniform int lod;

vec4 hsv_to_rgb(float h, float s, float v, float a)
{
        float c = v * s;
        h = mod((h * 6.0), 6.0);
        float x = c * (1.0 - abs(mod(h, 2.0) - 1.0));
        vec4 color;

        if (0.0 <= h && h < 1.0) {
                color = vec4(c, x, 0.0, a);
        } else if (1.0 <= h && h < 2.0) {
                color = vec4(x, c, 0.0, a);
        } else if (2.0 <= h && h < 3.0) {
                color = vec4(0.0, c, x, a);
        } else if (3.0 <= h && h < 4.0) {
                color = vec4(0.0, x, c, a);
        } else if (4.0 <= h && h < 5.0) {
                color = vec4(x, 0.0, c, a);
        } else if (5.0 <= h && h < 6.0) {
                color = vec4(c, 0.0, x, a);
        } else {
                color = vec4(0.0, 0.0, 0.0, a);
        }

        color.rgb += v - c;

        return color;
}

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

    // x2 would be perfect. But the smaller the points are, the better performance is.
    gl_PointSize = 1.9 * viewportMatrix[1][1] * projectionMatrix[1][1] * pointSize / gl_Position.w;

    float axis;
    vec4 worldPos = modelMatrix * vec4(vertexPosition, 1.0);
    vec3 worldNormal = normalize(vec4(modelMatrix * vec4(vertexNormal, 0.0)).xyz); // Note: this is not correct due to scaling!
    if(colorize == 0)
        axis = worldPos.x;
    else if(   colorize == 1 && yPointsUp == true
            || colorize == 2 && yPointsUp == false)
        axis = worldPos.y;
    else if(   colorize == 1 && yPointsUp == false
            || colorize == 2 && yPointsUp == true)
        axis = worldPos.z;
    else if(colorize == 3)
        axis = worldNormal.x;
    else if(   colorize == 4 && yPointsUp == true
            || colorize == 5 && yPointsUp == false)
        axis = worldNormal.y;
    else if(   colorize == 4 && yPointsUp == false
            || colorize == 5 && yPointsUp == true)
        axis = worldNormal.z;
    else if(   colorize == 7)
        axis = vertexColor.r;
//    else if(colorize == 7) // TODO: intensity
//        axis = intensity;
    if(colorMode == 0) // flashlight (double sided)
        color = vec3(abs(dot(modelViewNormal * vertexNormal, normalize(viewspacePosition))));
    else if(colorMode == 1) // gray axis
        color = vec3(axis*colorscale);
    else if(colorMode == 2) // HSV
        color = hsv_to_rgb(axis*-colorscale, 0.7, 1.0, 0.0).rgb;
    else if(colorMode == 3) // flashlight2
    color = vec3(-dot(modelViewNormal * vertexNormal, normalize(viewspacePosition)));
    else if(colorMode == 4) // flashlight (single sided)
        color = vec3(dot(modelViewNormal * vertexNormal, vec3(0.0,0.0,1.0)));
    if(colorize == 7) {
        color = vertexColor;//unpackColor(vertexColor);
    }
}
