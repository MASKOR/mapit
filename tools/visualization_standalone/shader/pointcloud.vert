#version 400

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

out vec3 position;
out vec3 normal;
out vec3 color;

uniform mat4 modelMatrix;
uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;
uniform mat4 projectionMatrix;
uniform mat4 viewportMatrix;

uniform int colorize;

uniform float lod;

uniform float pointSize;

uniform float colorscale;

uniform bool constantSize;
uniform bool yPointsUp;
//in int gl_VertexID;

int LFSR_Rand_Gen(in int n)
{
  // <<, ^ and & require GL_EXT_gpu_shader4.
  n = (n << 13) ^ n;
  return (n * (n*n*15731+789221) + 1376312589) & 0x7fffffff;
}

float rand(in int n)
{
    return float(LFSR_Rand_Gen(n))*(1.0/2147483647.0); // max int
}

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

void main()
{
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    normal = normalize(modelViewNormal * vertexNormal);
    position = vec3(modelView * vec4(vertexPosition, 1.0));

    float vertexRand = rand(gl_VertexID);
    float jitter = 1.0+vertexRand*0.1;
    float dist = length(position.xyz) * jitter;

    float nearMaxDetailDistance = 20;
    float farMinDetailDistance = 1000;
    float cappedDist = max(nearMaxDetailDistance, min(farMinDetailDistance,dist*lod));
    float percentSkipped = pow(smoothstep(nearMaxDetailDistance, farMinDetailDistance, cappedDist), 0.1)*0.999;
    float random01 = max(0.0, min(1.0, (vertexRand+1.0)*0.5));
    float visibility = step(percentSkipped, random01);

    if(constantSize)
    {
        gl_PointSize = pointSize*20.0;
    }
    else
    {
        gl_PointSize = max(0.5,viewportMatrix[1][1] * projectionMatrix[1][1] * pointSize / dist) * visibility;
    }


    //color = vertexPosition * 0.1;//vertexColor;
    float axis;
    vec4 worldPos = modelMatrix * vec4(vertexPosition, 1.0);
    if(colorize == 0)
        axis = worldPos.x;
    else if(   colorize == 1 && yPointsUp == true
            || colorize == 2 && yPointsUp == false)
        axis = worldPos.y;
    else if(   colorize == 1 && yPointsUp == false
            || colorize == 2 && yPointsUp == true)
        axis = worldPos.z;
    color = hsv_to_rgb(axis*-colorscale, 0.7, 1.0, 0.0).rgb;
}
