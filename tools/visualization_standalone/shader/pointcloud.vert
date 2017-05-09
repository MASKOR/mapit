#version 400

in vec3 vertexPosition;
in vec3 vertexNormal;
in vec3 vertexColor;

out vec3 position;
out vec3 normal;
out vec3 color;

uniform mat4 modelView;
uniform mat3 modelViewNormal;
uniform mat4 mvp;
uniform mat4 projectionMatrix;
uniform mat4 viewportMatrix;

uniform float lod;

uniform float pointSize;

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

void main()
{
    gl_Position = mvp * vec4(vertexPosition, 1.0);
    normal = normalize(modelViewNormal * vertexNormal);
    position = vec3(modelView * vec4(vertexPosition, 1.0));

    float vertexRand = rand(gl_VertexID);
    float jitter = 1.0+vertexRand*0.1;
    float dist = gl_Position.w * jitter;

    float nearMaxDetailDistance = 20;
    float farMinDetailDistance = 1000;
    float cappedDist = max(nearMaxDetailDistance, min(farMinDetailDistance,dist*lod));
    float percentSkipped = pow(smoothstep(nearMaxDetailDistance, farMinDetailDistance, cappedDist), 0.1)*0.999;
    float random01 = max(0.0, min(1.0, (vertexRand+1.0)*0.5));
    float visibility = step(percentSkipped, random01);

    gl_PointSize = max(0.5,viewportMatrix[1][1] * projectionMatrix[1][1] * pointSize / dist) * visibility;


    color = vertexPosition * 0.1;//vertexColor;
}
