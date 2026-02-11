#version 330

// Input vertex attributes
in vec3 vertexPosition;
in vec3 vertexNormal;
in vec4 vertexColor;

// Input uniform values
uniform mat4 mvp;

// Output vertex attributes (to fragment shader)
out vec4 fragColor;

vec3 lightDir = vec3(1.732, 1.732, 1.732);

#define INTENSITY 1

void main()
{   
    float NdotL = max(dot(vertexNormal, lightDir), 0.0);
    fragColor = vec4(vertexColor.rgb * NdotL, 1.0);

    // Calculate final vertex position
    gl_Position = mvp*vec4(vertexPosition, 1.0);
}