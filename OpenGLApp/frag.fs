#version 330 core
out vec4 FragColor;
in vec2 TexCoords;
in vec3 WorldPos;
in vec3 Normal;

// material parameters
uniform sampler2D texture_PBR_diffuse1;
uniform sampler2D texture_PBR_normal1;
uniform sampler2D texture_PBR_metallic1;
uniform sampler2D texture_PBR_roughness1;
uniform sampler2D texture_PBR_ambient_occlusion1;

uniform bool useMR;

uniform vec3 camPos;

uniform bool useColor;
uniform vec3 color;
uniform float opacity;

void main(){
	if (useColor){
		FragColor = vec4(color, opacity);
	}
	else{
		FragColor = vec4(texture(texture_PBR_diffuse1, TexCoords).rgb, opacity);
	}
}