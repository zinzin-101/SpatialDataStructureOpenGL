#version 330 core
out vec4 FragColor;

in vec2 TexCoords;

uniform sampler2D texture_PBR_diffuse;
uniform sampler2D texture_PBR_normal;
uniform sampler2D texture_PBR_metallic;
uniform sampler2D texture_PBR_roughness;
uniform sampler2D texture_PBR_ambient_occlusion;

void main()
{    
    FragColor = texture(texture_PBR_roughness, TexCoords);
    //FragColor = vec4(1.0);
}