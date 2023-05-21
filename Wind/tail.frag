#version 430

in vec4 tail_color;

out vec4 FragColor;

void main() 
{
    //FragColor = vec4(1, 1, 0, 1);
    FragColor = tail_color;
}