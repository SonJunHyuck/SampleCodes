// --- system header ---
#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <cmath>

#include "include/GL/glew.h"
#include "include/GLFW/glfw3.h"
#include "include/GL/glut.h"

 //glm is used to create perspective and transform matrices
#define GLM_SWIZZLE
#include "include/glm/gtc/matrix_transform.hpp"
#include "include/glm/gtc/type_ptr.hpp"
#include "include/glm/gtc/random.hpp"

// --- custom header ---
#include "Global_Variable.h"
#include "UtilityFunc.h"

#pragma comment(lib, "OpenGL32.lib")
#pragma comment(lib, "lib/freeglutd.lib")
#pragma comment(lib, "lib/glew32.lib")
#pragma comment(lib, "lib/glfw3.lib")

using namespace std;
using namespace Global_Variable;

GLFWwindow* window;

int width = 1680;
int height = 900;
int frame_current = 0;

// particle data
const int particles = 8 * 1024;  // Particle Count`
const int tail_length = 256;	// Particle path length
int particle_resolution;
int particle_num;
std::vector<glm::vec4> tail_colorData;
std::vector<glm::vec2> positionData, tailData;
std::vector<float> ageData, lifeData;
GLuint particle_VAO;
GLuint particle_IndexBuffer, particle_positionBuffer, particle_tailBuffer, particle_ageBuffer, particle_lifeBuffer, particle_tail_colorBuffer;

// quad
std::vector<glm::vec2> quadPos, quadTex;
GLuint quad_VAO, quad_position_buffer, quad_texcord_buffer;

// framebuffer
GLuint framebuffer, texture_Colorbuffer, renderbuffer;

// Shader Program
ShaderProgram* shader_particle;
ShaderProgram* shader_tail;
ShaderProgram* shader_map;
ShaderProgram* shader_screen;

// Shader_Compute
GLuint shaderProgram_comp;

// Texture
GLuint wind_texture;
GLuint world_texture;
GLuint particleState_texture;

#pragma region Check
// helper to check and display for shader compiler errors
bool check_shader_compile_status(GLuint obj) {
	GLint status;
	glGetShaderiv(obj, GL_COMPILE_STATUS, &status);
	if (status == GL_FALSE) {
		GLint length;
		glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &length);
		std::vector<char> log(length);
		glGetShaderInfoLog(obj, length, &length, &log[0]);
		std::cerr << &log[0];
		return false;
	}
	return true;
}

// helper to check and display for shader linker error
bool check_program_link_status(GLuint obj) {
	GLint status;
	glGetProgramiv(obj, GL_LINK_STATUS, &status);
	if (status == GL_FALSE) {
		GLint length;
		glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &length);
		std::vector<char> log(length);
		glGetProgramInfoLog(obj, length, &length, &log[0]);
		std::cerr << &log[0];
		return false;
	}
	return true;
}
#pragma endregion

#pragma region CALL BACK
bool mouse_left_button, mouse_right_button, mouse_mid_button;
double last_mouse_posX, last_mouse_posY;
double cursor_posX, cursor_posY;

void SetSize(int width, int height)
{
	window_width = width;
	window_height = height;
}

void errorCallBack(int errorCode, const char* errorDescription)
{
	fprintf(stderr, "Error : %s\n", errorDescription);
}

void windowSizeCallBack(GLFWwindow* win, int width, int height)
{
	SetSize(width, height);
	glViewport(0, 0, width, height);
}

void keyboardCallBack(GLFWwindow* win, int key, int scancode, int action, int mods)
{
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(win, GLFW_TRUE);
}

void cursorPosCallBack(GLFWwindow* win, double x, double y)
{
	cursor_posX = x;
	cursor_posY = y;
}

void mouseButtonCallBack(GLFWwindow* win, int button, int action, int mods)
{
	if (action == GLFW_PRESS) {
		double xpos, ypos;
		glfwGetCursorPos(win, &xpos, &ypos);
		last_mouse_posX = xpos;
		last_mouse_posY = ypos;
	}

	if (button == GLFW_MOUSE_BUTTON_LEFT) {
		if (GLFW_PRESS == action)
			mouse_left_button = true;
		else if (GLFW_RELEASE == action)
			mouse_left_button = false;
	}

	else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
		if (GLFW_PRESS == action)
			mouse_right_button = true;
		else if (GLFW_RELEASE == action)
			mouse_right_button = false;
	}

	else if (button == GLFW_MOUSE_BUTTON_MIDDLE) {
		if (GLFW_PRESS == action)
			mouse_mid_button = true;
		else if (GLFW_RELEASE == action)
			mouse_mid_button = false;
	}
}

void MouseDragging(double width, double height)
{
	if (mouse_left_button) {
		float fractionChangeX = static_cast<float>(cursor_posX - last_mouse_posX) / static_cast<float>(width);
		float fractionChangeY = static_cast<float>(last_mouse_posY - cursor_posY) / static_cast<float>(height);
		viewer->rotate(fractionChangeX, fractionChangeY);
	}
	else if (mouse_mid_button) {
		float fractionChangeX = static_cast<float>(cursor_posX - last_mouse_posX) / static_cast<float>(width);
		float fractionChangeY = static_cast<float>(last_mouse_posY - cursor_posY) / static_cast<float>(height);
		viewer->zoom(fractionChangeY);
	}
	else if (mouse_right_button) {
		float fractionChangeX = static_cast<float>(cursor_posX - last_mouse_posX) / static_cast<float>(width);
		float fractionChangeY = static_cast<float>(last_mouse_posY - cursor_posY) / static_cast<float>(height);
		viewer->translate(-fractionChangeX, -fractionChangeY, 1);
	}
	last_mouse_posX = cursor_posX;
	last_mouse_posY = cursor_posY;
}
#pragma endregion

#pragma region Init
// Set up GLEW / GLFW
bool Init()
{
	if (glfwInit() == GL_FALSE)
	{
		std::cerr << "failed to init GLFW" << std::endl;
		return 1;
	}

	// select opengl version
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);

	// create a window
	if ((window = glfwCreateWindow(width, height, "13compute_shader_nbody", 0, 0)) == 0)
	{
		std::cerr << "failed to open window" << std::endl;
		glfwTerminate();
		return 1;
	}

	// CALL BACK 설정
	glfwSetWindowSizeCallback(window, windowSizeCallBack);
	glfwSetMouseButtonCallback(window, mouseButtonCallBack);
	glfwSetCursorPosCallback(window, cursorPosCallBack);
	glfwSetKeyCallback(window, keyboardCallBack);

	glfwMakeContextCurrent(window);

	if (glewInit())
	{
		std::cerr << "failed to init GL3W" << std::endl;
		glfwDestroyWindow(window);
		glfwTerminate();
		return 1;
	}

	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION),
		glGetString(GL_SHADING_LANGUAGE_VERSION));

	return 0;
}

// Set up Shader
void Init_Shader()
{
	// Particle Shader
	shader_particle = new ShaderProgram();
	shader_particle->initFromFiles("particle.vert", "particle.frag");
	shader_particle->addUniform("particle_texture");
	shader_particle->addUniform("particle_res");
	shader_particle->addUniform("wind_texture");
	shader_particle->addUniform("wind_min");
	shader_particle->addUniform("wind_max");

	// Map Shader (Background)
	shader_map = new ShaderProgram();
	shader_map->initFromFiles("map.vert", "map.frag");
	shader_map->addUniform("screenTexture");

	// Screen Shader (Quad for FrameBuffer)
	shader_screen = new ShaderProgram();
	shader_screen->initFromFiles("screen.vert", "screen.frag");
	shader_screen->addUniform("screenTexture");
	shader_screen->addUniform("alpha");

	shader_tail = new ShaderProgram();
	shader_tail->initFromFiles("tail.vert", "tail.frag");
	shader_tail->addUniform("wind_texture");


	shaderProgram_comp = Load_ComputeShader("Compute.comp");
	glUseProgram(shaderProgram_comp);
	glUniform1i(0, frame_current);
	glUniform1f(1, particle_resolution);
	glUniform1i(2, 1);  // wind texture
	glUniform1i(3, 2);  // particle texture
	glUniform1i(4, tail_length);  // particle
}

// Set up Frame Buffer
void Init_FrameBuffer()
{
	glGenFramebuffers(1, &framebuffer);
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	glGenTextures(1, &texture_Colorbuffer);
	glBindTexture(GL_TEXTURE_2D, texture_Colorbuffer);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture_Colorbuffer, 0);

	glGenRenderbuffers(1, &renderbuffer);
	glBindRenderbuffer(GL_RENDERBUFFER, renderbuffer);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, width, height); // use a single renderbuffer object for both a depth AND stencil buffer.
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, renderbuffer); // now actually attach it

	// now that we actually created the framebuffer and added all attachments we want to check if it is actually complete now
	if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		cout << "ERROR::FRAMEBUFFER:: Framebuffer is not complete!" << endl;

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// Load Texture
void Init_Texture()
{
	world_texture = Load_Texture("world.png");
	wind_texture = Load_Texture("wind.png");
}

// Set up Quad Object
void Init_Quad()
{
	quadPos = std::vector<glm::vec2>();
	quadPos.push_back(glm::vec2(-1, 1));
	quadPos.push_back(glm::vec2(-1, -1));
	quadPos.push_back(glm::vec2(1, -1));
	quadPos.push_back(glm::vec2(-1, 1));
	quadPos.push_back(glm::vec2(1, -1));
	quadPos.push_back(glm::vec2(1, 1));

	quadTex = std::vector<glm::vec2>();
	quadTex.push_back(glm::vec2(0, 1));
	quadTex.push_back(glm::vec2(0, 0));
	quadTex.push_back(glm::vec2(1, 0));
	quadTex.push_back(glm::vec2(0, 1));
	quadTex.push_back(glm::vec2(1, 0));
	quadTex.push_back(glm::vec2(1, 1));

	glGenVertexArrays(1, &quad_VAO);
	glBindVertexArray(quad_VAO);

	glGenBuffers(1, &quad_position_buffer);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, quad_position_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * quadPos.size(), quadPos.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &quad_texcord_buffer);
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, quad_texcord_buffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 2 * quadTex.size(), quadTex.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glBindVertexArray(0);
}

// Set up Particle
void Init_Particle()
{
	// 랜덤 초기화
	srand(glfwGetTimerValue());

	// 파티클 임의 위치 Texture ( 파티클 수에 비례한 사각형 그리기 )
	const int particleRes = particle_resolution = ceil(sqrt(particles));  // 소수점 올림 처리
	particle_num = particleRes * particleRes;

	// randomize the initial particle positions
	std::vector<unsigned char> particleState = std::vector<unsigned char>((double)particle_num * 4);
	for (int i = 0; i < particleState.size(); i++)
	{
		particleState[i] = floor(((float)rand() / (float)RAND_MAX) * 256); 
	}

	std::vector<float> particleIndices = std::vector<float>(particle_num);
	positionData = std::vector<glm::vec2>(particle_num);
	tailData = std::vector<glm::vec2>((double)particle_num * tail_length);
	ageData = std::vector<float>(particle_num);
	lifeData = std::vector<float>(particle_num);
	tail_colorData = std::vector<glm::vec4>((double)particle_num * tail_length);

	for (int i = 0; i < particle_num; i++)
	{
		particleIndices[i] = (float)i;
		positionData[i] = glm::vec2(0);
		ageData[i] = 0;
		lifeData[i] = floor(((float)rand() / (float)RAND_MAX) * tail_length);
	}

	// textures to hold the particle state for the current and the next frame
	particleState_texture = Create_Texture(GL_NEAREST, particleState.data(), particleRes, particleRes);

	glGenVertexArrays(1, &particle_VAO);
	glBindVertexArray(particle_VAO);

	glGenBuffers(1, &particle_IndexBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, particle_IndexBuffer);
	glEnableVertexAttribArray(0);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * particleIndices.size(), particleIndices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &particle_positionBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, particle_positionBuffer);
	glEnableVertexAttribArray(1);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * positionData.size(), positionData.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 0, 0);
		
	glGenBuffers(1, &particle_tailBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, particle_tailBuffer);
	glEnableVertexAttribArray(2);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec2) * tailData.size(), tailData.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &particle_ageBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, particle_ageBuffer);
	glEnableVertexAttribArray(3);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * ageData.size(), ageData.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(3, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &particle_lifeBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, particle_lifeBuffer);
	glEnableVertexAttribArray(4);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * lifeData.size(), lifeData.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, 0, 0);

	glGenBuffers(1, &particle_tail_colorBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, particle_tail_colorBuffer);
	glEnableVertexAttribArray(5);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 4 * tail_colorData.size(), tail_colorData.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, 0, 0);

	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, particle_IndexBuffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 1, particle_positionBuffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 2, particle_tailBuffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 3, particle_ageBuffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 4, particle_lifeBuffer);
	glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 5, particle_tail_colorBuffer);

	glBindVertexArray(0);
}
#pragma endregion

#pragma region Function For Simple
void Bind_Textures()
{
	// Active Texture
	glEnable(GL_TEXTURE_2D);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, world_texture);

	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, wind_texture);

	glActiveTexture(GL_TEXTURE2);
	glBindTexture(GL_TEXTURE_2D, particleState_texture);

	glActiveTexture(GL_TEXTURE3);
	glBindTexture(GL_TEXTURE_2D, texture_Colorbuffer);
}

void Use_ComputeShader()
{
	glUseProgram(shaderProgram_comp);
	glDispatchCompute(particle_num / 256, 1, 1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_VERTEX_ATTRIB_ARRAY_BARRIER_BIT);

	glUniform1i(0, frame_current);
}
#pragma endregion

#pragma region Draw

// 프레임버퍼에 파티클 및 맵 그리기
void Draw_Framebuffer()
{
	// bind Framebuffer
	glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);

	// enable depth testing (is disabled for rendering screen-space quad)
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	glViewport(0, 0, width, height);

	//// draw particle : 경로를 그릴 때 그려주기 때문에 사용하지 않음
	//shader_particle->use();
	//glBindVertexArray(particle_VAO);
	//glUniform1i(shader_particle->uniform("wind_texture"), 1);
	//glUniform1i(shader_particle->uniform("particle_texture"), 2);
	//glUniform1f(shader_particle->uniform("particle_res"), particle_resolution);
	//glUniform2f(shader_particle->uniform("wind_min"), 0, 0);
	//glUniform2f(shader_particle->uniform("wind_max"), 0, 0);
	//glDrawArrays(GL_POINTS, 0, particle_num);
	//glBindVertexArray(0);
	//shader_particle->disable();

	// draw tail
	shader_tail->use();
	glBindVertexArray(particle_VAO);
	glUniform1i(shader_tail->uniform("wind_texture"), 1);

	glDrawArrays(GL_POINTS, 0, tail_length * particle_num);
	shader_tail->disable();

	// draw map
	shader_map->use();
	glBindVertexArray(quad_VAO);
	glUniform1i(shader_map->uniform("screenTexture"), 0);

	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	shader_map->disable();

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// FrameBuffer를 Quad로 그리기 때문에, Depth가 필요하지 않음
	glDisable(GL_DEPTH_TEST);
}

// 프레임버퍼로 그린 스크린을 실제로 렌더링
void Draw()
{
	// clear all relevant buffers
	glClear(GL_COLOR_BUFFER_BIT);

	shader_screen->use();
	glUniform1i(shader_screen->uniform("screenTexture"), 3);
	glUniform1f(shader_screen->uniform("alpha"), 1);

	glBindVertexArray(quad_VAO);
	glDrawArrays(GL_TRIANGLES, 0, 6);
	glBindVertexArray(0);
	shader_screen->disable();

	glDisable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, 0);
}
#pragma endregion

int main()
{
	if (!Init())
		std::cout << "Window initiation success" << std::endl;
	else
		return 1;

	// now, not use viewer
	//Global_Variable::Init_Viewer();

	// Initiation
	Init_Particle();
	Init_Shader();
	Init_Texture();
	Init_FrameBuffer();
	Init_Quad();

	while (!glfwWindowShouldClose(window))
	{
		int display_w, display_h;
		glfwGetFramebufferSize(window, &display_w, &display_h);

		// 1. Bind Texture, Use ComputeShader Program
		Bind_Textures();
		Use_ComputeShader();

		// 2. Draw
		Draw_Framebuffer();
		Draw();

		// 3. Swap Buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

		// Drag Event 설정
		MouseDragging(display_w, display_h);

		frame_current++;
	}

	glfwDestroyWindow(window);
	glfwTerminate();

	return 0;
}