#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <filesystem.h>
#include <shader_m.h>
#include <camera.h>
#include "PBRModel.h"

#include <limits>

#include <iostream>

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void processInput(GLFWwindow* window);

// settings
//const unsigned int SCR_WIDTH = 800;
//const unsigned int SCR_HEIGHT = 600;
//const unsigned int SCR_WIDTH = 1920;
//const unsigned int SCR_HEIGHT = 1080;
const unsigned int SCR_WIDTH = 1600;
const unsigned int SCR_HEIGHT = 900;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

struct DOP8 {
    float planeMin[4];
    float planeMax[4];
    PBRModel* model;
    glm::mat4 modelMat;
    DOP8(PBRModel* model): model(model), modelMat(1.0f) {
        calculateBounds();
    }

    void translate(glm::vec3 v) {
        glm::vec3 axes[4] = {
            { 1.0f, 1.0f, 1.0f },
            { -1.0f, 1.0f, 1.0f },
            { 1.0f, -1.0f, 1.0f },
            { -1.0f, -1.0f, 1.0 }
        };

        for (int i = 0; i < 4; i++) {
            float d = glm::dot(glm::normalize(axes[i]), v);
            planeMin[i] += d;
            planeMax[i] += d;
        }   
    }

    void calculateBounds() {
        glm::vec3 axes[4] = {
            { 1.0f, 1.0f, 1.0f },
            { -1.0f, 1.0f, 1.0f },
            { 1.0f, -1.0f, 1.0f },
            { -1.0f, -1.0f, 1.0 }
        };

        for (int i = 0; i < 4; i++) {
            axes[i] = glm::normalize(axes[i]);
            float min = std::numeric_limits<float>::max();
            float max = std::numeric_limits<float>::lowest();

            for (const PBRMesh& mesh : model->meshes) {
                for (const Vertex& vertex : mesh.vertices) {
                    glm::vec3 position = modelMat * glm::vec4(vertex.Position, 1.0f);
                    float d = glm::dot(axes[i], position);
                    min = std::min(min, d);
                    max = std::max(max, d);
                }
            }

            planeMin[i] = min;
            planeMax[i] = max;
        }
    }
};


int PBRMesh::maxTextureNumber = 0;

unsigned int PBRMesh::defaultAlbedo = 0;
unsigned int PBRMesh::defaultNormal = 0;
unsigned int PBRMesh::defaultMetallic = 0;
unsigned int PBRMesh::defaultRoughness = 0;
unsigned int PBRMesh::defaultAO = 0;

int main()
{
    // glfw: initialize and configure
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    //GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", primaryMonitor, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // tell stb_image.h to flip loaded texture's on the y-axis (before loading model).
    stbi_set_flip_vertically_on_load(false);

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);

    // build and compile shaders
    // -------------------------
    //Shader ourShader("1.model_loading.vs", "1.model_loading.fs");
    Shader ourShader("1.2.pbr.vs", "frag.fs");
    ourShader.setInt("texture_PBR_diffuse1", 0);
    ourShader.setInt("texture_PBR_normal1", 1);
    ourShader.setInt("texture_PBR_metallic1", 2);
    ourShader.setInt("texture_PBR_roughness1", 3);
    ourShader.setInt("texture_PBR_ambient_occlusion1", 4);
    //Shader ourShader("1.2.pbr.vs", "1.2.pbr.fs");

    // load models
    // -----------
    //Model ourModel(FileSystem::getPath("resources/objects/mask/source/mask.fbx"));
    //PBRModel ourModel(FileSystem::getPath("resources/objects/wooden_chest/scene.gltf"));
    PBRModel ourModel(FileSystem::getPath("resources/objects/basketball/scene.gltf"));
    //PBRModel chisaModel(FileSystem::getPath("resources/objects/chisa/scene.gltf"));
    //PBRModel swordModel(FileSystem::getPath("resources/objects/sword/scene.gltf"));
    //Model ourModel(FileSystem::getPath("resources/objects/wheel/wheel.fbx"));

    glm::vec3 lightPositions[4] = {
    glm::vec3(0.0f, 0.0f, 2.0f),
    //glm::vec3(0.0f, 0.0f, 2.0f),
    //glm::vec3(0.0f, 0.0f, 2.0f),
    //glm::vec3(0.0f, 0.0f, 2.0f),
    glm::vec3(0.0f, 0.0f, -10.0f),
    glm::vec3(5.0f, 2.0f, 5.0f),
    glm::vec3(0.0f, 0.0f, 10.0f),
    };
    glm::vec3 lightColors[4] = {
        glm::vec3(150.0f, 150.0f, 150.0f),
        glm::vec3(150.0f, 150.0f, 150.0f),
        glm::vec3(150.0f, 150.0f, 150.0f),
        glm::vec3(150.0f, 150.0f, 150.0f),
    };

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    DOP8 dop8(&ourModel);
    dop8.calculateBounds();
    for (int i = 0; i < 4; i++) {
        std::cout << "min " << i << ": " << dop8.planeMin[i] << std::endl;
        std::cout << "max " << i << ": " << dop8.planeMax[i] << std::endl;
    }

    // draw in wireframe
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // render loop
    // -----------
    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // input
        // -----
        processInput(window);

        // render
        // ------
        glClearColor(0.05f, 0.05f, 0.05f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // don't forget to enable shader before setting uniforms
        ourShader.use();

        //lighting
        //for (unsigned int i = 0; i < sizeof(lightPositions) / sizeof(lightPositions[0]); ++i)
        for (unsigned int i = 0; i < 3; ++i)
        {
            glm::vec3 newPos = lightPositions[i] + glm::vec3(sin(glfwGetTime() * 5.0) * 5.0, 0.0, 0.0);
            newPos = lightPositions[i];
            ourShader.setVec3("lightPositions[" + std::to_string(i) + "]", newPos);
            ourShader.setVec3("lightColors[" + std::to_string(i) + "]", lightColors[i]);

            //std::cout << i << std::endl;
        }

        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        ourShader.setMat4("projection", projection);
        ourShader.setMat4("view", view);

        ourShader.setVec3("camPos", camera.Position);

        // render the loaded model


        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, glm::vec3(0.0f, 0.0f, 0.0f)); // translate it down so it's at the center of the scene
        //model = glm::scale(model, glm::vec3(1.0f));	// it's a bit too big for our scene, so scale it down
        model = glm::scale(model, glm::vec3(0.01f));	// it's a bit too big for our scene, so scale it down
        model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(1, 0, 0));
        ourShader.setMat4("model", model);
        dop8.modelMat = model;

        ourShader.setMat3("normalMatrix", glm::transpose(glm::inverse(glm::mat3(model))));

        ourModel.Draw(ourShader);


        //chisaModel.Draw(ourShader);
        //swordModel.Draw(ourShader);


        // glfw: swap buffers and poll IO events (keys pressed/released, mouse moved etc.)
        // -------------------------------------------------------------------------------
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // glfw: terminate, clearing all previously allocated GLFW resources.
    // ------------------------------------------------------------------
    glfwTerminate();
    return 0;
}

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
// ---------------------------------------------------------------------------------------------------------
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

// glfw: whenever the mouse moves, this callback is called
// -------------------------------------------------------
void mouse_callback(GLFWwindow* window, double xposIn, double yposIn)
{
    float xpos = static_cast<float>(xposIn);
    float ypos = static_cast<float>(yposIn);

    if (firstMouse)
    {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

    lastX = xpos;
    lastY = ypos;

    camera.ProcessMouseMovement(xoffset, yoffset);
}

// glfw: whenever the mouse scroll wheel scrolls, this callback is called
// ----------------------------------------------------------------------
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}