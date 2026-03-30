#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// debug
//#define GLM_ENABLE_EXPERIMENTAL
//#include <glm/gtx/string_cast.hpp>
#include <filesystem.h>
#include <shader_m.h>
#include <camera.h>
#include "PBRModel.h"
#include "VerticesData.h"

#include <limits>
#include <queue>
#include <map>
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

// controls
std::map<unsigned int, bool> keyDownMap;
bool showVisualization = false;

struct DOP8 {
    static const glm::vec3 axes[4];

    float planeMin[4];
    float planeMax[4];

    PBRModel* model;
    glm::mat4 modelToWorldMat;
    DOP8(PBRModel* model, glm::mat4 modelToWorldMat): model(model), modelToWorldMat(modelToWorldMat) {
        calculateBounds();
    }

    void translates(float* planeTranslatedMin, float* planeTranslatedMax, glm::vec3 v) {
        for (int i = 0; i < 4; i++) {
            float d = glm::dot(glm::normalize(axes[i]), v);
            planeTranslatedMin[i] = planeMin[i] + d;
            planeTranslatedMax[i] = planeMax[i] + d;
        }   
    }

    void calculateBounds() {
        for (int i = 0; i < 4; i++) {
            glm::vec3 axis = glm::normalize(axes[i]);
            float min = std::numeric_limits<float>::max();
            float max = std::numeric_limits<float>::lowest();

            for (const PBRMesh& mesh : model->meshes) {
                for (const Vertex& vertex : mesh.vertices) {
                    glm::vec3 position = modelToWorldMat * glm::vec4(vertex.Position, 1.0f);
                    float d = glm::dot(axis, position);
                    min = std::min(min, d);
                    max = std::max(max, d);
                }
            }

            planeMin[i] = min;
            planeMax[i] = max;
        }
    }
};
const glm::vec3 DOP8::axes[4] = {
    { 1.0f, 1.0f, 1.0f },
    { -1.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, 1.0f },
    { -1.0f, -1.0f, 1.0 }
};

struct BoundingVolumeObject {
    PBRModel* objectModel;
    DOP8* boundingVolume;
    glm::vec3 position;
    glm::vec3 velocity;

    bool collided;

    float objectPlaneMin[4];
    float objectPlaneMax[4];

    BoundingVolumeObject(PBRModel* model, DOP8* boundingVolume): 
        objectModel(model), boundingVolume(boundingVolume), position(0.0f), velocity(0.0f), collided(false) 
    {
        for (int i = 0; i < 4; i++) {
            objectPlaneMin[i] = boundingVolume->planeMin[i];
            objectPlaneMax[i] = boundingVolume->planeMax[i];
        }
    }

    void update(float dt) {
        position += velocity * dt;
        boundingVolume->translates(objectPlaneMin, objectPlaneMax, position);
    }

    void draw(Shader& shader) {
        shader.use();
        glm::mat4 model(1.0f);
        model = glm::translate(model, position) * boundingVolume->modelToWorldMat;
        shader.setMat4("model", model);
        shader.setMat3("normalMatrix", glm::transpose(glm::inverse(glm::mat3(model))));

        objectModel->Draw(shader);
    }
};


bool DOP8Test(const BoundingVolumeObject& a, const BoundingVolumeObject& b) {
    for (int i = 0; i < 4; i++) {
        if (a.objectPlaneMin[i] > b.objectPlaneMax[i] || a.objectPlaneMax[i] < b.objectPlaneMin[i]) {
            return false;
        }
    }
    return true;
}

unsigned int planeVAO = 0;
unsigned int planeVBO = 0;
unsigned int planeEBO = 0;

void visualizeDOPPlanes(const BoundingVolumeObject& object, Shader& shader) {
    shader.use();
    shader.setBool("useColor", true);
    shader.setVec3("color", object.collided ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(1.0f));
    shader.setFloat("opacity", 0.5f);

    glDisable(GL_CULL_FACE);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            glm::vec3 axis = glm::normalize(DOP8::axes[i]);
            glm::vec3 rotationalAxis = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), axis));
            float angle = acos(glm::dot(glm::vec3(0.0f, 1.0f, 0.0f), axis));
            glm::mat4 rotMat = glm::rotate(glm::mat4(1.0f), angle, rotationalAxis);

            float d = j > 0 ? object.boundingVolume->planeMin[i] : object.boundingVolume->planeMax[i];
            //float d = object.objectPlaneMax[i];

            glm::mat4 model(1.0f);
            model = glm::translate(model, object.position) * rotMat;
            model = glm::scale(model, glm::vec3(1.8f, 1.0f, 1.8f));
            model = glm::translate(model, glm::vec3(0.0f, d, 0.0f));

            shader.setMat4("model", model);
            shader.setMat3("normalMatrix", glm::transpose(glm::inverse(glm::mat3(model))));

            glBindVertexArray(planeVAO);
            glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }

    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    shader.setBool("useColor", false);
}

struct TransparencyRenderingObject {
    BoundingVolumeObject* object;
    float distanceToCamera;
    TransparencyRenderingObject(BoundingVolumeObject* object, float distanceToCamera): object(object), distanceToCamera(distanceToCamera) {}
};

struct TransparencyComparator {
    bool operator()(const TransparencyRenderingObject& obj1, const TransparencyRenderingObject& obj2) {
        return obj1.distanceToCamera > obj2.distanceToCamera;
    }
};

void renderPlanes(std::vector<BoundingVolumeObject>& volumeObjects, Shader& shader) {
    std::priority_queue<TransparencyRenderingObject, std::vector<TransparencyRenderingObject>, TransparencyComparator> transparencyRenderQueue;
    for (BoundingVolumeObject& object : volumeObjects) {
        transparencyRenderQueue.emplace(TransparencyRenderingObject(&object, glm::length(object.position - camera.Position)));
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    while (!transparencyRenderQueue.empty()) {
        TransparencyRenderingObject obj = transparencyRenderQueue.top();
        transparencyRenderQueue.pop();
        visualizeDOPPlanes(*obj.object, shader);
    }
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

std::vector<BoundingVolumeObject> objects;

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
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "KDOP_BoundingVolume", NULL, NULL);
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
    Shader simpleShader("1.2.pbr.vs", "frag.fs");
    simpleShader.setInt("texture_PBR_diffuse1", 0);
    simpleShader.setInt("texture_PBR_normal1", 1);
    simpleShader.setInt("texture_PBR_metallic1", 2);
    simpleShader.setInt("texture_PBR_roughness1", 3);
    simpleShader.setInt("texture_PBR_ambient_occlusion1", 4);
    //Shader ourShader("1.2.pbr.vs", "1.2.pbr.fs");

    // load models
    // -----------
    //Model ourModel(FileSystem::getPath("resources/objects/mask/source/mask.fbx"));
    //PBRModel ourModel(FileSystem::getPath("resources/objects/wooden_chest/scene.gltf"));
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

    // Setting up plane object
    glGenVertexArrays(1, &planeVAO);
    glBindVertexArray(planeVAO);
    glGenBuffers(1, &planeVBO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(
        GL_ARRAY_BUFFER, 
        sizeof(float) * 4 * 6,
        PLANE_VERTICES,
        GL_STATIC_DRAW
    );
    // positions
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // normals
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glGenBuffers(1, &planeEBO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeEBO);
    glBufferData(
        GL_ELEMENT_ARRAY_BUFFER,
        6 * sizeof(unsigned int),
        PLANE_INDICES,
        GL_STATIC_DRAW
    );
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    // setting up bounding objects
    PBRModel ballModel(FileSystem::getPath("resources/objects/basketball/scene.gltf"));
    PBRModel chisaModel(FileSystem::getPath("resources/objects/chisa/scene.gltf"));

    glm::mat4 ballModelMat(1.0f);
    ballModelMat = glm::translate(ballModelMat, glm::vec3(0.0f, 0.0f, 0.0f));
    ballModelMat = glm::scale(ballModelMat, glm::vec3(0.01f));
    ballModelMat = glm::rotate(ballModelMat, glm::radians(-90.0f), glm::vec3(1, 0, 0));

    glm::mat4 chisaModelMat(1.0f);
    chisaModelMat = ballModelMat;

    //std::cout << glm::to_string(ballModelMat) << std::endl;

    DOP8 ballDOP = DOP8(&ballModel, ballModelMat);
    DOP8 chisaDOP = DOP8(&chisaModel, chisaModelMat);

    objects.emplace_back(BoundingVolumeObject(&ballModel, &ballDOP));
    objects.emplace_back(BoundingVolumeObject(&chisaModel, &chisaDOP));

    // draw in wireframe
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

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

        // update
        for (BoundingVolumeObject& object : objects) {
            object.update(deltaTime);
            object.collided = false;
        }

        // collision detection
        int numOfObjects = objects.size();
        for (int i = 0; i < numOfObjects; i++) {
            for (int j = i + 1; j < numOfObjects; j++) {
                BoundingVolumeObject& obj1 = objects[i];
                BoundingVolumeObject& obj2 = objects[j];

                if (DOP8Test(obj1, obj2)) {
                    obj1.collided = true;
                    obj2.collided = true;
                }
            }
        }

        // rendering
        simpleShader.use();

        //lighting
        //for (unsigned int i = 0; i < sizeof(lightPositions) / sizeof(lightPositions[0]); ++i)
        for (unsigned int i = 0; i < 3; ++i)
        {
            glm::vec3 newPos = lightPositions[i] + glm::vec3(sin(glfwGetTime() * 5.0) * 5.0, 0.0, 0.0);
            newPos = lightPositions[i];
            simpleShader.setVec3("lightPositions[" + std::to_string(i) + "]", newPos);
            simpleShader.setVec3("lightColors[" + std::to_string(i) + "]", lightColors[i]);

            //std::cout << i << std::endl;
        }


        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();
        simpleShader.setMat4("projection", projection);
        simpleShader.setMat4("view", view);
        simpleShader.setVec3("camPos", camera.Position);

        for (BoundingVolumeObject& object : objects) {
            simpleShader.setVec3("color", object.collided ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(1.0f));
            object.draw(simpleShader);
        }

        if (showVisualization) {
            renderPlanes(objects, simpleShader);
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}

bool getKeyDown(GLFWwindow* window, unsigned int key) {
    // init
    if (keyDownMap.count(key) == 0) {
        keyDownMap[key] = false;
        return false;
    }

    if (glfwGetKey(window, key) == GLFW_PRESS && keyDownMap.at(key)) {
        return false;
    }

    if (glfwGetKey(window, key) == GLFW_RELEASE && keyDownMap.at(key)) {
        keyDownMap[key] = false;
        return false;
    }

    if (glfwGetKey(window, key) == GLFW_PRESS && !keyDownMap.at(key)) {
        keyDownMap[key] = true;
        return true;
    }

    return false;
}

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

    if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
        objects[0].position += camera.Right * 5.0f * deltaTime;

    if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
        objects[0].position -= camera.Right * 5.0f * deltaTime;

    if (getKeyDown(window, GLFW_KEY_SPACE)) {
        showVisualization = !showVisualization;
    }
}

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
}

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

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}