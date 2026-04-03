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
#include <stdlib.h>
#include <time.h>

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
bool canUpdatePosition = true;
//float maxDistanceFromOrigin = 50.0f;
float maxDistanceFromOrigin = 100.0f;
float maxSpeedPerAxis = 5.0f;
//const int NUMBER_OF_OBJECTS = 250;
const int NUMBER_OF_OBJECTS = 5000;

enum Algorithm {
    SPATIAL_HASH_GRID,
    BVH,
    BRUTE_FORCE
};

Algorithm currentAlgorithm = SPATIAL_HASH_GRID;

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

struct DOP26 {
    static const glm::vec3 axes[13];

    float planeMin[13];
    float planeMax[13];

    PBRModel* model;
    glm::mat4 modelToWorldMat;
    DOP26(PBRModel* model, glm::mat4 modelToWorldMat) : model(model), modelToWorldMat(modelToWorldMat) {
        calculateBounds();
    }

    void translates(float* planeTranslatedMin, float* planeTranslatedMax, glm::vec3 v) {
        for (int i = 0; i < 13; i++) {
            float d = glm::dot(glm::normalize(axes[i]), v);
            planeTranslatedMin[i] = planeMin[i] + d;
            planeTranslatedMax[i] = planeMax[i] + d;
        }
    }

    void calculateBounds() {
        for (int i = 0; i < 13; i++) {
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

const glm::vec3 DOP26::axes[13] = {
    { 1.0f, 0.0f, 0.0f },
    { 0.0f, 1.0f, 0.0f },
    { 0.0f, 0.0f, 1.0f },

    { 1.0f, 1.0f, 0.0f },
    { 1.0f, -1.0f, 0.0f },
    { 1.0f, 0.0f, 1.0f },
    { 1.0f, 0.0f, -1.0f },
    { 0.0f, 1.0f, 1.0f },
    { 0.0f, 1.0f, -1.0f },

    { 1.0f, 1.0f, 1.0f },
    { -1.0f, 1.0f, 1.0f },
    { 1.0f, -1.0f, 1.0f },
    { 1.0f, 1.0f, -1.0f }
};

struct BoundingVolumeObject {
    PBRModel* objectModel;
    DOP26* boundingVolume;
    glm::vec3 position;
    glm::vec3 velocity;

    bool collided;

    float objectPlaneMin[13];
    float objectPlaneMax[13];

    BoundingVolumeObject(PBRModel* model, DOP26* boundingVolume): 
        objectModel(model), boundingVolume(boundingVolume), position(0.0f), velocity(0.0f), collided(false) 
    {
        for (int i = 0; i < 13; i++) {
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

bool DOP26Test(const BoundingVolumeObject& a, const BoundingVolumeObject& b) {
    for (int i = 0; i < 13; i++) {
        if (a.objectPlaneMin[i] > b.objectPlaneMax[i] || a.objectPlaneMax[i] < b.objectPlaneMin[i]) {
            return false;
        }
    }
    return true;
}

unsigned int planeVAO = 0;
unsigned int planeVBO = 0;
unsigned int planeEBO = 0;

void visualizeDOP8Planes(const BoundingVolumeObject& object, Shader& shader) {
    shader.use();
    shader.setBool("useColor", true);
    shader.setVec3("color", object.collided ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(1.0f));
    shader.setFloat("opacity", 0.5f);

    glDisable(GL_CULL_FACE);
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 2; j++) {
            glm::vec3 axis = glm::normalize(DOP8::axes[i]);
            glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
            glm::mat4 rotMat(1.0f);

            float dot = glm::dot(up, axis);
            if (std::abs(dot) < 0.99f) {
                glm::vec3 rotationalAxis = glm::normalize(glm::cross(up, axis));
                float angle = acos(glm::dot(glm::vec3(0.0f, 1.0f, 0.0f), axis));
                rotMat = glm::rotate(glm::mat4(1.0f), angle, rotationalAxis);
            }
            else if (dot < -0.99f) {
                rotMat = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            }
            else {
                rotMat = glm::rotate(glm::mat4(1.0f), glm::radians(-180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            }

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

void visualizeDOP26Planes(const BoundingVolumeObject& object, Shader& shader) {
    shader.use();
    shader.setBool("useColor", true);
    shader.setVec3("color", object.collided ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::vec3(1.0f));
    shader.setFloat("opacity", 0.5f);

    glDisable(GL_CULL_FACE);
    for (int i = 0; i < 13; i++) {
        for (int j = 0; j < 2; j++) {
            glm::vec3 axis = glm::normalize(DOP26::axes[i]);
            glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
            glm::mat4 rotMat(1.0f);

            float dot = glm::dot(up, axis);
            if (std::abs(dot) < 0.99f) {
                glm::vec3 rotationalAxis = glm::normalize(glm::cross(up, axis));
                float angle = acos(glm::dot(glm::vec3(0.0f, 1.0f, 0.0f), axis));
                rotMat = glm::rotate(glm::mat4(1.0f), angle, rotationalAxis);
            }
            else if (dot < -0.99f) {
                rotMat = glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(1.0f, 0.0f, 0.0f));
            }

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
        return obj1.distanceToCamera < obj2.distanceToCamera;
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
        //visualizeDOP8Planes(*obj.object, shader);
        visualizeDOP26Planes(*obj.object, shader);
    }
    glDisable(GL_BLEND);
    glDepthMask(GL_TRUE);
}

float randFloat() {
    return (float)rand() / (float)RAND_MAX;
}

struct SpatialHashGrid {
    static const float MAX_DISTANCE;
    float spacing;
    int tableSize;
    std::vector<int> cellStart;
    std::vector<int> cellEntries;
    std::vector<int> queryIds;
    int querySize;
    SpatialHashGrid(float spacing, int maxNumberOfObject): spacing(spacing), tableSize(maxNumberOfObject * 2), querySize(0)  {
        cellStart = std::vector<int>(tableSize + 1);
        cellEntries = std::vector<int>(maxNumberOfObject);
        queryIds = std::vector<int>(maxNumberOfObject);
    }

    int coordToHash(int x, int y, int z) {
        int h = (x * 92837111) ^ (y * 689287499) ^ (z * 283923481);
        return std::abs(h) % tableSize;
    }

    int floatToIntCoord(float coord) {
        return std::floor(coord / spacing);
    }

    int getHashFromPosition(glm::vec3 position) {
        return coordToHash(
            floatToIntCoord(position.x),
            floatToIntCoord(position.y),
            floatToIntCoord(position.z)
        );
    }

    void createHashGrid(const std::vector<BoundingVolumeObject>& objects) {
        int numberOfObjects = objects.size();
        int n = cellStart.size();
        // initialize to zeroes
        for (int i = 0; i < n; i++) {
            cellStart[i] = 0;

            if (i < cellEntries.size()) {
                cellEntries[i] = 0;
            }
        }

        // count number of objects present in each cells
        for (int i = 0; i < numberOfObjects; i++) {
            int hash = getHashFromPosition(objects[i].position);
            cellStart[hash]++;
        }

        // compute partial sum
        // e.g.
        // before -> cellStart: 0 0 2 0 4 0 1
        // after  -> cellStart: 0 0 2 2 6 6 7
        // each number points to the last cell entry + 1
        int start = 0;
        for (int i = 0; i < tableSize; i++) {
            start += cellStart[i];
            cellStart[i] = start;
        }
        cellStart[tableSize] = start;


        for (int i = 0; i < numberOfObjects; i++) {
            int hash = getHashFromPosition(objects[i].position); // get hash to each starting index of cell in cellStart
            cellStart[hash]--; // -1 because each each number points to cell entry + 1
            
            // use the starting index from cellStart as an index for cellEntries to set the starting index 
            // for looping through the objects contained in each cell
            cellEntries[cellStart[hash]] = i;
        }
    }

    void query(glm::vec3 position, float maxDistance) {
        int startX = floatToIntCoord(position.x - maxDistance);
        int startY = floatToIntCoord(position.y - maxDistance);
        int startZ = floatToIntCoord(position.z - maxDistance);

        int endX = floatToIntCoord(position.x + maxDistance);
        int endY = floatToIntCoord(position.y + maxDistance);
        int endZ = floatToIntCoord(position.z + maxDistance);

        querySize = 0;

        // adding objects to be queried for use
        for (int x = startX; x <= endX; x++) {
            for (int y = startY; y <= endY; y++) {
                for (int z = startZ; z <= endZ; z++) {
                    int hash = coordToHash(x, y, z);
                    int start = cellStart[hash];
                    int end = cellStart[hash + 1];

                    for (int i = start; i < end; i++) {
                        queryIds[querySize] = cellEntries[i]; // cellEntries[i] returns the object index in the object list
                        querySize++;
                    }
                }
            }
        }
    }
};
const float SpatialHashGrid::MAX_DISTANCE = 2.0f;

struct MortonCode {
    static unsigned int expandBits(unsigned int bits) {
        bits = (bits * 0x00010001) & 0xFF0000FF;
        bits = (bits * 0x00000101) & 0x0F00F00F;
        bits = (bits * 0x00000011) & 0xC30C30C3;
        bits = (bits * 0x00000005) & 0x49249249;
        return bits;
    }

    static unsigned int calculateMortonCode(glm::vec3 position) {
        // normalize coordinates to [0, 1] based on world size

        float maxWorldSize = maxDistanceFromOrigin * 2.0f;

        glm::vec3 normalizedCoord = glm::vec3(
            (position.x + maxWorldSize / 2.0f) / maxWorldSize,
            (position.y + maxWorldSize / 2.0f) / maxWorldSize,
            (position.z + maxWorldSize / 2.0f) / maxWorldSize 
        );

        normalizedCoord = glm::clamp(normalizedCoord, glm::vec3(0.0f), glm::vec3(1.0f)); // clamp within [0, 1]

        // scale to range [0, 1023] for 10 bit encoding
        unsigned int x = std::min((int)std::floor(normalizedCoord.x * 1023), 1023);
        unsigned int y = std::min((int)std::floor(normalizedCoord.y * 1023), 1023);
        unsigned int z = std::min((int)std::floor(normalizedCoord.z * 1023), 1023);

        x = expandBits(x);
        y = expandBits(y);
        z = expandBits(z);

        return x | (y << 1) | (z << 2);
    }
};

struct AABB {
    glm::vec3 min;
    glm::vec3 max;
    AABB() : min(0.0f), max(0.0f) {}
    AABB(glm::vec3 min, glm::vec3 max) : min(min), max(max) {}
    AABB(const BoundingVolumeObject& object) {
        // axes with indices 0, 1, 2 are standard axes
        // { 1.0f, 0.0f, 0.0f },
        // { 0.0f, 1.0f, 0.0f },
        // { 0.0f, 0.0f, 1.0f },

        min = glm::vec3(
            object.objectPlaneMin[0], 
            object.objectPlaneMin[1], 
            object.objectPlaneMin[2]
        );
        
        max = glm::vec3(
            object.objectPlaneMax[0],
            object.objectPlaneMax[1],
            object.objectPlaneMax[2]
        );
    }
    bool checkAABB(const AABB& other) {
        return(
            this->min.x <= other.max.x && this->max.x >= other.min.x &&
            this->min.y <= other.max.y && this->max.y >= other.min.y &&
            this->min.z <= other.max.z && this->max.z >= other.min.z
        );
    }
};

bool checkAABB(const AABB& a, const AABB& b) {
    return(
        a.min.x <= b.max.x && a.max.x >= b.min.x &&
        a.min.y <= b.max.y && a.max.y >= b.min.y &&
        a.min.z <= b.max.z && a.max.z >= b.min.z
    );
}

using NodeId = int;
struct BVHNode {
    //BVHNode* left;
    //BVHNode* right;
    NodeId leftId;
    NodeId rightId;
    int objectIdx;
    AABB box;
    //BVHNode(): left(nullptr), right(nullptr), objectIdx(-1) {}
    //BVHNode(int objectIdx): left(nullptr), right(nullptr), objectIdx(objectIdx) {}
    BVHNode() : leftId(-1), rightId(-1), objectIdx(-1) {}
    BVHNode(int objectIdx) : leftId(-1), rightId(-1), objectIdx(objectIdx) {}
    bool isLeaf() { return objectIdx >= 0; }
};

template <int N>
class NodePool {
    private:
    //std::vector<BVHNode> nodes;
    BVHNode nodes[2 * N - 1];
    int count;
    int maxSize;

    public:
    //NodePool(int n) {
    NodePool() {
        //maxSize = 2 * n - 1;
        //nodes = std::vector<BVHNode>(maxSize);
        maxSize = N;
        count = 0;
    }

    int createNode() {
        assert(count < maxSize && "Maximum pool capacity reached");

        int index = count++;
        nodes[index] = BVHNode();
        return index;
    }

    int createNode(int objectIdx) {
        assert(count < maxSize && "Maximum pool capacity reached");

        int index = count++;
        nodes[index] = BVHNode(objectIdx);
        return index;
    }

    BVHNode* getNode(int index) {
        if (index < 0 || index >= count) return nullptr;
        return &nodes[index];
    }

    BVHNode* operator[](int index) {
        return getNode(index);
    }

    void reset() {
        count = 0;
    }
};
//static NodePool nodePool = NodePool(NUMBER_OF_OBJECTS);
static NodePool nodePool = NodePool<NUMBER_OF_OBJECTS>();

struct IdData {
    int index;
    unsigned int mortonCode;
    IdData(): index(-1), mortonCode(0) {}
    IdData(int index, unsigned int mortonCode) : index(index), mortonCode(mortonCode) {}
};

int getSplitIndex(int start, int end) {
    return ((end - start) / 2) + start;
}

NodeId createLeaf(int objectIdx, AABB box) {
    NodeId nodeId = nodePool.createNode(objectIdx);
    BVHNode* node = nodePool[nodeId];

    node->box = box;

    return nodeId;
}

NodeId createNode() {
    return nodePool.createNode();
}

NodeId createSubTree(const std::vector<BoundingVolumeObject>& objects, const std::vector<IdData>& list, int start, int end) {
    if (start >= end) {
        return createLeaf(list[start].index, objects[list[start].index]);
    }

    int mid = getSplitIndex(start, end);
    NodeId nodeId = createNode();
    NodeId leftId = createSubTree(objects, list, start, mid);
    NodeId rightId = createSubTree(objects, list, mid + 1, end);

    BVHNode* node = nodePool[nodeId];
    node->leftId = leftId;
    node->rightId = rightId;

    node->box.min.x = std::min(nodePool[node->leftId]->box.min.x, nodePool[node->rightId]->box.min.x);
    node->box.min.y = std::min(nodePool[node->leftId]->box.min.y, nodePool[node->rightId]->box.min.y);
    node->box.min.z = std::min(nodePool[node->leftId]->box.min.z, nodePool[node->rightId]->box.min.z);

    node->box.max.x = std::max(nodePool[node->leftId]->box.max.x, nodePool[node->rightId]->box.max.x);
    node->box.max.y = std::max(nodePool[node->leftId]->box.max.y, nodePool[node->rightId]->box.max.y);
    node->box.max.z = std::max(nodePool[node->leftId]->box.max.z, nodePool[node->rightId]->box.max.z);

    return nodeId;
}

bool compareIdData(const IdData& a, const IdData& b) {
    return a.mortonCode < b.mortonCode;
}

NodeId createTree(const std::vector<BoundingVolumeObject>& objects) {
    int n = (int)objects.size();

    if (n == 0) return -1;

    std::vector<IdData> list(n);
    for (int i = 0; i < n; i++) {
        unsigned int mortonCode = MortonCode::calculateMortonCode(objects[i].position);
        list[i] = IdData(i, mortonCode);
    }

    std::sort(list.begin(), list.end(), compareIdData);

    return createSubTree(objects, list, 0, list.size() - 1);
}

void checkCollisionsBVH(int objectIdx, std::vector<BoundingVolumeObject>& objects, NodeId nodeId) {
    BVHNode* node = nodePool[nodeId];
    if (!checkAABB(AABB(objects[objectIdx]), node->box)) return;

    if (node->isLeaf()) {
        if (node->objectIdx != objectIdx) {
            BoundingVolumeObject& obj1 = objects[node->objectIdx];
            BoundingVolumeObject& obj2 = objects[objectIdx];
            if (DOP26Test(obj1, obj2)) {
                obj1.collided = true;
                obj2.collided = true;
            }
        }

        return;
    }

    checkCollisionsBVH(objectIdx, objects, node->leftId);
    checkCollisionsBVH(objectIdx, objects, node->rightId);
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
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor();
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "KDOP_BoundingVolume", NULL, NULL);
    //GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "KDOP_BoundingVolume", primaryMonitor, NULL);
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

    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    glfwSwapInterval(0);

    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    stbi_set_flip_vertically_on_load(false);

    srand(time(NULL));

    glEnable(GL_DEPTH_TEST);

    Shader simpleShader("1.2.pbr.vs", "frag.fs");
    simpleShader.setInt("texture_PBR_diffuse1", 0);
    simpleShader.setInt("texture_PBR_normal1", 1);
    simpleShader.setInt("texture_PBR_metallic1", 2);
    simpleShader.setInt("texture_PBR_roughness1", 3);
    simpleShader.setInt("texture_PBR_ambient_occlusion1", 4);

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
    PBRModel swordModel(FileSystem::getPath("resources/objects/sword/scene.gltf"));
    PBRModel scytheModel(FileSystem::getPath("resources/objects/scythe/scene.gltf"));
    PBRModel shotgunModel(FileSystem::getPath("resources/objects/shotgun/scene.gltf"));
    //PBRModel chisaModel(FileSystem::getPath("resources/objects/chisa/scene.gltf"));

    glm::mat4 ballModelMat(1.0f);
    ballModelMat = glm::translate(ballModelMat, glm::vec3(0.0f, 0.0f, 0.0f));
    ballModelMat = glm::scale(ballModelMat, glm::vec3(0.05f));
    ballModelMat = glm::rotate(ballModelMat, glm::radians(-90.0f), glm::vec3(1, 0, 0));

    glm::mat4 swordModelMat(1.0f);
    swordModelMat = glm::translate(swordModelMat, glm::vec3(1.0f, 0.4f, 0.0f));
    swordModelMat = glm::scale(swordModelMat, glm::vec3(0.65));
    swordModelMat = glm::rotate(swordModelMat, glm::radians(-90.0f), glm::vec3(0, 1, 0));

    glm::mat4 shotgunModelMat(1.0f);
    shotgunModelMat = glm::scale(shotgunModelMat, glm::vec3(10.0f));
    shotgunModelMat = glm::rotate(shotgunModelMat, glm::radians(-90.0f), glm::vec3(1, 0, 0));

    glm::mat4 scytheModelMat(1.0f);
    scytheModelMat = glm::scale(scytheModelMat, glm::vec3(0.075));
    scytheModelMat = glm::rotate(scytheModelMat, glm::radians(-90.0f), glm::vec3(1, 0, 0));

    glm::mat4 chisaModelMat(1.0f);
    chisaModelMat = glm::scale(ballModelMat, glm::vec3(2.5f));

    DOP26 ballDOP = DOP26(&ballModel, ballModelMat);
    DOP26 swordDOP = DOP26(&swordModel, swordModelMat);
    DOP26 shotgunDOP = DOP26(&shotgunModel, shotgunModelMat);
    DOP26 scytheDOP = DOP26(&scytheModel, scytheModelMat);
    //DOP26 chisaDOP = DOP26(&chisaModel, chisaModelMat);

    DOP26* dops[4] = {
        &ballDOP,
        &swordDOP,
        &shotgunDOP,
        &scytheDOP,
        //&chisaDOP
    };

    PBRModel* models[4] = {
        &ballModel,
        &swordModel,
        &shotgunModel,
        &scytheModel,
        //&chisaModel
    };

    for (int i = 0; i < NUMBER_OF_OBJECTS; i++) {
        float distanceFromOrigin = 0.0f;
        glm::vec3 position = glm::vec3(0.0f);
        do {
            position = glm::vec3(
                2.0f * maxDistanceFromOrigin * randFloat() - maxDistanceFromOrigin,
                2.0f * maxDistanceFromOrigin * randFloat() - maxDistanceFromOrigin,
                2.0f * maxDistanceFromOrigin * randFloat() - maxDistanceFromOrigin
            );
            distanceFromOrigin = glm::length(position - glm::vec3(0.0f));
        } while (distanceFromOrigin > maxDistanceFromOrigin);

        BoundingVolumeObject object = BoundingVolumeObject(models[i % 4], dops[i % 4]);
        object.position = position;
        object.velocity = glm::vec3(
            2.0f * maxSpeedPerAxis * randFloat() - maxSpeedPerAxis,
            2.0f * maxSpeedPerAxis * randFloat() - maxSpeedPerAxis,
            2.0f * maxSpeedPerAxis * randFloat() - maxSpeedPerAxis
        );
        objects.emplace_back(object);
    }

    // draw in wireframe
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    camera.MovementSpeed = 4.0f;

    SpatialHashGrid spatialHashGrid(4.0f, (int)objects.size());

    NodeId bvhRootNodeId = -1;

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
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // update
        if (canUpdatePosition) {
            for (BoundingVolumeObject& object : objects) {
                object.update(deltaTime);

                glm::vec3 directionFromOrigin = object.position - glm::vec3(0.0f);
                float distanceFromOrigin = glm::length(directionFromOrigin);
                if (distanceFromOrigin > maxDistanceFromOrigin) {
                    directionFromOrigin = glm::normalize(directionFromOrigin);

                    object.position = directionFromOrigin * (distanceFromOrigin - 1.0f);

                    object.velocity = -object.velocity;
                }

                object.collided = false;
            }
        }

        // collision detection
        int numOfObjects = objects.size();
        switch (currentAlgorithm) {
            case SPATIAL_HASH_GRID:
                spatialHashGrid.createHashGrid(objects);
                for (int i = 0; i < numOfObjects; i++) {
                    spatialHashGrid.query(objects[i].position, SpatialHashGrid::MAX_DISTANCE);
                    for (int query = 0; query < spatialHashGrid.querySize; query++) {
                        int j = spatialHashGrid.queryIds[query];

                        BoundingVolumeObject& obj1 = objects[i];
                        BoundingVolumeObject& obj2 = objects[j];

                        if (i != j && DOP26Test(obj1, obj2)) {
                            obj1.collided = true;
                            obj2.collided = true;
                        }
                    }
                }
                break;

            case BVH:
                nodePool.reset();
                bvhRootNodeId = createTree(objects);
                for (int i = 0; i < numOfObjects; i++) {
                    if (bvhRootNodeId == -1) break;
                    checkCollisionsBVH(i, objects, bvhRootNodeId);
                }
                break;

            case BRUTE_FORCE:
                for (int i = 0; i < numOfObjects; i++) {
                    for (int j = i + 1; j < numOfObjects; j++) {
                        BoundingVolumeObject& obj1 = objects[i];
                        BoundingVolumeObject& obj2 = objects[j];

                        if (DOP26Test(obj1, obj2)) {
                            obj1.collided = true;
                            obj2.collided = true;
                        }
                    }
                }
                break;
        }


        // rendering
        simpleShader.use();

        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 1000.0f);
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

        // Show Average FPS
        static unsigned int frameNum = 0;
        static double timeElapsed = 0.0;
        static double fps = 0.0;

        timeElapsed += deltaTime;
        frameNum++;

        if (timeElapsed >= 1.0f) { 
            fps = frameNum / timeElapsed;
            timeElapsed = 0.0f;
            frameNum = 0;
            std::string algorithm;
            switch (currentAlgorithm) {
                case SPATIAL_HASH_GRID:
                    algorithm = "Spatial Hash Grid";
                    break;

                case BVH:
                    algorithm = "BVH";
                    break;

                case BRUTE_FORCE:
                    algorithm = "Brute Force";
                    break;
            }

            std::cout << "FPS: " << fps << " Method: " << algorithm << std::endl;
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

    if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
        camera.MovementSpeed = 16.0f;
    else 
        camera.MovementSpeed = 4.0f;

    if (getKeyDown(window, GLFW_KEY_SPACE)) {
        showVisualization = !showVisualization;
    }

    if (getKeyDown(window, GLFW_KEY_X)) {
        canUpdatePosition = !canUpdatePosition;
    }

    if (getKeyDown(window, GLFW_KEY_H)) {
        currentAlgorithm = (Algorithm)((currentAlgorithm + 1) % 3);
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