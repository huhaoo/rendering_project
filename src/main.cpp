#include <iostream>
#include <assimp/Importer.hpp>   // 导入器
#include <assimp/scene.h>        // 场景数据结构
#include <assimp/postprocess.h>  // 后处理标志

void LoadMesh(const std::string& path) ;
void ProcessNode(aiNode* node, const aiScene* scene) ;
void ProcessMesh(aiMesh* mesh, const aiScene* scene) ;

void LoadMesh(const std::string& path) {
    Assimp::Importer importer;

    // 读取 .obj 文件，应用一些基础的后处理
    const aiScene* scene = importer.ReadFile(path, 
        aiProcess_Triangulate |               // 将所有模型转换为三角面片
        // aiProcess_JoinIdenticalVertices |  // 合并相同顶点，减少冗余
        aiProcess_GenNormals  |               // 如果模型没有法线，则生成法线
		0 );

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) {
        std::cerr << "Error::Assimp: " << importer.GetErrorString() << std::endl;
        return;
    }

    std::cout << "Successfully loaded " << path << std::endl;
    
    // 遍历场景中的每个节点
    ProcessNode(scene->mRootNode, scene);
}

void ProcessNode(aiNode* node, const aiScene* scene) {
    // 遍历节点中的每个 mesh
    for (unsigned int i = 0; i < node->mNumMeshes; i++) {
        aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
        ProcessMesh(mesh, scene);
    }

    // 递归处理子节点
    for (unsigned int i = 0; i < node->mNumChildren; i++) {
        ProcessNode(node->mChildren[i], scene);
    }
}

void ProcessMesh(aiMesh* mesh, const aiScene* scene) {
    std::cout << "Mesh has " << mesh->mNumVertices << " vertices." << std::endl;

    // 遍历顶点
    for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
        aiVector3D position = mesh->mVertices[i];
        std::cout << "Vertex[" << i << "]: (" 
                  << position.x << ", " << position.y << ", " << position.z << ")" 
                  << std::endl;
    }

    // 如果有法线，输出法线数据
    if (mesh->HasNormals()) {
        for (unsigned int i = 0; i < mesh->mNumVertices; i++) {
            aiVector3D normal = mesh->mNormals[i];
            std::cout << "Normal[" << i << "]: ("
                      << normal.x << ", " << normal.y << ", " << normal.z << ")"
                      << std::endl;
        }
    }

    // 如果有面索引数据，输出面信息
    for (unsigned int i = 0; i < mesh->mNumFaces; i++) {
        aiFace face = mesh->mFaces[i];
        std::cout << "Face[" << i << "] has " << face.mNumIndices << " indices: ";
        for (unsigned int j = 0; j < face.mNumIndices; j++) {
            std::cout << face.mIndices[j] << " ";
        }
        std::cout << std::endl;
    }
}

int main(int argc, char** argv) {
    printf("%s",argv[1]);
    std::string modelPath = argv[1];
    LoadMesh(modelPath);
    return 0;
}