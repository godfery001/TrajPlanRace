/*
 * objloader.h
 * Data structures for object rendering are defined here.
 * **DO NOT CHANGE ANYTHING HERE**
 */

#ifndef OBJLOADER_H
#define OBJLOADER_H
#include <QString>
#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <GL/glu.h>
#include "XMath.h"


struct Vertex
{
    float x, y, z, w;
    Vertex(float _x, float _y, float _z, float _w = 1.0f) :
        x(_x), y(_y), z(_z), w(_w) {}
};

struct Normal
{
    float i, j, k;
    Normal(float _i, float _j, float _k) : i(_i), j(_j), k(_k) {}
};

struct Texture
{
    float x, y, z;
    Texture(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
};


struct Face
{ 
    int numIndicies;
    std::vector<int> vertexIndicies;
    std::vector<int> normalIndicies;
    std::vector<int> textureIndicies;

    std::vector<float> vValues;
    std::vector<float> vnValues;
    std::vector<float> vtValues;

    Face() : numIndicies(0) {}
};



class ObjLoader
{
public:
    ObjLoader();

    bool Load(QString objFile,QString textureFile);
    int loadObjFile(QString pathToFile);
    int loadTexture(QString textureFile);
    void resize(float k);

    unsigned int targetTexture;

    std::vector<Vertex> mVertices;
    std::vector<Normal> mNormals;
    std::vector<Texture> mTextures;
    std::vector<Face> mFaces;

    float max_x, min_x, max_y, min_y, max_z, min_z;
    float size_x, size_y, size_z;

    // final rendering
    void Render_Texture(float sx, float sy, float sz);
    void Render(float sx, float sy, float sz, float r, float g, float b);
};

#endif // OBJLOADER_H
