/*
 * objloader.cpp
 * 3D model loader implementation for Wavefront OBJ format
 * Supports textured and solid-color rendering modes
 */

#include <QIODevice>
#include <QDebug>
#include <QStringList>
#include <QFile>
#include <QGLWidget>
#include <QTextStream>
#include <QKeyEvent>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <QMessageBox>
#include <QApplication>
#include "objloader.h"

ObjLoader::ObjLoader()
{
    // Default constructor initializes empty model data
}

/**
 * @brief Main model loading interface
 * @param objFile Path to .obj model file
 * @param textureFile Path to texture image
 * @return true if both model and texture loaded successfully
 */
bool ObjLoader::Load(QString objFile, QString textureFile)
{
    if(loadObjFile(objFile)<0) return false;
    loadTexture(textureFile);
    return true;
}

/**
 * @brief Loads texture image into OpenGL
 * @param textureFile Image file path
 * @return 0 on success
 * 
 * - Converts image to RGB888 format
 * - Mirrors vertically for OpenGL texture coordinates
 * - Generates OpenGL texture with linear filtering
 */
int ObjLoader::loadTexture(QString textureFile)
{
    QImage image(textureFile);
    image = image.convertToFormat(QImage::Format_RGB888);
    image = image.mirrored(); // Flip vertically for OpenGL texture origin
    
    glGenTextures(1, &targetTexture);
    glBindTexture(GL_TEXTURE_2D, targetTexture);
    
    // Set texture parameters
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);   
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

    // Upload texture data to GPU
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
                 image.width(), image.height(), 0, GL_RGB, GL_UNSIGNED_BYTE,
                 image.bits());
    return 0;
}

/**
 * @brief Parses Wavefront OBJ file
 * @param pathToFile OBJ file path
 * @return 0 on success, -2 on file error
 * 
 * Processes:
 * - v: Vertex positions
 * - vt: Texture coordinates
 * - vn: Normal vectors
 * - f: Face definitions with indices
 */
int ObjLoader::loadObjFile(QString pathToFile)
{
    QFile f(pathToFile);
    if(!f.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug()<<"Error: Obj file cannot be opened!";
        return -2;
    }

    QTextStream ts(&f);
    mVertices.clear();
    mNormals.clear();
    mTextures.clear();
    mFaces.clear();

    while(!ts.atEnd())
    {
        QString line = ts.readLine();
        QStringList strList = line.split(QRegExp("(\\s+)|/"));
        strList.removeAll("");

        if(strList.isEmpty()) continue;

        // Process vertex position (v)
        if(strList[0] == "v")
        {
            Vertex tmp(strList[1].toFloat(),strList[2].toFloat(),strList[3].toFloat());
            mVertices.push_back(tmp);
        }

        // Process texture coordinate (vt)
        if(strList[0] == "vt")
        {
            Texture tmp(strList[1].toFloat(),strList[2].toFloat(), 0.0f);
            if (strList.size()>3) tmp.z = strList[3].toFloat();
            mTextures.push_back(tmp);
        }

        // Process normal vector (vn)
        if(strList[0] == "vn")
        {
            Normal tmp(strList[1].toFloat(),strList[2].toFloat(),strList[3].toFloat());
            mNormals.push_back(tmp);
        }

        // Process face definition (f)
        if(strList[0] == "f")
        {
            int found = line.indexOf("//", 0, Qt::CaseInsensitive); 
            int step = (found != -1) ? 2 : 3; // Handle v//vn format

            Face tmp;
            tmp.numIndicies = strList.size()/step;

            // Parse face indices
            for(int i=0; i<tmp.numIndicies; i++)
            {
                tmp.vertexIndicies.push_back(strList[step*i+1].toInt());
                tmp.normalIndicies.push_back(strList[step*i+step].toInt());
                if(step == 3)
                    tmp.textureIndicies.push_back(strList[step*i+2].toInt());
                else
                    tmp.textureIndicies.push_back(0); // No texture index
            }
            mFaces.push_back(tmp);
        }
    }
    return 0;
}

/**
 * @brief Scales model and calculates bounding dimensions
 * @param scale Uniform scaling factor
 * 
 * Processes:
 * 1. Applies scaling to vertex positions
 * 2. Calculates model bounding box
 * 3. Stores final dimensions (size_x/y/z)
 */
void ObjLoader::resize(float scale)
{
    max_x=max_y=max_z=-1e5;
    min_x=min_y=min_z=1e5;

    // Process all faces
    for (int i = 0; i < mFaces.size(); ++i)
    {
        auto p = &mFaces.at(i);
        p->vValues.clear();

        // Scale vertices and store processed values
        for (int j = 0; j < p->vertexIndicies.size(); ++j)
        {
            if (!mVertices.empty())
            {
                auto v_ = mVertices.at(p->vertexIndicies.at(j)-1);
                p->vValues.push_back(v_.x*scale);
                p->vValues.push_back(v_.y*scale);
                p->vValues.push_back(v_.z*scale);
            }

            // Store texture coordinates if available
            if (!mTextures.empty() && j < p->textureIndicies.size())
            {
                auto vt_ = mTextures.at(p->textureIndicies.at(j)-1);
                p->vtValues.push_back(vt_.x);
                p->vtValues.push_back(vt_.y);
                p->vtValues.push_back(vt_.z);
            }

            // Store normals if available
            if (!mNormals.empty() && j < p->normalIndicies.size())
            {
                auto vn_ = mNormals.at(p->normalIndicies.at(j)-1);
                p->vnValues.push_back(vn_.i);
                p->vnValues.push_back(vn_.j);
                p->vnValues.push_back(vn_.k);
            }
        }

        // Update bounding box
        for(int j = 0; j < p->vValues.size(); j+=3)
        {
            max_x = MAX(max_x, p->vValues[j]);
            max_y = MAX(max_y, p->vValues[j+1]);
            max_z = MAX(max_z, p->vValues[j+2]);

            min_x = MIN(min_x, p->vValues[j]);
            min_y = MIN(min_y, p->vValues[j+1]);
            min_z = MIN(min_z, p->vValues[j+2]);
        }
    }

    // Calculate final dimensions
    size_x = max_x - min_x;
    size_y = max_y - min_y;
    size_z = max_z - min_z;
}

/**
 * @brief Renders model with uniform color
 * @param sx,sy,sz Scale factors per axis
 * @param r,g,b RGB color components [0-1]
 * 
 * - Centers model around origin
 * - Applies axis-specific scaling
 * - Uses face normal data for lighting
 */
void ObjLoader::Render(float sx, float sy, float sz, float r, float g, float b)
{
    // Calculate scaling factors
    float kx = sx/size_x;
    float ky = sy/size_y;
    float kz = sz/size_z;

    // Calculate center offset
    float dx = (max_x + min_x)*0.5f;
    float dy = (max_y + min_y)*0.5f;
    float dz = min_z;

    // Set material color
    GLfloat l_body[4] = {r, g, b, 1.f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, l_body);

    // Render each face
    for(unsigned long i = 0; i < mFaces.size(); ++i)
    {
        auto p = &mFaces.at(i);
        
        // Select primitive type
        if(p->numIndicies == 3) glBegin(GL_TRIANGLES);
        else if(p->numIndicies == 4) glBegin(GL_QUADS);
        else glBegin(GL_POLYGON);

        // Process vertices
        for(int j = 0; j < p->vValues.size(); j+=3)
        {
            // Apply normal (inverted X for coordinate system conversion)
            glNormal3d(-p->vnValues[j], p->vnValues[j+1], p->vnValues[j+2]);

            // Apply scaled and centered vertex
            glVertex3f(-(p->vValues[j]-dx)*kx, 
                      (p->vValues[j+1]-dy)*ky,
                      (p->vValues[j+2]-dz)*kz);
        }
        glEnd();
    }
}

/**
 * @brief Renders model with texture mapping
 * @param sx,sy,sz Scale factors per axis
 * 
 * - Enables texture mapping
 * - Centers and scales model
 * - Uses UV coordinates from OBJ file
 */
void ObjLoader::Render_Texture(float sx, float sy, float sz)
{
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, targetTexture);

    GLfloat l_body[4] = {1, 1, 1, 1.f};
    glMaterialfv(GL_FRONT, GL_DIFFUSE, l_body);

    // Calculate center offset
    float dx = (max_x + min_x)*0.5f;
    float dy = (max_y + min_y)*0.5f;
    float dz = min_z;

    // Calculate scaling factors
    float kx = sx/size_x;
    float ky = sy/size_y;
    float kz = sz/size_z;

    // Render each face
    for(unsigned long i = 0; i < mFaces.size(); ++i)
    {
        auto p = &mFaces.at(i);

        // Select primitive type
        if(p->numIndicies == 3) glBegin(GL_TRIANGLES);
        else if(p->numIndicies == 4) glBegin(GL_QUADS);
        else glBegin(GL_POLYGON);

        // Process vertices
        for(int j = 0; j < p->vValues.size(); j+=3)
        {
            // Apply normal (inverted X for coordinate system conversion)
            glNormal3d(-p->vnValues[j], p->vnValues[j+1], p->vnValues[j+2]);

            // Apply texture coordinate
            if(j/3 < p->vtValues.size()/3)
                glTexCoord2f(p->vtValues[j], p->vtValues[j+1]);

            // Apply scaled and centered vertex
            glVertex3f(-(p->vValues[j]-dx)*kx,
                      (p->vValues[j+1]-dy)*ky,
                      (p->vValues[j+2]-dz)*kz);
        }
        glEnd();
    }

    glDisable(GL_TEXTURE_2D);
}