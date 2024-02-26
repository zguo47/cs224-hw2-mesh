#pragma once

#include <vector>

#include "Eigen/StdVector"
#include "Eigen/Dense"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i);

struct Halfedge;
struct Vertex;
struct Face;

struct Halfedge {
    Halfedge *twin = nullptr;
    Halfedge *next = nullptr;
    Vertex *origin = nullptr;
    Face *face = nullptr;
};

struct Vertex {
    int index;
    Halfedge *halfedge = nullptr;
    Eigen::Vector3f position;
};

struct Face {
    Halfedge *halfedge = nullptr;
    Eigen::Vector3i face;
};


class Mesh
{
public:
    std::vector<Vertex*> m_vertices;
    std::vector<Halfedge*> m_halfedges;
    std::vector<Face*> m_faces;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
                         const std::vector<Eigen::Vector3i> &faces);

    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
    void storeHalfedge();
    void validate(const Mesh &meshDataStructure);
    void edgeflip(int flipIndex);
    void edgeSplit(int splitIndex);
    void collapse(int collapseIndex);

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
};
