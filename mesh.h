#pragma once

#include <vector>

#include "Eigen/StdVector"
#include "Eigen/Dense"
#include <map>
#include <set>

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
};

struct Edge {
    Halfedge *firsthalfedge = nullptr;
    Halfedge *secondhalfedge = nullptr;
};

struct HalfedgeErrorComparator {
    const std::map<Halfedge*, float>& errorMap;
    HalfedgeErrorComparator(const std::map<Halfedge*, float>& errorMap) : errorMap(errorMap) {}
    bool operator()(Halfedge* he1, Halfedge* he2) const {
        return errorMap.at(he1) < errorMap.at(he2);
    }
};





class Mesh
{
public:
    std::vector<Vertex*> m_vertices;
    std::vector<Halfedge*> m_halfedges;
    std::vector<Face*> m_faces;
    std::vector<Edge*> m_edges;
    std::map<Vertex*, int> vertexHalfedgeCount;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
                         const std::vector<Eigen::Vector3i> &faces);

    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
    void storeHalfedge();
    void validate(const Mesh &meshDataStructure);
    void edgeflip(Halfedge &toFlip);
    void edgeSplit(Halfedge &toSplit);
    void collapse(Halfedge &toCollapse, Eigen::Vector3f& optimalpos, bool method);

    void subdivide(int numIterations);
    void simplify(int numTrianglesRemove);
    void remesh(float weight, int numIter);
    void noise();
    void denoise(int numIter, float sigma_s, float sigma_c);

    std::pair<int, std::vector<Vertex*>> countCommonElements(const std::vector<Vertex*>& v1, const std::vector<Vertex*>& v2);
    Eigen::Vector3f computeOptimalPosition(const Eigen::Matrix4f& Q, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2);
    float antiFlippedTriangle(Face* oldFace, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2, const Eigen::Vector3f& v3);
    Eigen::Vector3f computeFaceNormal(const Face& face);

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
};
