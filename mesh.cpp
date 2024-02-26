#include "mesh.h"

#include <cassert>
#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>

#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"

#include <sstream>

#define ASSERT_WITH_MESSAGE(condition, message) \
do { \
        if (!(condition)) { \
            std::stringstream ss; \
            ss << "Assertion failed: " << (message) << "\n"; \
            assert((condition) && ss.str().c_str()); \
    } \
} while (false)

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const vector<Vector3f> &vertices,
                           const vector<Vector3i> &faces)
{
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _faces    = faces;
}

void Mesh::loadFromFile(const string &filePath)
{
    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> materials;

    QFileInfo info(QString(filePath.c_str()));
    string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if (!err.empty()) {
        cerr << err << endl;
    }

    if (!ret) {
        cerr << "Failed to load/parse .obj file" << endl;
        return;
    }

    for (size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for (size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                face[v] = idx.vertex_index;
            }
            _faces.push_back(face);

            index_offset += fv;
        }
    }
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        _vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    }
    cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << endl;
}

void Mesh::saveToFile(const string &filePath)
{
    ofstream outfile;
    outfile.open(filePath);

    // Write vertices
    for (size_t i = 0; i < m_vertices.size(); i++) {
        const Vector3f &v = m_vertices[i]->position;
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for (size_t i = 0; i < m_faces.size(); i++) {
        const Vector3i &f = m_faces[i]->face;
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << endl;
    }


    outfile.close();
}

void Mesh::storeHalfedge() {
    // Store pointers to Vertex objects
    for (size_t i = 0; i < _vertices.size(); i++) {
        Vector3f &v = _vertices[i];
        Vertex *origin = new Vertex();
        origin->position = v;
        origin->halfedge = nullptr;
        m_vertices.push_back(origin);
        const int index = i;
        origin->index = index;
    }

    // Create faces and half-edges
    for (size_t i = 0; i < _faces.size(); i++) {
        Vector3i &f = _faces[i];
        Face *face = new Face();
        Halfedge *prevHe = nullptr;
        for (int j = 0; j < 3; ++j) {
            int curr_faceIdx = f[j];
            int next_faceIdx = f[(j + 1) % 3];
            Halfedge *he = new Halfedge();
            he->origin = m_vertices[curr_faceIdx];
            he->origin->halfedge = he;
            he->face = face;
            if (prevHe) {
                prevHe->next = he;
            } else {
                face->halfedge = he;
            }
            prevHe = he;
            m_halfedges.push_back(he);
        }
        prevHe->next = face->halfedge;
        face->face = f;
        m_faces.push_back(face);
    }

    // Link twin half-edges
    for (auto &he : m_halfedges) {
        for (auto &otherHe : m_halfedges) {
            if (he != otherHe && he->origin == otherHe->next->origin && otherHe->origin == he->next->origin) {
                he->twin = otherHe;
                break;
            }
        }
    }
}

void Mesh::validate(const Mesh& meshDataStructure) {
    ASSERT_WITH_MESSAGE(m_halfedges.size() == 3 * m_faces.size(), "There should be 3 x faces halfedges");

    for (auto *he : meshDataStructure.m_halfedges) {
        ASSERT_WITH_MESSAGE(he->twin != nullptr, "Halfedge has no twin");
        ASSERT_WITH_MESSAGE(he->next != nullptr, "Halfedge has no next");
        ASSERT_WITH_MESSAGE(he->origin != nullptr, "Halfedge has no origin");
        ASSERT_WITH_MESSAGE(he->face != nullptr, "Halfedge has no face");

        // Check that halfedges that are twins to each other are indeed twins
        ASSERT_WITH_MESSAGE(he->twin->twin == he, "Halfedge's twin's twin does not point back to the halfedge");
        ASSERT_WITH_MESSAGE(he->twin->origin == he->next->origin, "Halfedge's twin's origin does not match halfedge's next origin");

        // Check that the next pointer forms a closed loop
        Halfedge *current = he;
        do {
            ASSERT_WITH_MESSAGE(current != nullptr, "Halfedge loop is broken");
            current = current->next;
        } while (current != he);
    }

    int index = 0;
    for (auto *v : meshDataStructure.m_vertices) {
        ASSERT_WITH_MESSAGE(v->halfedge != nullptr, "Vertex has no halfedge");
        ASSERT_WITH_MESSAGE(v->halfedge->origin == v, "Vertex's halfedge does not point back to the vertex");
        ASSERT_WITH_MESSAGE(v->index == index, "Vertex's index is incorrect, should be equal to its index in m_vertices");
        index += 1;

        // Traverse through all edges from this vertex
        Halfedge *start = v->halfedge;
        Halfedge *current = start;
        do {
            ASSERT_WITH_MESSAGE(current != nullptr, "Vertex edge loop is broken");
            current = current->twin->next;
        } while (current != start);
    }

    for (auto *f : meshDataStructure.m_faces) {
        ASSERT_WITH_MESSAGE(f->halfedge != nullptr, "Face has no halfedge");
        ASSERT_WITH_MESSAGE(f->halfedge->face == f, "Face's halfedge does not point back to the face");
        ASSERT_WITH_MESSAGE(f->face[0] != f->face[1], "Face has zero area");
        ASSERT_WITH_MESSAGE(f->face[1] != f->face[2], "Face has zero area");
        ASSERT_WITH_MESSAGE(f->face[0] != f->face[2], "Face has zero area");
    }
}

void Mesh::edgeflip(int flipIndex){
    Halfedge *toFlip = m_halfedges[flipIndex];
    Halfedge *toFlipTwin = toFlip->twin;

    // Store the original half-edges for updating
    Halfedge *origNext = toFlip->next;
    Halfedge *origPrev = toFlip->next->next;
    Halfedge *twinNext = toFlipTwin->next;
    Halfedge *twinPrev = toFlipTwin->next->next;

    // Update the origin of the flipped half-edges
    toFlip->origin = origPrev->origin;
    toFlipTwin->origin = twinPrev->origin;

    // Update the next of next to rearrange the half-edges
    twinPrev->next = toFlip->next;
    origPrev->next = toFlipTwin->next;

    // Update the next pointers to rearrange the half-edges
    toFlip->next = twinPrev;
    toFlipTwin->next = origPrev;
    origNext->next = toFlip;
    twinNext->next = toFlipTwin;

    // Update the half-edge pointers in the vertices
    toFlip->origin->halfedge = toFlip;
    toFlipTwin->origin->halfedge = toFlipTwin;

    // Update the half-edge pointers in the faces
    Face *origFace = toFlip->face;
    Face *twinFace = toFlipTwin->face;
    origFace->halfedge = toFlip;
    twinFace->halfedge = toFlipTwin;

    origFace->face = Eigen::Vector3i(toFlip->origin->index, toFlip->next->origin->index, toFlip->next->next->origin->index);
    twinFace->face = Eigen::Vector3i(toFlipTwin->origin->index, toFlipTwin->next->origin->index, toFlipTwin->next->next->origin->index);

}

void Mesh::edgeSplit(int splitIndex){
    Halfedge *toSplit = m_halfedges[splitIndex];
    Halfedge *toSplitTwin = toSplit->twin;

    // Store the original half-edges for updating
    Halfedge *origNext = toSplit->next;
    Halfedge *origPrev = toSplit->next->next;
    Halfedge *twinNext = toSplitTwin->next;
    Halfedge *twinPrev = toSplitTwin->next->next;

    Vector3f distance = toSplitTwin->origin->position - toSplit->origin->position;
    Vector3f midpoint(toSplit->origin->position[0] + 0.5 * distance[0], toSplit->origin->position[1] + 0.5 * distance[1], toSplit->origin->position[2] + 0.5 * distance[2]);
    Vertex *midVertex = new Vertex();
    midVertex->position = midpoint;
    midVertex->index = m_vertices.size();

    //new halfedges on left and right
    Halfedge *leftNewHe = new Halfedge();
    Halfedge *rightNewHe = new Halfedge();
    Halfedge *leftNewHeTwin = new Halfedge();
    Halfedge *rightNewHeTwin = new Halfedge();
    //assign twin and origin
    leftNewHe->twin = leftNewHeTwin;
    leftNewHeTwin->twin = leftNewHe;
    rightNewHe->twin = rightNewHeTwin;
    rightNewHeTwin->twin = rightNewHe;

    leftNewHe->origin = origPrev->origin;
    leftNewHeTwin->origin = midVertex;
    rightNewHe->origin = twinPrev->origin;
    rightNewHeTwin->origin = midVertex;

    //new halfedges on up and down
    Halfedge *upNewHe = new Halfedge();
    Halfedge *downNewHe = new Halfedge();
    Halfedge *upNewHeTwin = toSplitTwin;
    Halfedge *downNewHeTwin = toSplit;
    //assign twin and origin
    upNewHe->twin = upNewHeTwin;
    upNewHeTwin->twin = upNewHe;
    downNewHe->twin = downNewHeTwin;
    downNewHeTwin->twin = downNewHe;

    upNewHe->origin = midVertex;
    downNewHe->origin = midVertex;

    //assign next to new halfedges and origNext, origPrev, twinNext, twinPrev
    leftNewHe->next = upNewHe;
    upNewHe->next = origNext;
    origNext->next = leftNewHe;

    leftNewHeTwin->next = origPrev;
    origPrev->next = downNewHeTwin;
    downNewHeTwin->next = leftNewHeTwin;

    rightNewHe->next = downNewHe;
    downNewHe->next = twinNext;
    twinNext->next = rightNewHe;

    rightNewHeTwin->next = twinPrev;
    twinPrev->next = upNewHeTwin;
    upNewHeTwin->next = rightNewHeTwin;

    //assigh halfedge to new vertex and re-assign old
    midVertex->halfedge = upNewHe;

    leftNewHe->origin->halfedge = leftNewHe;
    leftNewHeTwin->origin->halfedge = leftNewHeTwin;
    rightNewHe->origin->halfedge = rightNewHe;
    rightNewHeTwin->origin->halfedge = rightNewHeTwin;
    upNewHe->origin->halfedge = upNewHe;
    upNewHeTwin->origin->halfedge = upNewHeTwin;
    downNewHe->origin->halfedge = downNewHe;
    downNewHeTwin->origin->halfedge = downNewHeTwin;


    //create new faces
    Face *leftNewHeFace = new Face();
    Face *rightNewHeFace = new Face();
    Face *origFace = toSplit->face;
    Face *twinFace = toSplitTwin->face;

    leftNewHeFace->halfedge = leftNewHe;
    origFace->halfedge = leftNewHeTwin;
    twinFace->halfedge = rightNewHeTwin;
    rightNewHeFace->halfedge = rightNewHe;

    leftNewHeFace->face = Eigen::Vector3i(leftNewHe->origin->index, upNewHe->origin->index, origNext->origin->index);
    origFace->face = Eigen::Vector3i(leftNewHeTwin->origin->index, origPrev->origin->index, downNewHeTwin->origin->index);
    rightNewHeFace->face = Eigen::Vector3i(rightNewHe->origin->index, downNewHe->origin->index, twinNext->origin->index);
    twinFace->face = Eigen::Vector3i(rightNewHeTwin->origin->index, twinPrev->origin->index, upNewHeTwin->origin->index);

    //assign faces to relevant halfedges
    leftNewHe->face = leftNewHeFace;
    leftNewHeTwin->face = origFace;
    upNewHe->face = leftNewHeFace;
    origNext->face = leftNewHeFace;
    origPrev->face = origFace;
    downNewHeTwin->face = origFace;

    rightNewHe->face = rightNewHeFace;
    rightNewHeTwin->face = twinFace;
    downNewHe->face = rightNewHeFace;
    twinNext->face = rightNewHeFace;
    twinPrev->face = twinFace;
    upNewHeTwin->face = twinFace;

    //push back new vertex
    m_vertices.push_back(midVertex);

    //push back new halfedges
    m_halfedges.push_back(downNewHe);
    m_halfedges.push_back(upNewHe);
    m_halfedges.push_back(leftNewHe);
    m_halfedges.push_back(rightNewHe);
    m_halfedges.push_back(leftNewHeTwin);
    m_halfedges.push_back(rightNewHeTwin);

    //push back new faces
    m_faces.push_back(leftNewHeFace);
    m_faces.push_back(rightNewHeFace);

}

void Mesh::collapse(int collapseIndex){
    Halfedge *toCollapse = m_halfedges[collapseIndex];
    Halfedge *toCollapseTwin = toCollapse->twin;

    // Store the original half-edges for updating
    Halfedge *origNext = toCollapse->next;
    Halfedge *origPrev = toCollapse->next->next;
    Halfedge *twinNext = toCollapseTwin->next;
    Halfedge *twinPrev = toCollapseTwin->next->next;

    // Remove vertices c and d
    m_vertices.erase(std::remove(m_vertices.begin(), m_vertices.end(), toCollapse->origin), m_vertices.end());
    m_vertices.erase(std::remove(m_vertices.begin(), m_vertices.end(), toCollapseTwin->origin), m_vertices.end());

    //update the vertices to normal order
    for (size_t i = 0; i < m_vertices.size(); ++i) {
        m_vertices[i]->index = i;
    }

    Vector3f distance = toCollapseTwin->origin->position - toCollapse->origin->position;
    Vector3f midpoint(toCollapse->origin->position[0] + 0.5 * distance[0], toCollapse->origin->position[1] + 0.5 * distance[1], toCollapse->origin->position[2] + 0.5 * distance[2]);
    Vertex *midVertex = new Vertex();
    midVertex->position = midpoint;
    midVertex->index = m_vertices.size();

    // Remove faces adjacent to the collapsing edge
    Face* faceC = toCollapse->face;
    Face* faceD = toCollapseTwin->face;
    m_faces.erase(std::remove(m_faces.begin(), m_faces.end(), faceC), m_faces.end());
    m_faces.erase(std::remove(m_faces.begin(), m_faces.end(), faceD), m_faces.end());

    //reassign origNext and twinNext, discarding origPrev and twinPrev
    origNext->next = origNext->next->twin->next;
    twinNext->next = twinNext->next->twin->next;
    origPrev->next = origNext->twin->next;
    twinPrev->next = twinNext->twin->next;

    //reassign origNext.next and twinNext.next
    origNext->next->next->next = origNext;
    twinNext->next->next->next = twinNext;
    origPrev->next->next->next = origPrev;
    twinPrev->next->next->next = twinPrev;

    //assign prev
    origNext->twin->next->next->next = origPrev;
    twinNext->twin->next->next->next = twinPrev;
    origPrev->twin->next->next->next = origNext;
    twinPrev->twin->next->next->next = twinNext;

    // Remove the collapsing edge and its twin from m_halfedges
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), toCollapse), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), toCollapseTwin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), origNext->twin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), origPrev->twin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), twinNext->twin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), twinPrev->twin), m_halfedges.end());

    //assign twins
    origNext->twin = origPrev;
    origPrev->twin = origNext;
    twinNext->twin = twinPrev;
    twinPrev->twin = twinNext;

    // Replace references from c and d to m in all half-edges
    for (auto *he : m_halfedges) {
        if (he->origin == toCollapse->origin || he->origin == toCollapseTwin->origin) {
            he->origin = midVertex;
        }
    }

    for (auto *he : m_halfedges){
        if (he->origin == midVertex){
            he->face->face = Vector3i(midVertex->index, he->next->origin->index, he->next->next->origin->index);
            he->next->face->face = Vector3i(midVertex->index, he->next->origin->index, he->next->next->origin->index);
            he->next->next->face->face = Vector3i(midVertex->index, he->next->origin->index, he->next->next->origin->index);
        }else{
            he->face->face = Vector3i(he->origin->index, he->next->origin->index, he->next->next->origin->index);
        }
    }

    midVertex->halfedge = origNext;
    m_vertices.push_back(midVertex);

    delete toCollapse;
    delete toCollapseTwin;
    delete faceC;
    delete faceD;

}


