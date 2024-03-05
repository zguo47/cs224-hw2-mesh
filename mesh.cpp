#include "mesh.h"

#include <cassert>
#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>
#include <random>

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

    for (size_t i = 0; i < m_vertices.size(); ++i) {
        m_vertices[i]->index = i;
    }

    // Write faces
    for (size_t i = 0; i < m_faces.size(); i++) {
        const Vector3i &f = Vector3i(m_faces[i]->halfedge->origin->index, m_faces[i]->halfedge->next->origin->index, m_faces[i]->halfedge->next->next->origin->index);
        outfile << "f " << (f[0] + 1) << " " << (f[1] + 1) << " " << (f[2] + 1) << endl;
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

    for (auto *vertex : m_vertices){
        int count = 0;
        for (auto* he : m_halfedges) {
            if (he->origin == vertex) {
                ++count;
            }
        }
        vertexHalfedgeCount[vertex] = count;
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
        index += 1;

        // Traverse through all edges from this vertex
        int degree = 0;
        Halfedge *start = v->halfedge;
        Halfedge *current = start;
        do {
            ASSERT_WITH_MESSAGE(current != nullptr, "Vertex edge loop is broken");
            current = current->twin->next;
            ++degree;
        } while (current != start);
        ASSERT_WITH_MESSAGE(vertexHalfedgeCount[v] == degree, "vertex degree does not match vertexHalfedgeCount");
    }

    for (auto *f : meshDataStructure.m_faces) {
        ASSERT_WITH_MESSAGE(f->halfedge != nullptr, "Face has no halfedge");
        ASSERT_WITH_MESSAGE(f->halfedge->face == f, "Face's halfedge does not point back to the face");
        ASSERT_WITH_MESSAGE(f->halfedge->face != f->halfedge->twin->face, "twin edges cannot share the same face");
        ASSERT_WITH_MESSAGE(f->halfedge->origin != f->halfedge->next->origin, "Face has zero area");
        ASSERT_WITH_MESSAGE(f->halfedge->next->origin != f->halfedge->next->next->origin, "Face has zero area");
        ASSERT_WITH_MESSAGE(f->halfedge->next->next->origin != f->halfedge->origin, "Face has zero area");
    }

}

void Mesh::edgeflip(Halfedge &toflip){
    Halfedge *toFlip = &toflip;
    Halfedge *toFlipTwin = toFlip->twin;

    int count = 0;
    int counttwin = 0;
    if (vertexHalfedgeCount[toFlip->origin] == 3 || vertexHalfedgeCount[toFlipTwin->origin] == 3){
        return;
    }
    vertexHalfedgeCount[toFlip->origin] -= 1;
    vertexHalfedgeCount[toFlipTwin->origin] -= 1;

    // Store the original half-edges for updating
    Halfedge *origNext = toFlip->next;
    Halfedge *origPrev = toFlip->next->next;
    Halfedge *twinNext = toFlipTwin->next;
    Halfedge *twinPrev = toFlipTwin->next->next;

    // Update the origin of the flipped half-edges
    Vertex *oppositeA = origPrev->origin;
    Vertex *oppositeC = twinPrev->origin;
    if (toFlip->origin->halfedge == toFlip){
        toFlip->origin->halfedge = toFlip->twin->next;
    }
    if (toFlipTwin->origin->halfedge == toFlipTwin){
        toFlipTwin->origin->halfedge = toFlipTwin->twin->next;
    }
    toFlip->origin = oppositeA;
    toFlipTwin->origin = oppositeC;

    // update vertexHalfedgeCount
    vertexHalfedgeCount[toFlip->origin] += 1;
    vertexHalfedgeCount[toFlipTwin->origin] += 1;

    // Update next pointers to rearrange the half-edges
    toFlip->next = twinPrev;
    twinPrev->next = origNext;
    origNext->next = toFlip;

    toFlipTwin->next = origPrev;
    origPrev->next = twinNext;
    twinNext->next = toFlipTwin;

    twinPrev->face = toFlip->face;
    origPrev->face = toFlipTwin->face;

    // Update vertex halfedge pointers
    toFlip->origin->halfedge = toFlip;
    toFlipTwin->origin->halfedge = toFlipTwin;

    // Update the half-edge pointers in the faces
    Face *origFace = toFlip->face;
    Face *twinFace = toFlipTwin->face;
    origFace->halfedge = toFlip;
    twinFace->halfedge = toFlipTwin;

}

void Mesh::edgeSplit(Halfedge &tosplit){
    Halfedge *toSplit = &tosplit;
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

    // update edge count
    vertexHalfedgeCount[leftNewHe->origin] += 1;
    vertexHalfedgeCount[rightNewHe->origin] += 1;

    //push back new vertex
    m_vertices.push_back(midVertex);
    vertexHalfedgeCount[midVertex] = 4;

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

void Mesh::collapse(Halfedge &tocollapse, Vector3f& optimalpos, bool method){
    Halfedge *toCollapse = &tocollapse;
    Halfedge *toCollapseTwin = toCollapse->twin;

    if (method){
        // Traverse through all edges from this vertex
        std::vector<Vertex*> toCollapseIndices;
        std::vector<Vertex*> toCollapseTwinIndices;
        Halfedge *start = toCollapse;
        Halfedge *current = start;
        do {
            toCollapseIndices.push_back(current->next->origin);
            current = current->twin->next;
        } while (current != start);

        Halfedge *startTwin = toCollapseTwin;
        Halfedge *currentTwin = startTwin;
        do {
            toCollapseTwinIndices.push_back(currentTwin->next->origin);
            currentTwin = currentTwin->twin->next;
        } while (currentTwin != startTwin);

        std::pair<int, std::vector<Vertex*>> result = countCommonElements(toCollapseIndices, toCollapseTwinIndices);
        int commonN = result.first;
        if (commonN != 2){
            return;
        }
        for (auto *i : result.second){
            if (vertexHalfedgeCount[i] == 3){
                return;
            }
        }
    }

    // Store the original half-edges for updating
    Halfedge *origNext = toCollapse->next;
    Halfedge *origPrev = toCollapse->next->next;
    Halfedge *twinNext = toCollapseTwin->next;
    Halfedge *twinPrev = toCollapseTwin->next->next;

    // Remove vertices c and d
    m_vertices.erase(std::remove(m_vertices.begin(), m_vertices.end(), toCollapse->origin), m_vertices.end());
    m_vertices.erase(std::remove(m_vertices.begin(), m_vertices.end(), toCollapseTwin->origin), m_vertices.end());

    // Vector3f distance = toCollapseTwin->origin->position - toCollapse->origin->position;
    // Vector3f midpoint(toCollapse->origin->position[0] + 0.5 * distance[0], toCollapse->origin->position[1] + 0.5 * distance[1], toCollapse->origin->position[2] + 0.5 * distance[2]);
    Vertex *midVertex = new Vertex();
    midVertex->position = optimalpos;
    midVertex->index = m_vertices.size();

    int prevFacesSize = m_faces.size();

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

    // //assign prev
    origNext->twin->next->next->next = origPrev;
    twinNext->twin->next->next->next = twinPrev;
    origPrev->twin->next->next->next = origNext;
    twinPrev->twin->next->next->next = twinNext;

    if (origPrev->origin->halfedge == origNext->twin){
        origPrev->origin->halfedge = origPrev;
    }
    if (twinPrev->origin->halfedge == twinNext->twin){
        twinPrev->origin->halfedge = twinPrev;
    }

    // Remove the collapsing edge and its twin from m_halfedges
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), toCollapse), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), toCollapseTwin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), origNext->twin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), origPrev->twin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), twinNext->twin), m_halfedges.end());
    m_halfedges.erase(std::remove(m_halfedges.begin(), m_halfedges.end(), twinPrev->twin), m_halfedges.end());

    vertexHalfedgeCount[origPrev->origin] -= 1;
    vertexHalfedgeCount[twinPrev->origin] -= 1;

    //assign twins
    origNext->twin = origPrev;
    origPrev->twin = origNext;
    twinNext->twin = twinPrev;
    twinPrev->twin = twinNext;

    midVertex->halfedge = origNext;
    m_vertices.push_back(midVertex);

    Halfedge *s1 = origNext;
    Halfedge *c1 = s1;
    do {
        c1->origin = midVertex;
        c1 = c1->twin->next;
    } while (c1 != s1);

    Halfedge *s2 = twinNext;
    Halfedge *c2 = s2;
    do {
        c2->origin = midVertex;
        c2 = c2->twin->next;
    } while (c2 != s2);

    origNext->face = origNext->next->face;
    origNext->face->halfedge = origNext;

    origPrev->face = origPrev->next->face;
    origPrev->face->halfedge = origPrev;

    twinNext->face = twinNext->next->face;
    twinNext->face->halfedge = twinNext;

    twinPrev->face = twinPrev->next->face;
    twinPrev->face->halfedge = twinPrev;

    if (origPrev->next == twinNext){
        origPrev->face = twinNext->face;
    }

    vertexHalfedgeCount[midVertex] = vertexHalfedgeCount[toCollapse->origin] + vertexHalfedgeCount[toCollapseTwin->origin] - 4;
    vertexHalfedgeCount.erase(toCollapse->origin);
    vertexHalfedgeCount.erase(toCollapseTwin->origin);

}

std::pair<int, std::vector<Vertex*>> Mesh::countCommonElements(const std::vector<Vertex*>& v1, const std::vector<Vertex*>& v2) {
    auto compare = [](const Vertex* a, const Vertex* b) {
        return a->index < b->index;
    };

    // Sort the vectors
    std::vector<Vertex*> sorted_v1 = v1;
    std::vector<Vertex*> sorted_v2 = v2;
    std::sort(sorted_v1.begin(), sorted_v1.end(), compare);
    std::sort(sorted_v2.begin(), sorted_v2.end(), compare);

    // Find the intersection
    std::vector<Vertex*> intersection;
    std::set_intersection(sorted_v1.begin(), sorted_v1.end(),
                          sorted_v2.begin(), sorted_v2.end(),
                          std::back_inserter(intersection), compare);

    // Return the number of common elements and the intersection vector
    return std::make_pair(intersection.size(), intersection);
}

void Mesh::subdivide(int numIterations){
    while(numIterations != 0){
        int numOldVertices = m_vertices.size();
        std::vector<Halfedge*> edgesToSplit;
        std::map<Halfedge*, Vector3f> newVertexPositions;

        // Collect half-edges to split
        for (auto *he : m_halfedges) {
            if (he->twin && std::find(edgesToSplit.begin(), edgesToSplit.end(), he) == edgesToSplit.end()
                && std::find(edgesToSplit.begin(), edgesToSplit.end(), he->twin) == edgesToSplit.end()) {
                edgesToSplit.push_back(he);
            }
            newVertexPositions[he] = 3.0f / 8 * (he->origin->position + he->twin->origin->position) + 1.0f / 8 * (he->next->next->origin->position + he->twin->next->next->origin->position);
        }
        std::vector<Halfedge*> oldEdges = m_halfedges;

        // Calculate new positions for original vertices
        std::vector<Vector3f> newPositions(numOldVertices);
        for (int i = 0; i < numOldVertices; ++i) {
            Vertex *v = m_vertices[i];
            std::vector<Vertex*> neighbors;
            Halfedge *start = v->halfedge;
            Halfedge *he = start;
            do {
                neighbors.push_back(he->twin->origin);
                he = he->twin->next;
            } while (he != start);

            int n = neighbors.size();
            float beta = (n == 3) ? 3.0f / 16 : (5.0f / 8.0f - pow(3.0f / 8.0f + (1.0f / 4.0f) * cos(2 * M_PI / n), 2)) / n;
            Vector3f newPos = (1 - n * beta) * v->position;
            for (Vertex *neighbor : neighbors) {
                newPos += beta * neighbor->position;
            }
            newPositions[i] = newPos;
        }

        // Update positions of original vertices
        for (int i = 0; i < numOldVertices; ++i) {
            m_vertices[i]->position = newPositions[i];
        }

        // Split edges
        for (auto *he : edgesToSplit) {
            edgeSplit(*he);
            m_vertices[m_vertices.size() - 1]->position = newVertexPositions[he];
        }

        // Collect half-edges to flip
        std::vector<Halfedge*> edgesToFlip;
        for (auto *he : m_halfedges) {
            if (he->twin && std::find(edgesToFlip.begin(), edgesToFlip.end(), he) == edgesToFlip.end()
                && std::find(edgesToFlip.begin(), edgesToFlip.end(), he->twin) == edgesToFlip.end()) {
                edgesToFlip.push_back(he);
            }
        }

        // Flip edges
        int n = 0;
        for (auto *he : edgesToFlip){
            if ((he->origin->index >= numOldVertices && he->twin->origin->index < numOldVertices) || (he->origin->index < numOldVertices && he->twin->origin->index >= numOldVertices)
                && std::find(oldEdges.begin(), oldEdges.end(), he) == oldEdges.end()){
                edgeflip(*he);
            }
            n++;
        }

        numIterations -= 1;
    }
}

void Mesh::simplify(int numTrianglesRemove){
    std::map<Face*, Matrix4f> faceToQ;
    for (auto *f : m_faces){
        Vector3f v1 = f->halfedge->origin->position;
        Vector3f v2 = f->halfedge->next->origin->position;
        Vector3f v3 = f->halfedge->next->next->origin->position;

        // Compute normal
        Vector3f normal = (v2 - v1).cross(v3 - v1).normalized();

        // Compute plane equation coefficients
        float a = normal.x();
        float b = normal.y();
        float c = normal.z();
        float d = -normal.dot(v1);

        // Construct quadric matrix
        Matrix4f Q;
        Q << a * a, a * b, a * c, a * d,
            a * b, b * b, b * c, b * d,
            a * c, b * c, c * c, c * d,
            a * d, b * d, c * d, d * d;

        faceToQ[f] = Q;
    }

    std::map<Vertex*, Matrix4f> vertexToQ;
    for (auto *v : m_vertices){
        std::vector<Face*> neighbors;
        Halfedge *start = v->halfedge;
        Halfedge *he = start;
        do {
            neighbors.push_back(he->face);
            he = he->twin->next;
        } while (he != start);

        Matrix4f newQ;
        newQ << 0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f;
        for (Face *neighbor : neighbors) {
            newQ += faceToQ[neighbor];
        }
        vertexToQ[v] = newQ;
    }

    // Initialize the edge queue with error values
    std::map<Halfedge*, float> halfedgeToErrors;
    HalfedgeErrorComparator comp(halfedgeToErrors);
    std::multiset<Halfedge*, HalfedgeErrorComparator> edgeQueue(comp);
    for (auto *he : m_halfedges) {

        Halfedge *toCollapse = he;
        Halfedge *toCollapseTwin = toCollapse->twin;

        Vertex* v1 = he->origin;
        Vertex* v2 = he->twin->origin;

        Matrix4f Q = vertexToQ[v1] + vertexToQ[v2];

        Vector4f v1Pos(v1->position.x(), v1->position.y(), v1->position.z(), 1.0f);
        Vector4f v2Pos(v2->position.x(), v2->position.y(), v2->position.z(), 1.0f);
        Vector4f edgeVec = v2Pos - v1Pos;
        float error = edgeVec.transpose() * Q * edgeVec;

        halfedgeToErrors[he] = error;
        edgeQueue.insert(he);

    }

    while (numTrianglesRemove > 0){

        Halfedge* halfedgeToCollapse = *edgeQueue.begin();

        // Traverse through all edges from this vertex
        std::vector<Vertex*> toCollapseIndices;
        std::vector<Vertex*> toCollapseTwinIndices;
        Halfedge *firststart = halfedgeToCollapse;
        Halfedge *current = firststart;
        do {
            toCollapseIndices.push_back(current->next->origin);
            current = current->twin->next;
        } while (current != firststart);

        Halfedge *startTwin = halfedgeToCollapse->twin;
        Halfedge *currentTwin = startTwin;
        do {
            toCollapseTwinIndices.push_back(currentTwin->next->origin);
            currentTwin = currentTwin->twin->next;
        } while (currentTwin != startTwin);

        std::pair<int, std::vector<Vertex*>> result = countCommonElements(toCollapseIndices, toCollapseTwinIndices);
        int commonN = result.first;
        if (commonN != 2){
            edgeQueue.erase(halfedgeToCollapse);
            edgeQueue.erase(halfedgeToCollapse->twin);
            continue;
        }
        for (auto *i : result.second){
            if (vertexHalfedgeCount[i] == 3){
                edgeQueue.erase(halfedgeToCollapse);
                edgeQueue.erase(halfedgeToCollapse->twin);
                continue;
            }
        }

        Vertex* v1 = halfedgeToCollapse->origin;
        Vertex* v2 = halfedgeToCollapse->twin->origin;

        Matrix4f Q = vertexToQ[v1] + vertexToQ[v2];

        Vector3f distance = halfedgeToCollapse->origin->position - halfedgeToCollapse->twin->origin->position;
        Vector3f midpoint(halfedgeToCollapse->origin->position[0] + 0.5 * distance[0], halfedgeToCollapse->origin->position[1] + 0.5 * distance[1], halfedgeToCollapse->origin->position[2] + 0.5 * distance[2]);

        Vector3f optimalPosition = computeOptimalPosition(Q, v1->position, v2->position);

        bool collapseornot = true;
        std::vector<Halfedge*> sideHalfedges = {halfedgeToCollapse->next->next->twin, halfedgeToCollapse->twin->next->twin, halfedgeToCollapse->next->twin, halfedgeToCollapse->twin->next->next->twin};
        for (Halfedge* he : sideHalfedges) {
            if (he == halfedgeToCollapse->next->next->twin || halfedgeToCollapse->twin->next->next->twin){
                Face* sideFace = he->face;
                Vector3f newv1 = he->next->next->origin->position;
                Vector3f newv2 = he->twin->origin->position;
                Vector3f newv3 = optimalPosition;
                float dotProduct = antiFlippedTriangle(sideFace, newv1, newv2, newv3);
                if (dotProduct < -1){
                    collapseornot = false;
                }
            }else{
                Face* sideFace = he->face;
                Vector3f newv1 = he->origin->position;
                Vector3f newv2 = he->next->next->origin->position;
                Vector3f newv3 = optimalPosition;
                float dotProduct = antiFlippedTriangle(sideFace, newv1, newv2, newv3);
                if (dotProduct < -1){
                    collapseornot = false;
                }
            }
        }
        if (!collapseornot){
            edgeQueue.erase(halfedgeToCollapse);
            edgeQueue.erase(halfedgeToCollapse->twin);
            continue;
        }

        std::vector<Halfedge*> removedHalfedges = {halfedgeToCollapse, halfedgeToCollapse->twin, halfedgeToCollapse->next->twin, halfedgeToCollapse->next->next->twin, halfedgeToCollapse->twin->next->twin, halfedgeToCollapse->twin->next->next->twin};
        for (Halfedge* he : removedHalfedges) {
            edgeQueue.erase(he);
            halfedgeToErrors.erase(he);
        }

        // Remove the affected edges from the queue
        std::vector<Halfedge*> affectedEdges;
        Halfedge* start = halfedgeToCollapse->origin->halfedge;
        do {
            if (std::find(removedHalfedges.begin(), removedHalfedges.end(), start) == removedHalfedges.end()) {
                affectedEdges.push_back(start);
            }
            start = start->twin->next;
        } while (start != halfedgeToCollapse->origin->halfedge);

        start = halfedgeToCollapse->twin->origin->halfedge;
        do {
            if (std::find(removedHalfedges.begin(), removedHalfedges.end(), start) == removedHalfedges.end()) {
                affectedEdges.push_back(start);
            }
            start = start->twin->next;
        } while (start != halfedgeToCollapse->twin->origin->halfedge);

        //collapse
        collapse(*halfedgeToCollapse, optimalPosition, false);

        //update Q map
        Vertex* newVertex = m_vertices[m_vertices.size() - 1];
        vertexToQ[newVertex] = vertexToQ[halfedgeToCollapse->origin] + vertexToQ[halfedgeToCollapse->twin->origin];
        vertexToQ.erase(halfedgeToCollapse->origin);
        vertexToQ.erase(halfedgeToCollapse->twin->origin);

        // Recalculate errors and reinsert the affected edges
        for (Halfedge* he : affectedEdges) {
            edgeQueue.erase(he); // Remove the old entry from the queue
            Vertex* v1 = he->origin;
            Vertex* v2 = he->twin->origin;
            Matrix4f Q = vertexToQ.at(v1) + vertexToQ.at(v2);

            Vector4f v1Pos(v1->position.x(), v1->position.y(), v1->position.z(), 1.0f);
            Vector4f v2Pos(v2->position.x(), v2->position.y(), v2->position.z(), 1.0f);
            Vector4f edgeVec = v2Pos - v1Pos;
            float error = edgeVec.transpose() * Q * edgeVec;

            halfedgeToErrors[he] = error;
            edgeQueue.insert(he); // Insert the updated entry
        }

        numTrianglesRemove -= 2;


    }
}

Vector3f Mesh::computeOptimalPosition(const Matrix4f& Q, const Vector3f& v1, const Vector3f& v2) {
    // Construct the modified quadric matrix for solving
    Matrix4f Qmod = Q;
    Qmod.row(3) << 0, 0, 0, 1;

    // Solve the system Qmod * v = 0 for v
    if (std::abs(Qmod.determinant()) > 1e-6){
        Vector4f b(0, 0, 0, 1);
        Vector4f v = Qmod.fullPivLu().solve(b);

        // Check if the solution is valid and finite
        if (v[3] != 0 && Qmod * v == b) {
            return Vector3f(v[0] / v[3], v[1] / v[3], v[2] / v[3]);
        }
    }

    // If the optimal solution is not found, try finding the best position along the edge
    Vector3f bestPosition = (v1 + v2) / 2; // Start with the midpoint as the best position
    float bestError = std::numeric_limits<float>::max();

    std::vector<Vector3f> candidates = {v1, v2, (v1 + v2) / 2};
    for (const Vector3f& candidate : candidates) {
        Vector4f candidateHomogeneous(candidate.x(), candidate.y(), candidate.z(), 1);
        float error = candidateHomogeneous.transpose() * Q * candidateHomogeneous;
        if (error < bestError) {
            bestError = error;
            bestPosition = candidate;
        }
    }

    return bestPosition;
}

float Mesh::antiFlippedTriangle(Face* oldFace, const Vector3f& newv1, const Vector3f& newv2, const Vector3f& newv3){
    Vector3f oldv1 = oldFace->halfedge->origin->position;
    Vector3f oldv2 = oldFace->halfedge->next->origin->position;
    Vector3f oldv3 = oldFace->halfedge->next->next->origin->position;

    // Compute normals for old and new triangles
    Vector3f oldNormal = (oldv2 - oldv1).cross(oldv3 - oldv1).normalized();
    Vector3f newNormal = (newv2 - newv1).cross(newv3 - newv1).normalized();

    // Return the dot product of the normals
    return oldNormal.dot(newNormal);
}

void Mesh::remesh(float weight, int numIter){
    while (numIter != 0){
    std::vector<Halfedge*> edgesToSplit;

    // Collect half-edges to split
    for (auto *he : m_halfedges) {
        if (he->twin && std::find(edgesToSplit.begin(), edgesToSplit.end(), he) == edgesToSplit.end()
            && std::find(edgesToSplit.begin(), edgesToSplit.end(), he->twin) == edgesToSplit.end()) {
            edgesToSplit.push_back(he);
        }
    }

    float totalLength = 0;
    for (Halfedge* he : m_halfedges){
        Vector3f start = he->origin->position;
        Vector3f end = he->next->origin->position;
        totalLength += (end - start).norm();
    }
    float meanLength = totalLength / m_halfedges.size();

    for (Halfedge* he : edgesToSplit){
        Vector3f start = he->origin->position;
        Vector3f end = he->next->origin->position;
        float currentlength = (end - start).norm();

        if (currentlength > (4.0f/3.0f) * meanLength){
            edgeSplit(*he);
        }

    }

    std::vector<Halfedge*> edgesToCollapse = m_halfedges;

    for (Halfedge* he : edgesToCollapse){
        if (std::find(m_halfedges.begin(), m_halfedges.end(), he) != m_halfedges.end()){
            Vector3f start = he->origin->position;
            Vector3f end = he->next->origin->position;
            float currentlength = (end - start).norm();

            if (currentlength < 0.8 * meanLength){
                Vertex* v1 = he->origin;
                Vertex* v2 = he->twin->origin;
                Vector3f midPoint = (v1->position + v2->position) / 2;
                collapse(*he, midPoint, true);
            }
        }
    }

    for (Halfedge* he : m_halfedges){

        int originCount = vertexHalfedgeCount[he->origin];
        int twinCount = vertexHalfedgeCount[he->twin->origin];
        int oppositeOneCount = vertexHalfedgeCount[he->next->next->origin];
        int oppositeTwoCount = vertexHalfedgeCount[he->twin->next->next->origin];

        std::vector<int> counts = {originCount, twinCount, oppositeOneCount, oppositeTwoCount};

        // Update counts after flipping
        counts[0] -= 1; // originCount
        counts[1] -= 1; // twinCount
        counts[2] += 1; // oppositeOneCount
        counts[3] += 1; // oppositeTwoCount

        // Check if the total deviation from degree 6 is reduced
        int originalDeviation = std::abs(originCount - 6) + std::abs(twinCount - 6) + std::abs(oppositeOneCount - 6) + std::abs(oppositeTwoCount - 6);
        int newDeviation = std::abs(counts[0] - 6) + std::abs(counts[1] - 6) + std::abs(counts[2] - 6) + std::abs(counts[3] - 6);

        if (newDeviation < originalDeviation) {
            edgeflip(*he);
        }

    }

    std::vector<Vector3f> newPositions(m_vertices.size());
    int i = 0; // Use an index to keep track of the current vertex
    for (auto *v : m_vertices) {
        Halfedge *firststart = v->halfedge;
        Halfedge *current = firststart;
        Vector3f normal = Vector3f(0, 0, 0);
        Vector3f centroid = Vector3f(0, 0, 0);
        int neighborCount = 0; // Initialize a count of the neighbors

        do {
            Face* curr_face = current->face;
            Vector3f faceNormal = computeFaceNormal(*curr_face);
            normal += faceNormal;

            Vertex* neighbor = current->next->origin;
            centroid += neighbor->position;
            neighborCount++; // Increment neighbor count
            current = current->twin->next;
        } while (current && current != firststart);

        normal.normalize();
        centroid /= static_cast<float>(neighborCount);

        Vector3f difference = centroid - v->position;
        Vector3f projectedDifference = difference - (normal.dot(difference)) * normal;
        newPositions[i] = v->position + weight * projectedDifference; // Assign to the index in newPositions
        i++; // Increment the index for the next vertex
    }

    // Assign the new positions to the vertices
    for (size_t j = 0; j < m_vertices.size(); ++j) {
        m_vertices[j]->position = newPositions[j];
    }
    numIter -= 1;
    }

}

Vector3f Mesh::computeFaceNormal(const Face& face) {
    const Vector3f& v1 = face.halfedge->origin->position;
    const Vector3f& v2 = face.halfedge->next->origin->position;
    const Vector3f& v3 = face.halfedge->next->next->origin->position;

    Vector3f edge1 = v2 - v1;
    Vector3f edge2 = v3 - v1;

    return edge1.cross(edge2).normalized();
}

void Mesh::denoise(int numIter, float sigma_s, float sigma_c){

    while (numIter != 0){
        std::map<Vertex*, Vector3f> vertexToNormal;
        for (auto *v : m_vertices) {
            Halfedge *firststart = v->halfedge;
            Halfedge *current = firststart;
            Vector3f normal = Vector3f(0, 0, 0);

            int neighborCount = 0; // Initialize a count of the neighbors

            do {
                Face* curr_face = current->face;
                Vector3f faceNormal = computeFaceNormal(*curr_face);
                normal += faceNormal;

                Vertex* neighbor = current->next->origin;
                neighborCount++; // Increment neighbor count
                current = current->twin->next;
            } while (current && current != firststart);

            normal.normalize();
            vertexToNormal[v] = normal;
        }

        std::map<Vertex*, Vector3f> newPositions;
        for (auto *v : m_vertices) {
            Halfedge *firststart = v->halfedge;
            Halfedge *current = firststart;
            Vector3f sum = Vector3f(0, 0, 0);
            float normalizer = 0.0f;
            int neighborCount = 0; // Initialize a count of the neighbors

            do {

                Vertex* neighbor = current->next->origin;
                float t = (v->position - neighbor->position).norm();
                float h = vertexToNormal[v].dot(neighbor->position - v->position);

                float w_c = std::exp(-t * t / (2.0f * sigma_c * sigma_c));
                float w_s = std::exp(-h * h / (2.0f * sigma_s * sigma_s));

                sum += (w_c * w_s) * h * vertexToNormal[v];
                normalizer += w_c * w_s;
                neighborCount++; // Increment neighbor count
                current = current->twin->next;
            } while (current && current != firststart);
            newPositions[v] = v->position + (sum / normalizer);
        }

        for (auto *v : m_vertices) {
            v->position = newPositions[v];
        }
        numIter -= 1;
    }
}

void Mesh::noise(){

    std::map<Vertex*, Vector3f> vertexToNormal;
    for (auto *v : m_vertices) {
        Halfedge *firststart = v->halfedge;
        Halfedge *current = firststart;
        Vector3f normal = Vector3f(0, 0, 0);

        int neighborCount = 0; // Initialize a count of the neighbors

        do {
            Face* curr_face = current->face;
            Vector3f faceNormal = computeFaceNormal(*curr_face);
            normal += faceNormal;

            Vertex* neighbor = current->next->origin;
            neighborCount++; // Increment neighbor count
            current = current->twin->next;
        } while (current && current != firststart);

        normal.normalize();
        vertexToNormal[v] = normal;
    }

    for (auto *v : m_vertices) {
        float random = static_cast<double>(rand()) / static_cast<double>(RAND_MAX);

        Vector3f normal = vertexToNormal[v];

        // Generate a random displacement
        float displacement = random * 0.01;

        // Add the displacement along the vertex normal to the vertex position
        v->position += normal * displacement;
    }
}
