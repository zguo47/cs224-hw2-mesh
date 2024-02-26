## Mesh (milestone submission)

Please fill this out and submit your work to Gradescope by the milestone deadline.

### Mesh Validator
My mesh validator checks if the provided mesh data structure is valid. I implemented the half-edge structure, 
I defined a halfedge to contain its twin halfedge, the next halfedge it's pointing to, the origin vertex,
and the face it's on. For each Vertex, its field contains its index, which should also be its index in the 
vector of all vertices; the halfedge it's pointing to (arbitrary one); and its position vector. For each face,
its field contains the (arbitrary) halfedge pointing to it, and the face as a vector. 
My validator contains 17 assert statements that check on the above logic. 
To start, I check that size of halfedges should be 3 times size of faces (since each face correspond to 3 halfedges).
Second, I make sure for all halfedges, vertices and faces, every field should not be a nullptr, since we 
assume we are working on a closed geometry, and no operations should cause non-manifold. 
Third, I check that each halfedge is twin with its twin, that is, they are twins to each other. Also, 
the origin of each halfedge's twin should be the origin of its next halfedge. 
Fourth, I check that starting any halfedge and traverse through "next", it should perform a closed loop.
Fifth, I make sure that starting from any vertex and traverse the halfedges, there is also a closed loop.
Last, I make sure that there exist no zero area faces, in other words, the vertices of each face should be
unique. 
The above features a total of 17 assert statements.
The below are the detailed implementation. 

struct Halfedge {
    Halfedge *twin = nullptr;
    Halfedge *next = nullptr;
    Vertex *origin = nullptr;
    Face *face = nullptr
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

### Collaboration/References
N/A

### Known Bugs
N/A