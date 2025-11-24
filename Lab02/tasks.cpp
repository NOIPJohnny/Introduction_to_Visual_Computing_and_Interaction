#include <unordered_map>

#include <glm/gtc/matrix_inverse.hpp>
#include <spdlog/spdlog.h>

#include "Labs/2-GeometryProcessing/DCEL.hpp"
#include "Labs/2-GeometryProcessing/tasks.h"

namespace VCX::Labs::GeometryProcessing {

#include "Labs/2-GeometryProcessing/marching_cubes_table.h"

    /******************* 1. Mesh Subdivision *****************/
    void SubdivisionMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations) {
        Engine::SurfaceMesh curr_mesh = input;
        // We do subdivison iteratively.
        for (std::uint32_t it = 0; it < numIterations; ++it) {
            // During each iteration, we first move curr_mesh into prev_mesh.
            Engine::SurfaceMesh prev_mesh;
            prev_mesh.Swap(curr_mesh);
            // Then we create doubly connected edge list.
            DCEL G(prev_mesh);
            if (! G.IsManifold()) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Non-manifold mesh.");
                return;
            }
            // Note that here curr_mesh has already been empty.
            // We reserve memory first for efficiency.
            curr_mesh.Positions.reserve(prev_mesh.Positions.size() * 3 / 2);
            curr_mesh.Indices.reserve(prev_mesh.Indices.size() * 4);
            // Then we iteratively update currently existing vertices.
            for (std::size_t i = 0; i < prev_mesh.Positions.size(); ++i) {
                // Update the currently existing vetex v from prev_mesh.Positions.
                // Then add the updated vertex into curr_mesh.Positions.
                auto v           = G.Vertex(i);
                auto neighbors   = v->Neighbors();
                // your code here:
                glm::vec3 updated_v = glm::vec3(0.0f);
                if(v->OnBoundary())
                {
                    glm::vec3 boundary_neighbor_sum(0.0f);
                    int boundary_count = 0;
                    for(auto neighbor_idx:neighbors)
                    {
                        auto neighbor_vertex = G.Vertex(neighbor_idx);
                        if(neighbor_vertex->OnBoundary())
                        {
                            boundary_neighbor_sum += prev_mesh.Positions[neighbor_idx];
                            boundary_count++;
                        }
                    }
                    updated_v=0.75f*prev_mesh.Positions[i]+0.25f*(boundary_neighbor_sum/static_cast<float>(boundary_count));
                }
                else
                {
                    float n = static_cast<float>(neighbors.size());
                    float beta = (n == 3.0f) ? 3.0f / 16.0f : 3.0f / (8.0f * n);
                    glm::vec3 neighbor_sum(0.0f);
                    for(auto neighbor_idx:neighbors)
                        neighbor_sum += prev_mesh.Positions[neighbor_idx];
                    updated_v = (1.0f - n * beta) * prev_mesh.Positions[i] + beta * neighbor_sum;
                }
                curr_mesh.Positions.push_back(updated_v);
            }
            // We create an array to store indices of the newly generated vertices.
            // Note: newIndices[i][j] is the index of vertex generated on the "opposite edge" of j-th
            //       vertex in the i-th triangle.
            std::vector<std::array<std::uint32_t, 3U>> newIndices(prev_mesh.Indices.size() / 3, { ~0U, ~0U, ~0U });
            // Iteratively process each halfedge.
            for (auto e : G.Edges()) {
                // newIndices[face index][vertex index] = index of the newly generated vertex
                newIndices[G.IndexOf(e->Face())][e->EdgeLabel()] = curr_mesh.Positions.size();
                auto eTwin                                       = e->TwinEdgeOr(nullptr);
                // eTwin stores the twin halfedge.
                if (! eTwin) {
                    // When there is no twin halfedge (so, e is a boundary edge):
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 new_vertex = 0.5f * (prev_mesh.Positions[e->From()] + prev_mesh.Positions[e->To()]);
                    curr_mesh.Positions.push_back(new_vertex);
                } else {
                    // When the twin halfedge exists, we should also record:
                    //     newIndices[face index][vertex index] = index of the newly generated vertex
                    // Because G.Edges() will only traverse once for two halfedges,
                    //     we have to record twice.
                    newIndices[G.IndexOf(eTwin->Face())][e->TwinEdge()->EdgeLabel()] = curr_mesh.Positions.size();
                    // your code here: generate the new vertex and add it into curr_mesh.Positions.
                    glm::vec3 p1=prev_mesh.Positions[e->From()];
                    glm::vec3 p2=prev_mesh.Positions[e->To()];
                    glm::vec3 p3=prev_mesh.Positions[e->NextEdge()->To()];
                    glm::vec3 p4=prev_mesh.Positions[eTwin->NextEdge()->To()];
                    glm::vec3 new_vertex = 0.375f * (p1 + p2) + 0.125f * (p3 + p4);
                    curr_mesh.Positions.push_back(new_vertex);
                }
            }

            // Here we've already build all the vertices.
            // Next, it's time to reconstruct face indices.
            for (std::size_t i = 0; i < prev_mesh.Indices.size(); i += 3U) {
                // For each face F in prev_mesh, we should create 4 sub-faces.
                // v0,v1,v2 are indices of vertices in F.
                // m0,m1,m2 are generated vertices on the edges of F.
                auto v0           = prev_mesh.Indices[i + 0U];
                auto v1           = prev_mesh.Indices[i + 1U];
                auto v2           = prev_mesh.Indices[i + 2U];
                auto [m0, m1, m2] = newIndices[i / 3U];
                // Note: m0 is on the opposite edge (v1-v2) to v0.
                // Please keep the correct indices order (consistent with order v0-v1-v2)
                //     when inserting new face indices.
                // toInsert[i][j] stores the j-th vertex index of the i-th sub-face.
                std::uint32_t toInsert[4][3] = {
                    // your code here:
                    {v0,m2,m1},
                    {v1,m0,m2},
                    {v2,m1,m0},
                    {m0,m1,m2}
                };
                // Do insertion.
                curr_mesh.Indices.insert(
                    curr_mesh.Indices.end(),
                    reinterpret_cast<std::uint32_t *>(toInsert),
                    reinterpret_cast<std::uint32_t *>(toInsert) + 12U
                );
            }

            if (curr_mesh.Positions.size() == 0) {
                spdlog::warn("VCX::Labs::GeometryProcessing::SubdivisionMesh(..): Empty mesh.");
                output = input;
                return;
            }
        }
        // Update output.
        output.Swap(curr_mesh);
    }

    /******************* 2. Mesh Parameterization *****************/
    void Parameterization(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, const std::uint32_t numIterations) {
        // Copy.
        output = input;
        // Reset output.TexCoords.
        output.TexCoords.resize(input.Positions.size(), glm::vec2 { 0 });

        // Build DCEL.
        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): non-manifold mesh.");
            return;
        }

        // Set boundary UVs for boundary vertices.
        // your code here: directly edit output.TexCoords
        std::vector<DCEL::VertexIdx> boundary_vertices;//ordered boundary vertices
        DCEL::HalfEdge const * start_edge = nullptr;
        for(auto e:G.Edges())
        {
            if(!e->TwinEdgeOr(nullptr))
            {
                start_edge=e;
                break;
            }
        }
        if(start_edge)
        {
            DCEL::HalfEdge const * curr_edge = start_edge;
            do
            {
                boundary_vertices.push_back(curr_edge->From());
                curr_edge=curr_edge->NextEdge();
                while(curr_edge->TwinEdgeOr(nullptr))
                    curr_edge=curr_edge->TwinEdge()->NextEdge();
            }while(curr_edge!=start_edge);
        }
        if(boundary_vertices.empty())
        {
            spdlog::warn("VCX::Labs::GeometryProcessing::Parameterization(..): no boundary found.");
            return;
        }
        std::vector<float> edge_lengths;
        float perimeter=0.0f;
        for(std::size_t i=0;i<boundary_vertices.size();++i)
        {
            auto v_curr=boundary_vertices[i];
            auto v_next=boundary_vertices[(i+1)%boundary_vertices.size()];
            float length=glm::length(input.Positions[v_curr]-input.Positions[v_next]);
            edge_lengths.push_back(length);
            perimeter+=length;
        }
        float cumulative_length=0.0f;
        for(std::size_t i=0;i<boundary_vertices.size();++i)
        {
            auto v_idx=boundary_vertices[i];
            float t=cumulative_length/perimeter;
            float angle=t*2.0f*glm::pi<float>();
            cumulative_length+=edge_lengths[i];
            output.TexCoords[v_idx]=glm::vec2(0.5f+0.5f*std::cos(angle),0.5f+0.5f*std::sin(angle));
        }
        // Solve equation via Gauss-Seidel Iterative Method.
        for (int k = 0; k < numIterations; ++k) {
            // your code here:
            for(std::size_t i=0;i<input.Positions.size();++i)
            {
                auto v=G.Vertex(i);
                if(v->OnBoundary())
                    continue;
                auto neighbors=v->Neighbors();
                glm::vec2 sum_uv(0.0f);
                for(auto neighbor_idx:neighbors)
                    sum_uv += output.TexCoords[neighbor_idx];
                output.TexCoords[i] = sum_uv / static_cast<float>(neighbors.size());
            }
        }
    }

    /******************* 3. Mesh Simplification *****************/
    void SimplifyMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, float simplification_ratio) {

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Non-watertight mesh.");
            return;
        }

        // Copy.
        output = input;

        // Compute Kp matrix of the face f.
        auto UpdateQ {
            [&G, &output] (DCEL::Triangle const * f) -> glm::mat4 {
                glm::mat4 Kp;
                // your code here:
                auto v0 = f->VertexIndex(0);
                auto v1 = f->VertexIndex(1);
                auto v2 = f->VertexIndex(2);
                glm::vec3 p0 = output.Positions[v0];
                glm::vec3 p1 = output.Positions[v1];
                glm::vec3 p2 = output.Positions[v2];
                glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
                float d = -glm::dot(normal, p0);
                glm::vec4 plane_eq(normal, d);
                Kp = glm::outerProduct(plane_eq, plane_eq);
                return Kp;
            }
        };

        // The struct to record contraction info.
        struct ContractionPair {
            DCEL::HalfEdge const * edge;            // which edge to contract; if $edge == nullptr$, it means this pair is no longer valid
            glm::vec4              targetPosition;  // the targetPosition $v$ for vertex $edge->From()$ to move to
            float                  cost;            // the cost $v.T * Qbar * v$
        };

        // Given an edge (v1->v2), the positions of its two endpoints (p1, p2) and the Q matrix (Q1+Q2),
        //     return the ContractionPair struct.
        static constexpr auto MakePair {
            [] (DCEL::HalfEdge const * edge,
                glm::vec3 const & p1,
                glm::vec3 const & p2,
                glm::mat4 const & Q
            ) -> ContractionPair {
                // your code here:
                glm::mat4 Qbar = Q;
                // Qbar[3] = glm::vec4(0.0f, 0.0f, 0.0f, 1.0f); //is wrong
                Qbar[0][3] = 0.0f;
                Qbar[1][3] = 0.0f;
                Qbar[2][3] = 0.0f;
                Qbar[3][3] = 1.0f;
                glm::vec4 target_position;
                float cost;
                if (std::abs(glm::determinant(Qbar))<=1e-3f) //not invertible
                {
                    glm::vec3 mid_point = 0.5f * (p1 + p2);
                    float costmid= glm::dot(glm::vec4(mid_point, 1.0f), Q * glm::vec4(mid_point, 1.0f));
                    glm::vec4 p1_homogeneous = glm::vec4(p1, 1.0f);
                    float costp1= glm::dot(p1_homogeneous, Q * p1_homogeneous);
                    glm::vec4 p2_homogeneous = glm::vec4(p2, 1.0f);
                    float costp2= glm::dot(p2_homogeneous, Q * p2_homogeneous);
                    if(costmid<=costp1 && costmid<=costp2)
                    {
                        target_position = glm::vec4(mid_point, 1.0f);
                        cost=costmid;
                    }
                    else if(costp1<=costmid && costp1<=costp2)
                    {
                        target_position = p1_homogeneous;
                        cost=costp1;
                    }
                    else
                    {
                        target_position = p2_homogeneous;
                        cost=costp2;
                    }
                }
                else 
                {
                    glm::mat4 Qbar_inv = glm::inverse(Qbar);
                    target_position = Qbar_inv * glm::vec4(0.0f, 0.0f, 0.0f, 1.0f);
                    cost= glm::dot(target_position, Q * target_position);
                }
                float w=target_position.w;
                if(std::abs(w)>1e-6f)
                    target_position /= w;
                return ContractionPair{ edge, target_position, cost };
            }
        };

        // pair_map: map EdgeIdx to index of $pairs$
        // pairs:    store ContractionPair
        // Qv:       $Qv[idx]$ is the Q matrix of vertex with index $idx$
        // Kf:       $Kf[idx]$ is the Kp matrix of face with index $idx$
        std::unordered_map<DCEL::EdgeIdx, std::size_t> pair_map; 
        std::vector<ContractionPair>                  pairs; 
        std::vector<glm::mat4>                         Qv(G.NumOfVertices(), glm::mat4(0));
        std::vector<glm::mat4>                         Kf(G.NumOfFaces(),    glm::mat4(0));

        // Initially, we compute Q matrix for each faces and it accumulates at each vertex.
        for (auto f : G.Faces()) {
            auto Q                 = UpdateQ(f);
            Qv[f->VertexIndex(0)] += Q;
            Qv[f->VertexIndex(1)] += Q;
            Qv[f->VertexIndex(2)] += Q;
            Kf[G.IndexOf(f)]       = Q;
        }

        pair_map.reserve(G.NumOfFaces() * 3);
        pairs.reserve(G.NumOfFaces() * 3 / 2);

        // Initially, we make pairs from all the contractable edges.
        for (auto e : G.Edges()) {
            if (! G.IsContractable(e)) continue;
            auto v1                            = e->From();
            auto v2                            = e->To();
            auto pair                          = MakePair(e, input.Positions[v1], input.Positions[v2], Qv[v1] + Qv[v2]);
            pair_map[G.IndexOf(e)]             = pairs.size();
            pair_map[G.IndexOf(e->TwinEdge())] = pairs.size();
            pairs.emplace_back(pair);
        }

        // Loop until the number of vertices is less than $simplification_ratio * initial_size$.
        while (G.NumOfVertices() > simplification_ratio * Qv.size()) {
            // Find the contractable pair with minimal cost.
            std::size_t min_idx = ~0;
            for (std::size_t i = 1; i < pairs.size(); ++i) {
                if (! pairs[i].edge) continue;
                if (!~min_idx || pairs[i].cost < pairs[min_idx].cost) {
                    if (G.IsContractable(pairs[i].edge)) min_idx = i;
                    else pairs[i].edge = nullptr;
                }
            }
            if (!~min_idx) break;

            // top:    the contractable pair with minimal cost
            // v1:     the reserved vertex
            // v2:     the removed vertex
            // result: the contract result
            // ring:   the edge ring of vertex v1
            ContractionPair & top    = pairs[min_idx];
            auto               v1     = top.edge->From();
            auto               v2     = top.edge->To();
            auto               result = G.Contract(top.edge);
            auto               ring   = G.Vertex(v1)->Ring();

            top.edge             = nullptr;            // The contraction has already been done, so the pair is no longer valid. Mark it as invalid.
            output.Positions[v1] = top.targetPosition; // Update the positions.

            // We do something to repair $pair_map$ and $pairs$ because some edges and vertices no longer exist.
            for (int i = 0; i < 2; ++i) {
                DCEL::EdgeIdx removed           = G.IndexOf(result.removed_edges[i].first);
                DCEL::EdgeIdx collapsed         = G.IndexOf(result.collapsed_edges[i].second);
                pairs[pair_map[removed]].edge   = result.collapsed_edges[i].first;
                pairs[pair_map[collapsed]].edge = nullptr;
                pair_map[collapsed]             = pair_map[G.IndexOf(result.collapsed_edges[i].first)];
            }

            // For the two wing vertices, each of them lose one incident face.
            // So, we update the Q matrix.
            Qv[result.removed_faces[0].first] -= Kf[G.IndexOf(result.removed_faces[0].second)];
            Qv[result.removed_faces[1].first] -= Kf[G.IndexOf(result.removed_faces[1].second)];

            // For the vertex v1, Q matrix should be recomputed.
            // And as the position of v1 changed, all the vertices which are on the ring of v1 should update their Q matrix as well.
            Qv[v1] = glm::mat4(0);
            for (auto e : ring) {
                // your code here:
                //     1. Compute the new Kp matrix for $e->Face()$.
                auto f = e->Face();
                auto faceidx=G.IndexOf(f);
                auto old_Kp = Kf[faceidx];
                auto new_Kp = UpdateQ(f);
                //     2. According to the difference between the old Kp (in $Kf$) and the new Kp (computed in step 1),
                //        update Q matrix of each vertex on the ring (update $Qv$).
                //     3. Update Q matrix of vertex v1 as well (update $Qv$).

                auto delta_Kp = new_Kp - old_Kp;
                auto i0= f->VertexIndex(0);
                auto i1= f->VertexIndex(1);
                auto i2= f->VertexIndex(2);
                if (i0 == v1)Qv[i0] += new_Kp;
                else Qv[i0] += (new_Kp - old_Kp);    
                if (i1 == v1)Qv[i1] += new_Kp;
                else Qv[i1] += (new_Kp - old_Kp);
                if (i2 == v1)Qv[i2] += new_Kp;
                else Qv[i2] += (new_Kp - old_Kp);
                //     4. Update $Kf$.
                Kf[faceidx] = new_Kp;
            }

            // Finally, as the Q matrix changed, we should update the relative $ContractionPair$ in $pairs$.
            // Any pair with the Q matrix of its endpoints changed, should be remade by $MakePair$.
            // your code here:
            for (auto e : ring) {
                auto edge_idx = G.IndexOf(e);
                if (pair_map.find(edge_idx) != pair_map.end()) {
                    auto & pair = pairs[pair_map[edge_idx]];
                    if (pair.edge) {
                        auto v1 = pair.edge->From();
                        auto v2 = pair.edge->To();
                        pair = MakePair(pair.edge, output.Positions[v1], output.Positions[v2], Qv[v1] + Qv[v2]);
                    }
                }
                auto twin = e->TwinEdgeOr(nullptr);
                if (twin) {
                    auto twin_edge_idx = G.IndexOf(twin);
                    if (pair_map.find(twin_edge_idx) != pair_map.end()) {
                        auto & pair = pairs[pair_map[twin_edge_idx]];
                        if (pair.edge) {
                            auto v1 = pair.edge->From();
                            auto v2 = pair.edge->To();
                            pair = MakePair(pair.edge, output.Positions[v1], output.Positions[v2], Qv[v1] + Qv[v2]);
                        }
                    }
                }
            }
        }

        // In the end, we check if the result mesh is watertight and manifold.
        if (! G.DebugWatertightManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SimplifyMesh(..): Result is not watertight manifold.");
        }

        auto exported = G.ExportMesh();
        output.Indices.swap(exported.Indices);
    }

    /******************* 4. Mesh Smoothing *****************/
    void SmoothMesh(Engine::SurfaceMesh const & input, Engine::SurfaceMesh & output, std::uint32_t numIterations, float lambda, bool useUniformWeight) {
        // Define function to compute cotangent value of the angle v1-vAngle-v2
        static constexpr auto GetCotangent {
            [] (glm::vec3 vAngle, glm::vec3 v1, glm::vec3 v2) -> float {
                // your code here:
                glm::vec3 p1=v1-vAngle;
                glm::vec3 p2=v2-vAngle;
                float p1p2_length=glm::length(p1)*glm::length(p2);
                float cos_theta = glm::dot(p1, p2)/p1p2_length;
                float sin_theta = glm::length(glm::cross(p1, p2))/p1p2_length;
                if(std::abs(sin_theta)>1e-6f)
                    return cos_theta/sin_theta;
                return 0.0f;
            }
        };

        DCEL G(input);
        if (! G.IsManifold()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-manifold mesh.");
            return;
        }
        // We only allow watertight mesh.
        if (! G.IsWatertight()) {
            spdlog::warn("VCX::Labs::GeometryProcessing::SmoothMesh(..): Non-watertight mesh.");
            return;
        }

        Engine::SurfaceMesh prev_mesh;
        prev_mesh.Positions = input.Positions;
        for (std::uint32_t iter = 0; iter < numIterations; ++iter) {
            Engine::SurfaceMesh curr_mesh = prev_mesh;
            for (std::size_t i = 0; i < input.Positions.size(); ++i) {
                // your code here: curr_mesh.Positions[i] = ...
                curr_mesh.Positions[i] = prev_mesh.Positions[i];
                auto v = G.Vertex(i);
                glm::vec3 laplacian(0.0f);
                float weight_sum=0.0f;
                for(auto e:v->Ring())
                {
                    auto vj_idx=e->To();
                    if(useUniformWeight)
                    {
                        laplacian+=prev_mesh.Positions[vj_idx];
                        weight_sum+=1.0f;
                    }
                    else
                    {
                        auto v_alpha_idx=e->From();
                        auto v_beta_idx=e->NextEdge()->TwinEdge()->NextEdge()->To();
                        glm::vec3 v_alpha=prev_mesh.Positions[v_alpha_idx];
                        glm::vec3 v_beta=prev_mesh.Positions[v_beta_idx];
                        glm::vec3 vi=prev_mesh.Positions[i];
                        glm::vec3 vj=prev_mesh.Positions[vj_idx];
                        float cot_alpha=GetCotangent(v_alpha,vi,vj);
                        float cot_beta=GetCotangent(v_beta,vi,vj);
                        laplacian+=(cot_alpha+cot_beta)*prev_mesh.Positions[vj_idx];
                        weight_sum+=(cot_alpha+cot_beta);
                    }
                }
                if(weight_sum>1e-6f)
                    laplacian/=weight_sum;
                curr_mesh.Positions[i]=lambda*laplacian+(1.0f-lambda)*prev_mesh.Positions[i];
            }
            // Move curr_mesh to prev_mesh.
            prev_mesh.Swap(curr_mesh);
        }
        // Move prev_mesh to output.
        output.Swap(prev_mesh);
        // Copy indices from input.
        output.Indices = input.Indices;
    }

    /******************* 5. Marching Cubes *****************/
    void MarchingCubes(Engine::SurfaceMesh & output, const std::function<float(const glm::vec3 &)> & sdf, const glm::vec3 & grid_min, const float dx, const int n) {
        // your code here:
        output.Positions.clear();
        output.Indices.clear();
        glm::vec3 unit[3]={
            glm::vec3(1.0f,0.0f,0.0f),
            glm::vec3(0.0f,1.0f,0.0f),
            glm::vec3(0.0f,0.0f,1.0f)
        };
        //int point_idx[n+1][n+1][n+1][3];//store the point idx generated at each cube
        std::vector<std::vector<std::vector<std::array<int,3>>>>point_idx(n+1,std::vector<std::vector<std::array<int,3>>>(n+1,std::vector<std::array<int,3>>(n+1,{-1,-1,-1})));
        for(int nx=0;nx<n;++nx)
            for(int ny=0;ny<n;++ny)
                for(int nz=0;nz<n;++nz)
                {
                    std::vector<glm::vec3> cube_vertices(8);
                    std::vector<float> cube_sdf(8);
                    std::vector<int> edge_vertex_idx(12,-1);
                    cube_vertices[0]=grid_min+glm::vec3(nx*dx,ny*dx,nz*dx);
                    for(int i=0;i<8;++i)
                        cube_vertices[i]=grid_min+glm::vec3((nx+(i&1))*dx,(ny+((i>>1)&1))*dx,(nz+(i>>2))*dx);
                    for(int i=0;i<8;++i)
                        cube_sdf[i]=sdf(cube_vertices[i]);
                    int v_binary_index=0;
                    for(int i=7;i>=0;--i)
                    {
                        if(cube_sdf[i]>0.0f)
                            v_binary_index++;
                        v_binary_index<<=1;
                    }
                    v_binary_index>>=1;
                    int edge_flags=c_EdgeStateTable[v_binary_index];
                    if(edge_flags==0)continue;
                    for(int i=0;i<12;++i)
                    {
                        int unit_num[]={0,0,0};
                        unit_num[((i>>2)+1)%3]=i&1;
                        unit_num[((i>>2)+2)%3]=(i>>1)&1;
                        if(edge_flags&(1<<i))
                        {
                            glm::vec3 pstart,pend,pnew;
                            if(point_idx[nx+unit_num[0]][ny+unit_num[1]][nz+unit_num[2]][i>>2]==-1)
                            {
                                point_idx[nx+unit_num[0]][ny+unit_num[1]][nz+unit_num[2]][i>>2]=output.Positions.size();
                                pstart=dx*glm::vec3{unit_num[0],unit_num[1],unit_num[2]}+cube_vertices[0];
                                pend=pstart+unit[(i>>2)]*dx;
                                float sdf_start=sdf(pstart);
                                float sdf_end=sdf(pend);
                                float t=std::abs(sdf_start)/(std::abs(sdf_start-sdf_end));
                                pnew=(1.0f-t)*pstart+t*pend;
                                output.Positions.push_back(pnew);
                                edge_vertex_idx[i]=output.Positions.size()-1;
                            }
                            else edge_vertex_idx[i]=point_idx[nx+unit_num[0]][ny+unit_num[1]][nz+unit_num[2]][i>>2];
                        }
                    }
                    for(int i=0;i<5;++i)
                    {
                        uint32_t e0=c_EdgeOrdsTable[v_binary_index][3*i];
                        uint32_t e1=c_EdgeOrdsTable[v_binary_index][3*i+1];
                        uint32_t e2=c_EdgeOrdsTable[v_binary_index][3*i+2];
                        if(e0==-1)break;
                        output.Indices.push_back(edge_vertex_idx[e0]);
                        output.Indices.push_back(edge_vertex_idx[e1]);
                        output.Indices.push_back(edge_vertex_idx[e2]);
                    }
                }
    }
} // namespace VCX::Labs::GeometryProcessing
