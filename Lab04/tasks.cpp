#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <spdlog/spdlog.h>
#include <iostream>
#include "Labs/4-Animation/tasks.h"
#include "IKSystem.h"
#include "CustomFunc.inl"


namespace VCX::Labs::Animation {
    void ForwardKinematics(IKSystem & ik, int StartIndex) {
        if (StartIndex == 0) {
            ik.JointGlobalRotation[0] = ik.JointLocalRotation[0];
            ik.JointGlobalPosition[0] = ik.JointLocalOffset[0];
            StartIndex                = 1;
        }
        
        for (int i = StartIndex; i < ik.JointLocalOffset.size(); i++) {
            // your code here: forward kinematics, update JointGlobalPosition and JointGlobalRotation
            glm::vec4 pos=glm::vec4(ik.JointLocalOffset[i], 1.0f);
            pos=glm::mat4_cast(ik.JointGlobalRotation[i-1])*pos;
            ik.JointGlobalPosition[i]=ik.JointGlobalPosition[i-1]+glm::vec3(pos.x,pos.y,pos.z)/pos.w;
            ik.JointGlobalRotation[i]=ik.JointGlobalRotation[i-1]*ik.JointLocalRotation[i];
        }
    }

    void InverseKinematicsCCD(IKSystem & ik, const glm::vec3 & EndPosition, int maxCCDIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        // These functions will be useful: glm::normalize, glm::rotation, glm::quat * glm::quat
        for (int CCDIKIteration = 0; CCDIKIteration < maxCCDIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; CCDIKIteration++) {
            // your code here: ccd ik
            for(int i=ik.NumJoints()-2;i>=0;--i){
                glm::vec3 to_end=ik.EndEffectorPosition()-ik.JointGlobalPosition[i];
                glm::vec3 to_target=EndPosition-ik.JointGlobalPosition[i];
                glm::quat rot=glm::rotation(glm::normalize(to_end),glm::normalize(to_target));
                ik.JointLocalRotation[i]=rot*ik.JointLocalRotation[i];
                ForwardKinematics(ik,i);
            }
            //printf("maxiter=%d,iter=%d,dist=%f\n", maxCCDIKIteration, CCDIKIteration, glm::l2Norm(ik.EndEffectorPosition() - EndPosition));
        }

    }

    void InverseKinematicsFABR(IKSystem & ik, const glm::vec3 & EndPosition, int maxFABRIKIteration, float eps) {
        ForwardKinematics(ik, 0);
        int nJoints = ik.NumJoints();
        std::vector<glm::vec3> backward_positions(nJoints, glm::vec3(0, 0, 0)), forward_positions(nJoints, glm::vec3(0, 0, 0));
        for (int IKIteration = 0; IKIteration < maxFABRIKIteration && glm::l2Norm(ik.EndEffectorPosition() - EndPosition) > eps; IKIteration++) {
            // task: fabr ik
            // backward update
            glm::vec3 next_position         = EndPosition;
            backward_positions[nJoints - 1] = EndPosition;

            for (int i = nJoints - 2; i >= 0; i--) {
                // your code here
                glm::vec3 dir = glm::normalize(ik.JointGlobalPosition[i] - next_position);
                backward_positions[i]=next_position+dir*ik.JointOffsetLength[i+1];
                next_position=backward_positions[i];
            }

            // forward update
            glm::vec3 now_position = ik.JointGlobalPosition[0];
            forward_positions[0] = ik.JointGlobalPosition[0];
            for (int i = 0; i < nJoints - 1; i++) {
                // your code here
                glm::vec3 dir = glm::normalize(backward_positions[i + 1] - now_position);
                forward_positions[i + 1] = now_position + dir * ik.JointOffsetLength[i + 1];
                now_position = forward_positions[i + 1];
            }
            ik.JointGlobalPosition = forward_positions; // copy forward positions to joint_positions
            //printf("maxiter=%d,iter=%d,dist=%f\n", maxFABRIKIteration, IKIteration, glm::l2Norm(ik.EndEffectorPosition() - EndPosition));
        }

        // Compute joint rotation by position here.
        for (int i = 0; i < nJoints - 1; i++) {
            ik.JointGlobalRotation[i] = glm::rotation(glm::normalize(ik.JointLocalOffset[i + 1]), glm::normalize(ik.JointGlobalPosition[i + 1] - ik.JointGlobalPosition[i]));
        }
        ik.JointLocalRotation[0] = ik.JointGlobalRotation[0];
        for (int i = 1; i < nJoints - 1; i++) {
            ik.JointLocalRotation[i] = glm::inverse(ik.JointGlobalRotation[i - 1]) * ik.JointGlobalRotation[i];
        }
        ForwardKinematics(ik, 0);
    }

    IKSystem::Vec3ArrPtr IKSystem::BuildCustomTargetPosition() {
        int nums=5000;
        using Vec3Arr=std::vector<glm::vec3>;
        std::shared_ptr<Vec3Arr> custom(new Vec3Arr(nums));
        int index=0;
        for(int i=0;i<nums;i++){
            float t=2.0f*glm::pi<float>()*i/nums;
            float x_val=16.0f*std::pow(std::sin(t),3);
            float y_val=13.0f*std::cos(t)-5.0f*std::cos(2.0f*t)-2.0f*std::cos(3.0f*t)-std::cos(4.0f*t);
            float scale=0.05f;
            x_val*=scale;
            y_val*=scale;
            (*custom)[index++]=glm::vec3(0.6f-x_val,0.0f,y_val+0.5f);
        }
        custom->resize(index);
        return custom;
    }

    static Eigen::VectorXf glm2eigen(std::vector<glm::vec3> const & glm_v) {
        Eigen::VectorXf v = Eigen::Map<Eigen::VectorXf const, Eigen::Aligned>(reinterpret_cast<float const *>(glm_v.data()), static_cast<int>(glm_v.size() * 3));
        return v;
    }

    static std::vector<glm::vec3> eigen2glm(Eigen::VectorXf const & eigen_v) {
        return std::vector<glm::vec3>(
            reinterpret_cast<glm::vec3 const *>(eigen_v.data()),
            reinterpret_cast<glm::vec3 const *>(eigen_v.data() + eigen_v.size())
        );
    }

    static Eigen::SparseMatrix<float> CreateEigenSparseMatrix(std::size_t n, std::vector<Eigen::Triplet<float>> const & triplets) {
        Eigen::SparseMatrix<float> matLinearized(n, n);
        matLinearized.setFromTriplets(triplets.begin(), triplets.end());
        return matLinearized;
    }

    // solve Ax = b and return x
    static Eigen::VectorXf ComputeSimplicialLLT(
        Eigen::SparseMatrix<float> const & A,
        Eigen::VectorXf const & b) {
        auto solver = Eigen::SimplicialLLT<Eigen::SparseMatrix<float>>(A);
        return solver.solve(b);
    }

    void AdvanceMassSpringSystem(MassSpringSystem & system, float const dt) {
        int const steps = 6;
        float const h = dt / steps; 
        for (std::size_t s = 0; s < steps; s++) {
            int n = static_cast<int>(system.Positions.size());
            Eigen::VectorXf x0=glm2eigen(system.Positions);
            Eigen::VectorXf v0=glm2eigen(system.Velocities);
            Eigen::VectorXf gravity(n * 3);
            gravity.setZero();
            for (int i = 0; i < n; i++)
                if (!system.Fixed[i])
                    gravity[3*i+1]=-system.Gravity;
            Eigen::VectorXf y=x0+h*v0+h*h*gravity;//x_{n+1}=y+h^2/M*f_{int}(x_{n+1}),f_ext/M=gravity
            Eigen::VectorXf x_next=x0;
            int iter_max=8;           
            for(int iter=0;iter<iter_max;iter++){
                Eigen::VectorXf f_int(n * 3);
                f_int.setZero();
                std::vector<Eigen::Triplet<float>> triplets;
                for(auto const & spring:system.Springs){
                    int p0=spring.AdjIdx.first;
                    int p1=spring.AdjIdx.second;
                    Eigen::Vector3f p1_pos=x_next.segment<3>(3*p1);
                    Eigen::Vector3f p0_pos=x_next.segment<3>(3*p0);
                    Eigen::Vector3f x01=p1_pos-p0_pos;
                    float length=x01.norm();
                    Eigen::Vector3f e01=x01.normalized();
                    Eigen::Vector3f p1_pos_old=x0.segment<3>(3*p1);
                    Eigen::Vector3f p0_pos_old=x0.segment<3>(3*p0);
                    Eigen::Vector3f v01=((p1_pos-p1_pos_old)-(p0_pos-p0_pos_old))/h;
                    float damping_force=system.Damping*e01.dot(v01);
                    Eigen::Vector3f f=(system.Stiffness*(length-spring.RestLength)+damping_force)*e01;
                    if (!system.Fixed[p0])f_int.segment<3>(3*p0)+=f;
                    if (!system.Fixed[p1])f_int.segment<3>(3*p1)-=f;//f_int=-∇E

                    //compute hessian of spring energy
                    Eigen::Matrix3f I=Eigen::Matrix3f::Identity(),
                                    eeT=e01*e01.transpose();
                    assert(length>1e-6);
                    Eigen::Matrix3f H_e=system.Stiffness*((1 - spring.RestLength/length)*(I - eeT) + eeT);
                    H_e+=(system.Damping/h)*eeT;//Damping term
                    if(!system.Fixed[p0])
                        for(int i=0;i<3;++i)
                            for(int j=0;j<3;++j)
                                triplets.emplace_back(3*p0+i,3*p0+j,H_e(i,j));
                    if(!system.Fixed[p1])
                        for(int i=0;i<3;++i)
                            for(int j=0;j<3;++j)
                                triplets.emplace_back(3*p1+i,3*p1+j,H_e(i,j));
                    if (!system.Fixed[p0]&&!system.Fixed[p1]) {
                        for(int i=0;i<3;++i) for(int j=0;j<3;++j) {
                            triplets.emplace_back(3*p0+i,3*p1+j,-H_e(i,j));
                            triplets.emplace_back(3*p1+i,3*p0+j,-H_e(i,j));
                        }
                    }
                }
                Eigen::VectorXf nabla_g=x_next-y-h*h*f_int/system.Mass;//∇g(x)
                auto H_E=CreateEigenSparseMatrix(n * 3, triplets);
                Eigen::SparseMatrix<float> nabla2_g=Eigen::SparseMatrix<float>(n * 3, n * 3);
                nabla2_g.setIdentity();
                nabla2_g+=h*h*H_E/system.Mass;//H=I+h^2/M*H_E
                Eigen::VectorXf dx=ComputeSimplicialLLT(nabla2_g,-nabla_g);
                for(int i=0;i<n;i++)
                    if(system.Fixed[i])
                        dx.segment<3>(3*i).setZero();
                x_next+=dx;
                if (dx.norm() < 1e-4f) break;
            }
            Eigen::VectorXf v_next=(x_next - x0)/h;
            system.Positions=eigen2glm(x_next);
            system.Velocities=eigen2glm(v_next);
        }
    }
}
