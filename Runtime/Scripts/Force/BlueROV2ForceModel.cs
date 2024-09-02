using System;
using DefaultNamespace.LookUpTable;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

namespace DefaultNamespace
{
    public class BlueROV2ForceModel : MonoBehaviour
    {
        public ArticulationBody mainBody;
        public ArticulationBody propeller_front_left_top;
        public ArticulationBody propeller_front_right_top;
        public ArticulationBody propeller_back_left_top;
        public ArticulationBody propeller_back_right_top;
        public ArticulationBody propeller_front_left_bottom;
        public ArticulationBody propeller_front_right_bottom;
        public ArticulationBody propeller_back_left_bottom;
        public ArticulationBody propeller_back_right_bottom;

        public float rpm_front_left_top = 0.0f;
        public float rpm_front_right_top = 0.0f;
        public float rpm_back_left_top = 0.0f;
        public float rpm_back_right_top = 0.0f;
        public float rpm_front_left_bottom = 0.0f;
        public float rpm_front_right_bottom = 0.0f;
        public float rpm_back_left_bottom = 0.0f;
        public float rpm_back_right_bottom = 0.0f;

        public void Start()
        {
            // Inertia tensor
            Matrix<double> I_o = DenseMatrix.OfArray(new double[,]
            {
                { rigidBody.inertiaTensor.x, 0, 0 },
                { 0, rigidBody.inertiaTensor.z, 0 },
                { 0, 0, -rigidBody.inertiaTensor.y } //Here we use the inertia configured in Unity. Could replace if you want.
            });

            // # Hydrodynamic coefficients. Damping
            // # Mostly pressure drag.
            var m = 13.5; // [kg]. Previous: rigidBody.mass; 
            var W = m * 9.81; // In OSBS they use g = 9.82
            var B = W + vbs * 1.5;
            // Added from OSBS
            var Xuu = 141; // #1.0
            var Yvv = 217; // #100.0
            var Zww = 190; // #100.0
            var Kpp = 1.19; // #10.0
            var Mqq = 0.47; // #100.0
            var Nrr = 1.5; // #150.0
            
            var Xu = 13.7;
            var Yv = 0;
            var Zw = 33;
            var Kp = 0;
            var Mq = 0.8;
            var Nr = 0;

            // Added mass coeficience
            var X_udot = 6.36; // [kg]
            var Y_vdot = 7.12; // [kg]
            var Z_wdot = 18.68; // [kg]
            var K_pdot = 0.189; // [kg*m^2]
            var M_qdot = 0.135; // [kg*m^2]
            var N_rdot = 0.222; // [kg*m^2]
            
            var I_x = 0.26; // [kg*m^2], from OSBS's CAD
            var I_y = 0.23; // [kg*m^2], from OSBS's CAD
            var I_z = 0.37; // [kg*m^2], from OSBS's CAD

            // From eq 2 in OSBS
            // For M
            Matrix<double> I_c = DenseMatrix.OfDiagonalArray(new double[] {I_x, I_y, I_z}); // TODO: check how we actually create diagonal matrices 
            
            Matrix<double> M_RB = DenseMatrix.OfDiagonalArray(new double[] {m, m, m, I_x, I_y, I_z});
            Matrix<double> M_A = -DenseMatrix.OfDiagonalArray(new double[] {X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot});
            Matrix<double> M = M_RB + M_A;

            // v_dot see TODO above

            // For C
            // Matrix<double> someMatrix = DenseMatrix.OfColumnMajor(4, 4,  new double[] { 11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34, 41, 42, 43, 44 });
            Matrix<double> C_RB = DenseMatrix.OfArray(new double[,]
            {
                {0,     0,      0,      0,      m*w,     m*v    },
                {0,     0,      0,      -m*w,   0,       m*u    },
                {0,     0,      0,      m*v,    -m*u,    0      },
                {0,     m*w,    -m*v,   0,      -I_z*r, -I_y*q  },
                {-m*w,  0,      m*u,    I_z*r,  0,       I_x*p  },
                {m*v,   -m*u,   0,      I_y*q,  -I_x*p,  0      },
            });
            Matrix<double> C_A = DenseMatrix.OfArray(new double[,]
            {
                {0,         0,          0,          0,          -Z_wdot*w,  Y_vdot*v    },
                {0,         0,          0,          Z_wdot*w,   0,          -X_udot*u   },
                {0,         0,          0,          -Y_vdot*v,  X_udot*u,   0           },
                {0,         -Z_wdot*w,  Y_vdot*v,   0,          -N_rdot*r,  M_qdot*q    },
                {Z_wdot*w,  0,          -X_udot*u,  N_rdot*r,   0,          -K_pdot*p   },
                {-Y_vdot*v, X_udot*u,   0,          -M_qdot*q,  K_pdot*p,   0           }
            });
            Matrix<double> C = C_RB + C_A;

            // g_vec
            var rho = 1000; // [kg/m^3]
            var nabla = 0.0134; // [m^3], given experimental by OSBS
            B = rho*g*nabla; // The B given by OSBS
            Vector<double> g_vec = Vector<double>.Build.DenseOfArray(new double[] 
                {
                (W-B)*sin(theta),
                -(W-B)*cos(theta)*sin(phi),
                -(W-B)*cos(theta)*cos(phi),
                y_b*B*cos(theta)*sin(phi)-z_b*B*cos(theta)*sin(phi),
                -z_b*B*sin(theta)-x_b*B*cos(theta)*sin(phi),
                x_b*B*cos(theta)*sin(phi)+y_b*B*sin(theta)
                }
            );

            mainBody.inertiaTensor = I_o; //Can set inertia hera
            mainBody.mass = m;

            //Usually the control and damping forces need to be computed separately 
            Matrix<double> D = DenseMatrix.OfDiagonalArray(new double[] 
            {
                -Xu,
                -Yv, 
                -Zw,
                -Kp,
                -Mq, 
                -Nr
            });
            Matrix<double> Dn = DenseMatrix.OfDiagonalArray(new double[] 
            {
                -Xuu*abs(u),
                -Yvv*abs(v), 
                -Zww*abs(w),
                -Kpp*abs(p),
                -Mqq*abs(q), 
                -Nrr*abs(r)
            });
            Matrix<double> D_of_vel = D + Dn;

            // Do any "Once at startup" stuff here.
            
            
        }

        public void FixedUpdate()
        {
            // Inertia tensor
            Matrix<double> I_o = DenseMatrix.OfArray(new double[,]
            {
                { rigidBody.inertiaTensor.x, 0, 0 },
                { 0, rigidBody.inertiaTensor.z, 0 },
                { 0, 0, -rigidBody.inertiaTensor.y } //Here we use the inertia configured in Unity. Could replace if you want.
            });

            // # Hydrodynamic coefficients. Damping
            // # Mostly pressure drag.
            var m = 13.5; // [kg]. Previous: rigidBody.mass; 
            var W = m * 9.81; // In OSBS they use g = 9.82
            var B = W + vbs * 1.5;
            // Added from OSBS
            var Xuu = 141; // #1.0
            var Yvv = 217; // #100.0
            var Zww = 190; // #100.0
            var Kpp = 1.19; // #10.0
            var Mqq = 0.47; // #100.0
            var Nrr = 1.5; // #150.0
            
            var Xu = 13.7;
            var Yv = 0;
            var Zw = 33;
            var Kp = 0;
            var Mq = 0.8;
            var Nr = 0;

            // Added mass coeficience
            var X_udot = 6.36; // [kg]
            var Y_vdot = 7.12; // [kg]
            var Z_wdot = 18.68; // [kg]
            var K_pdot = 0.189; // [kg*m^2]
            var M_qdot = 0.135; // [kg*m^2]
            var N_rdot = 0.222; // [kg*m^2]
            
            var I_x = 0.26; // [kg*m^2], from OSBS's CAD
            var I_y = 0.23; // [kg*m^2], from OSBS's CAD
            var I_z = 0.37; // [kg*m^2], from OSBS's CAD

            // From eq 2 in OSBS
            // For M
            Matrix<double> I_c = DenseMatrix.OfDiagonalArray(new double[] {I_x, I_y, I_z}); // TODO: check how we actually create diagonal matrices 
            
            Matrix<double> M_RB = DenseMatrix.OfDiagonalArray(new double[] {m, m, m, I_x, I_y, I_z});
            Matrix<double> M_A = -DenseMatrix.OfDiagonalArray(new double[] {X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot});
            Matrix<double> M = M_RB + M_A;

            // v_dot see TODO above

            // For C
            // Matrix<double> someMatrix = DenseMatrix.OfColumnMajor(4, 4,  new double[] { 11, 12, 13, 14, 21, 22, 23, 24, 31, 32, 33, 34, 41, 42, 43, 44 });
            Matrix<double> C_RB = DenseMatrix.OfArray(new double[,]
            {
                {0,     0,      0,      0,      m*w,     m*v    },
                {0,     0,      0,      -m*w,   0,       m*u    },
                {0,     0,      0,      m*v,    -m*u,    0      },
                {0,     m*w,    -m*v,   0,      -I_z*r, -I_y*q  },
                {-m*w,  0,      m*u,    I_z*r,  0,       I_x*p  },
                {m*v,   -m*u,   0,      I_y*q,  -I_x*p,  0      },
            });
            Matrix<double> C_A = DenseMatrix.OfArray(new double[,]
            {
                {0,         0,          0,          0,          -Z_wdot*w,  Y_vdot*v    },
                {0,         0,          0,          Z_wdot*w,   0,          -X_udot*u   },
                {0,         0,          0,          -Y_vdot*v,  X_udot*u,   0           },
                {0,         -Z_wdot*w,  Y_vdot*v,   0,          -N_rdot*r,  M_qdot*q    },
                {Z_wdot*w,  0,          -X_udot*u,  N_rdot*r,   0,          -K_pdot*p   },
                {-Y_vdot*v, X_udot*u,   0,          -M_qdot*q,  K_pdot*p,   0           }
            });
            Matrix<double> C = C_RB + C_A;

            // g_vec
            var rho = 1000; // [kg/m^3]
            var nabla = 0.0134; // [m^3], given experimental by OSBS
            B = rho*g*nabla; // The B given by OSBS
            Vector<double> g_vec = Vector<double>.Build.DenseOfArray(new double[] 
                {
                (W-B)*sin(theta),
                -(W-B)*cos(theta)*sin(phi),
                -(W-B)*cos(theta)*cos(phi),
                y_b*B*cos(theta)*sin(phi)-z_b*B*cos(theta)*sin(phi),
                -z_b*B*sin(theta)-x_b*B*cos(theta)*sin(phi),
                x_b*B*cos(theta)*sin(phi)+y_b*B*sin(theta)
                }
            );

            mainBody.inertiaTensor = I_o; //Can set inertia hera
            mainBody.mass = m;

            //Usually the control and damping forces need to be computed separately 
            Matrix<double> D = DenseMatrix.OfDiagonalArray(new double[] 
            {
                -Xu,
                -Yv, 
                -Zw,
                -Kp,
                -Mq, 
                -Nr
            });
            Matrix<double> Dn = DenseMatrix.OfDiagonalArray(new double[] 
            {
                -Xuu*abs(u),
                -Yvv*abs(v), 
                -Zww*abs(w),
                -Kpp*abs(p),
                -Mqq*abs(q), 
                -Nrr*abs(r)
            });
            Matrix<double> D_of_vel = D + Dn;

            var mainBodyVelocity = mainBody.velocity;
            var mainBodyAngularVelocity = mainBody.angularVelocity;
            var mainBodyMass = mainBody.mass;


            var inverseTransformDirection_local = transform.InverseTransformDirection(mainBody.velocity);
            var transformAngularVelocity_local = transform.InverseTransformDirection(mainBody.angularVelocity);

            //An additional transform. From Unity RUF to a more appropriate frame of reference.
            var velocity_CorrectCoordinateFrame =
                inverseTransformDirection_local.To<NED>()
                    .ToDense(); // Might need to revisit. Rel. velocity in point m block.
            var angularVelocity_CorrectCoordinateFrame =
                FRD.ConvertAngularVelocityFromRUF(transformAngularVelocity_local).ToDense();

            // Do calculations here
            var tau_sum_control = M*v_dot + C*v + g_vec; // TODO: fix matrix vector multiplication

            // 3 first elements of tau_sum is force control
            // 3 last elements of tau_sum is torque control
            var resultingForce  = tau_sum.SubVector(0, 3).ToVector3();
            var resultingTorque = tau_sum.SubVector(4, 6).ToVector3();

            // Dampning forces
            var tau_sum_dampining = D*v;
            var force_damping = tau_sum_dampining.SubVector(0, 3).ToVector3(); //Vector3.zero; //These will be replaced with your model output
            var torque_damping = tau_sum_dampining.SubVector(4, 6).ToVector3();

            // Resulting force and torque vectors

            //var resultingForce = Vector3.zero;
            //var resultingTorque = Vector3.zero;

            mainBody.AddRelativeForce(force_damping);
            mainBody.AddRelativeTorque(torque_damping);
            mainBody.AddRelativeForce(resultingForce);
            mainBody.AddRelativeTorque(resultingTorque);

            // Set RPMs for Visuals
            propeller_front_left_top.SetDriveTargetVelocity(ArticulationDriveAxis.X, 0);
            propeller_front_right_top.SetDriveTargetVelocity(ArticulationDriveAxis.X, 0);
            propeller_back_left_top.SetDriveTargetVelocity(ArticulationDriveAxis.X, 0);
            propeller_back_right_top.SetDriveTargetVelocity(ArticulationDriveAxis.X, 0);

            propeller_front_left_bottom.SetDriveTargetVelocity(ArticulationDriveAxis.Z, 0);
            propeller_front_right_bottom.SetDriveTargetVelocity(ArticulationDriveAxis.Z, 0);
            propeller_back_left_bottom.SetDriveTargetVelocity(ArticulationDriveAxis.Z, 0);
            propeller_back_right_bottom.SetDriveTargetVelocity(ArticulationDriveAxis.Z, 0);
        }
    }
}