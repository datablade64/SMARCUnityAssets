using System;
using DefaultNamespace.LookUpTable;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
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

        public double rpm_front_left_top = 0.0f;
        public double rpm_front_right_top = 0.0f;
        public double rpm_back_left_top = 0.0f;
        public double rpm_back_right_top = 0.0f;
        public double rpm_front_left_bottom = 0.0f;
        public double rpm_front_right_bottom = 0.0f;
        public double rpm_back_left_bottom = 0.0f;
        public double rpm_back_right_bottom = 0.0f;
        public double vbs = 0.0f;
        
        // Hydrodynamic coefficients. Damping
        // Mostly pressure drag.
        // Initalized in start.
        private double m = 0;
        private double W = 0;
        private double B = 0;
        
        //Added from OSBS
        double Xuu = 141; // #1.0
        double Yvv = 217; // #100.0
        double Zww = 190; // #100.0
        double Kpp = 1.19; // #10.0
        double Mqq = 0.47; // #100.0
        double Nrr = 1.5; // #150.0
        
        double Xu = 13.7;
        double Yv = 0;
        double Zw = 33;
        double Kp = 0;
        double Mq = 0.8;
        double Nr = 0;
        // Added mass coeficience
        double X_udot = 6.36; // [kg]
        double Y_vdot = 7.12; // [kg]
        double Z_wdot = 18.68; // [kg]
        double K_pdot = 0.189; // [kg*m^2]
        double M_qdot = 0.135; // [kg*m^2]
        double N_rdot = 0.222; // [kg*m^2]

        public Camera myCamera;
        public Vector3 camera_offset;
        double I_x = 0.26; // [kg*m^2], from OSBS's CAD
        double I_y = 0.23; // [kg*m^2], from OSBS's CAD
        double I_z = 0.37; // [kg*m^2], from OSBS's CAD
        private Vector<double> vel_vec_prev = Vector<double>.Build.DenseOfArray(new double[] { 0, 0, 0 ,0, 0, 0 });

        public void Start()
        {
            myCamera =  Camera.main;
            camera_offset = new Vector3(0f, 0.5f, -2f);
            // mass 11
            m = mainBody.mass;
            W = m * -9.81; // In OSBS they use g = 9.82
            // TODO: set inertias to main body. kanske också
        }
        
        public void FixedUpdate()
        {
            
            B = W + vbs * 1.5;
            var world_pos = mainBody.transform.position; // Needs to be verified
            var world_rot = mainBody.transform.rotation.eulerAngles; 
            var inverseTransformDirection = mainBody.transform.InverseTransformDirection(mainBody.velocity); // Local frame vel
            var transformAngularVelocity = mainBody.transform.InverseTransformDirection(mainBody.angularVelocity); // Local frame angular vel
            
            // To NED coordinates
            var xyz = world_pos.To<NED>().ToDense();
            
            var x_b = xyz[0];
            var y_b = xyz[1];
            var z_b = xyz[2];
            
            var phiThetaTau = FRD.ConvertAngularVelocityFromRUF(world_rot).ToDense();
            float phi = (float) phiThetaTau[0]; 
            float theta = (float) phiThetaTau[1];
            
            var uvw = inverseTransformDirection.To<NED>().ToDense(); // Might need to revisit. Rel. velocity in point m block.
            float u = (float) uvw[0];
            float v = (float) uvw[1];
            float w = (float) uvw[2];
            
            var pqr = FRD.ConvertAngularVelocityFromRUF(transformAngularVelocity).ToDense(); // FRD is same as NED for ANGLES ONLY
            float p = (float) pqr[0];
            float q = (float) pqr[1];
            float r = (float) pqr[2];
        
            Vector<double> vel_vec = Vector<double>.Build.DenseOfArray(new double[] { u, v, w, p, q, r });
             
            /* Getting inertia tensor from OSBS CAD instead of getting it from Unity
            // Inertia tensor
            Matrix<double> I_o = DenseMatrix.OfArray(new double[,]
            {
                { mainBody.inertiaTensor.x, 0, 0 },
                { 0, mainBody.inertiaTensor.z, 0 },
                { 0, 0, -mainBody.inertiaTensor.y } //Here we use the inertia configured in Unity. Could replace if you want.
            });
            */
            // From eq 2 in OSBS
            // For M
            //Vector<double> I_vec = Vector<double>.Build.DenseOfArray(new double [] {I_x, I_y, I_z}); // TODO: check how we actually create diagonal matrices 
            Vector3 I_vec = new Vector3((float) I_x, (float) I_y, (float) I_z);
            Matrix<double> I_c = DenseMatrix.OfDiagonalArray(new double[] {I_x, I_y, I_z}); // TODO: check how we actually create diagonal matrices 
            
            // TODO: scrap?
            Matrix<double> M_RB = DenseMatrix.OfDiagonalArray(new double[] {m, m, m, I_x, I_y, I_z});
            Matrix<double> M_A = -DenseMatrix.OfDiagonalArray(new double[] {X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot});
            Matrix<double> M = M_RB + M_A;
            
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
            B = rho*-9.81*nabla; // The B given by OSBS
            Vector<double> g_vec = Vector<double>.Build.DenseOfArray(new double[] 
                {
                (W-B)*Mathf.Sin(theta),
                -(W-B)*Mathf.Cos(theta)*Mathf.Sin(phi),
                -(W-B)*Mathf.Cos(theta)*Mathf.Cos(phi),
                y_b*B*Mathf.Cos(theta)*Mathf.Sin(phi)-z_b*B*Mathf.Cos(theta)*Mathf.Sin(phi),
                -z_b*B*Mathf.Sin(theta)-x_b*B*Mathf.Cos(theta)*Mathf.Sin(phi),
                x_b*B*Mathf.Cos(theta)*Mathf.Sin(phi)+y_b*B*Mathf.Sin(theta)
                }
            );

            mainBody.inertiaTensor = I_vec;  
            //mainBody.mass = (float) m;

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
                -Xuu*Mathf.Abs(u),
                -Yvv*Mathf.Abs(v), 
                -Zww*Mathf.Abs(w),
                -Kpp*Mathf.Abs(p),
                -Mqq*Mathf.Abs(q), 
                -Nrr*Mathf.Abs(r)
            });
            Matrix<double> D_of_vel = D + Dn;

            //var mainBodyVelocity = mainBody.velocity;
            //var mainBodyAngularVelocity = mainBody.angularVelocity;
            //var mainBodyMass = mainBody.mass;


            var inverseTransformDirection_local = transform.InverseTransformDirection(mainBody.velocity);
            var transformAngularVelocity_local = transform.InverseTransformDirection(mainBody.angularVelocity);

            //An additional transform. From Unity RUF to a more appropriate frame of reference.
            var velocity_CorrectCoordinateFrame =
                inverseTransformDirection_local.To<NED>()
                    .ToDense(); // Might need to revisit. Rel. velocity in point m block.
            var angularVelocity_CorrectCoordinateFrame =
                FRD.ConvertAngularVelocityFromRUF(transformAngularVelocity_local).ToDense();

            var vel_vec_dot = (vel_vec-vel_vec_prev)/Time.fixedDeltaTime;
            // TODO: M*vel_vec_dot <- scrap, ersätt med input forces
            // TODO: resulting forces -> lateral forces + coriolis
            var tau_sum_coriolis =  C * vel_vec;
            vel_vec_prev = vel_vec;

            // Resulting force and torque vectors
            // 3 first elements of tau_sum is force control
            // 3 last elements of tau_sum is torque control
            var coriolisForce  = tau_sum_coriolis.SubVector(0, 3).ToVector3();
            var coriolisTorque = tau_sum_coriolis.SubVector(3, 3).ToVector3();
            var lateralForce  = g_vec.SubVector(0, 3).ToVector3();
            var lateralTorque = g_vec.SubVector(3, 3).ToVector3();

            // Dampning forces
            var v_c = 0; // Assume no ocean current. If desired to integrete it, info about it can be found in OSBS
            var vr = vel_vec - v_c;
            var tau_sum_dampining = D_of_vel*vr; // TODO: fix matrix multiplication
            var force_damping = tau_sum_dampining.SubVector(0, 3).ToVector3(); //Vector3.zero; //These will be replaced with your model output
            var torque_damping = tau_sum_dampining.SubVector(3, 3).ToVector3();
            
            // Back to RUF (Unity) coordinates)
            force_damping = NED.ConvertToRUF(force_damping);
            torque_damping = FRD.ConvertAngularVelocityFromRUF(torque_damping);
            
            coriolisForce = NED.ConvertToRUF(coriolisForce);
            coriolisTorque = FRD.ConvertAngularVelocityFromRUF(coriolisTorque);
            
            lateralForce = NED.ConvertToRUF(lateralForce);
            lateralTorque = FRD.ConvertAngularVelocityFromRUF(lateralTorque);
            
            // VVV UNCOMMENT FOR FOLLOWING CAMERA VVV
            myCamera.transform.position = camera_offset + world_pos;
           
            Vector3 inputForce = Vector3.zero;
            Vector3 inputTorque = Vector3.zero;
            if (Input.GetKey(KeyCode.W))
            {
                inputForce[2] = 85;
            }
            if (Input.GetKey(KeyCode.A))
            {
                inputForce[0] = -85;
            }
            if (Input.GetKey(KeyCode.S))
            {
                inputForce[2] = -85;
            }
            if (Input.GetKey(KeyCode.D))
            {
                inputForce[0] = 85;
            }
            if (Input.GetKey(KeyCode.Space))
            {
                inputForce[1] = 120;
            }
            if (Input.GetKey(KeyCode.LeftShift))
            {
                inputForce[1] = -120;
            }
            if (Input.GetKey(KeyCode.Q))
            {
                inputTorque[1] = -14;
            }
            if (Input.GetKey(KeyCode.E))
            {
                inputTorque[1] = 14;
            }

            // Vector3 com_pos = new Vector3(-0.17f, 0.11f, -0.13f);
            // mainBody.centerOfMass = com_pos;
            // Vector3 global_com_pos = com_pos + mainBody.transform.position;
            print("x" + vel_vec[0]);
            print("y" + vel_vec[1]);
            print("z" + vel_vec[2]);
            // print(force_damping);
            // print(torque_damping);
            // mainBody.AddForceAtPosition(force_damping,global_com_pos);
            mainBody.AddRelativeForce(force_damping);
            mainBody.AddRelativeTorque(torque_damping);
            mainBody.AddRelativeForce(coriolisForce);
            // mainBody.AddRelativeTorque(coriolisTorque);
            mainBody.AddRelativeForce(lateralForce);
            // mainBody.AddRelativeTorque(lateralTorque);
            mainBody.AddRelativeForce(inputForce);
            // mainBody.AddForceAtPosition(input_force_vector,global_com_pos);
            mainBody.AddRelativeTorque(inputTorque);


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