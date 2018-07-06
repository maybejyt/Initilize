#include "plane_constraint.h"
#include "../estimator.h"

PlaneFactor::PlaneFactor(IntegrationBase* _pre_integration,PlaneParams& planeparams) : pre_integration(_pre_integration),planeparams_(planeparams)
{

};

bool PlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
  
      Eigen::Vector3d Pi(parameters[0][1],parameters[0][1],parameters[0][2]);
      Sophus::SO3 SO3_v(parameters[0][3],parameters[0][4],parameters[0][5]);
      //Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
      Eigen::Vector3d residual;
      Eigen::Matrix3d tmp_R = SO3_v.Adj();
      Eigen::Quaterniond Qi(tmp_R);
      Eigen::Matrix3d i_R_g = Qi.toRotationMatrix();
      Eigen::Quaterniond tmp_q;
      Eigen::Vector3d tmp_p;
      tmp_q.x() = planeparams_.pi_q_g(0);
      tmp_q.y() = planeparams_.pi_q_g(1);
      tmp_q.z() = planeparams_.pi_q_g(2);
      tmp_q.w() = planeparams_.pi_q_g(3);
      Eigen::Quaterniond tmp_o_q;
      tmp_o_q.x() = planeparams_.o_q_i(0);
      tmp_o_q.y() = planeparams_.o_q_i(1);
      tmp_o_q.z() = planeparams_.o_q_i(2);
      tmp_o_q.w() = planeparams_.o_q_i(3);
      //ROS_INFO_STREAM("tmp_p: " << tmp_p );
      //ROS_INFO_STREAM("planeparams_.pi_z_g: " << planeparams_.pi_z_g );
      tmp_p = Pi - i_R_g.transpose()*planeparams_.o_R_i.transpose()*planeparams_.i_p_o;
      Eigen::Matrix3d pi_R_g = tmp_q.toRotationMatrix();
      //ROS_INFO_STREAM("pi_R_g: " << pi_R_g );
      residual.head(2).noalias() = planeparams_.span_e1_e2*planeparams_.o_R_i*i_R_g*pi_R_g.transpose()*planeparams_.e3;
      residual(2) = planeparams_.pi_z_g+planeparams_.e3.transpose()*pi_R_g*tmp_p;
      double tmp_b = planeparams_.e3.transpose()*pi_R_g*tmp_p;
      //ROS_INFO_STREAM("tmp_b:========== " << tmp_b );
      residuals[0] = residual(0);
      residuals[1] = residual(1);
      residuals[2] = residual(2);
      //ROS_INFO_STREAM("residuals: " << residual.transpose());
      Eigen::Vector3d temp_trans_cross_matrix = i_R_g.transpose()*planeparams_.o_R_i*planeparams_.i_p_o;
      Eigen::Vector3d temp_rot_cross_matrix   = pi_R_g.transpose()*planeparams_.e3;
      Eigen::Vector3d temp_odom_cross_matrix  = planeparams_.o_R_i*i_R_g*pi_R_g.transpose()*planeparams_.e3;
      Eigen::Matrix3d temp_a = crossproMatrix(temp_rot_cross_matrix);
      Eigen::Matrix3d temp_b = crossproMatrix(temp_trans_cross_matrix);
      Eigen::Matrix3d temp_c = crossproMatrix(temp_odom_cross_matrix);
      if(jacobians)
      {
	    if(jacobians[0])
	    {
	      
		Eigen::Map<Eigen::Matrix<double, 3, 6, Eigen::RowMajor>> jacobian_qi(jacobians[0]);
		jacobian_qi.setZero();
		jacobian_qi.block(0,0,2,3) = planeparams_.span_e1_e2*planeparams_.o_R_i*i_R_g*temp_a;
	        jacobian_qi.block(2,0,1,3).noalias() = planeparams_.e3.transpose()*pi_R_g*temp_b;
	    	jacobian_qi.block(2,3,1,3).noalias() = planeparams_.e3.transpose()*pi_R_g;
		
	    	
	    }
            	    
	    if(jacobians[1])
	    {
	        
		Eigen::Map<Eigen::Matrix<double, 3, 2, Eigen::RowMajor>> jacobian_pi(jacobians[1]);
		jacobian_pi.setZero();
		jacobian_pi.block(0,0,2,2).noalias() = planeparams_.span_e1_e2*planeparams_.o_R_i*i_R_g*pi_R_g.transpose()*planeparams_.span_plus_e1_e2;
		jacobian_pi.block(2,0,1,2).noalias() = tmp_p.transpose()*pi_R_g.transpose()*planeparams_.span_plus_e1_e2;
		
	    }
	    
	     if(jacobians[2])
	    {
	      
	        Eigen::Map<Eigen::Vector3d> jacobian_z(jacobians[2]);
		jacobian_z.setZero();
		jacobian_z(2) = 1.0;
		
	    }
	    
	    
	
      }
       
      return true;
      
}

void PlaneFactor::check(double **parameters)
{
 
}

Matrix3d PlaneFactor::crossproMatrix(Eigen::Vector3d a) const
{
     Matrix3d tmp;
     tmp<<  0, -a(2), a(1),
            a(2), 0, -a(0),
           -a(1) , a(0), 0;
     return tmp;
   
}
