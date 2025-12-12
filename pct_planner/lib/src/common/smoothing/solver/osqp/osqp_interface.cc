#include "common/smoothing/solver/osqp/osqp_interface.h"

#include <iostream>
#include <cstdlib>
#include <vector>

#include "common/smoothing/solver/osqp/osqp_sparse_matrix.h"

namespace common {

// Helper to free OSQPCscMatrix allocated by CreateOsqpSparseMatrix
void FreeOsqpSparseMatrix(OSQPCscMatrix* mat) {
  if (mat) {
    if (mat->p) free(mat->p);
    if (mat->i) free(mat->i);
    if (mat->x) free(mat->x);
    free(mat);
  }
}

bool OsqpInterface::Solve(
    const ColSparseMatrix& P,
    Eigen::Ref<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> q,
    const ColSparseMatrix& A,
    Eigen::Ref<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> l,
    Eigen::Ref<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> u,
    Eigen::VectorXd* x) {
  OSQPSettings* settings =
      static_cast<OSQPSettings*>(malloc(sizeof(OSQPSettings)));
  if (settings) osqp_set_default_settings(settings);
  
  settings->alpha = 1.0;
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polishing = 1;
  settings->verbose = 0;

  OSQPCscMatrix* P_osqp = nullptr;
  OSQPCscMatrix* A_osqp = nullptr;

  ColSparseMatrix P_uppper_triangular = P.triangularView<Eigen::Upper>();
  if (!CreateOsqpSparseMatrix(P_uppper_triangular, P_osqp)) {
    free(settings);
    return false;
  }

  if (!CreateOsqpSparseMatrix(A, A_osqp)) {
    FreeOsqpSparseMatrix(P_osqp);
    free(settings);
    return false;
  }

  OSQPSolver* solver = nullptr;
  OSQPInt m = A.rows();
  OSQPInt n = P.rows();
  
  OSQPInt exitflag = osqp_setup(&solver, P_osqp, q.data(), A_osqp, l.data(), u.data(), m, n, settings);

  if (exitflag != 0) {
      FreeOsqpSparseMatrix(P_osqp);
      FreeOsqpSparseMatrix(A_osqp);
      free(settings);
      // osqp_cleanup(solver); // Solver might be null if setup failed
      return false;
  }

  osqp_solve(solver);
  
  bool success = false;
  if (solver->info->status_val == 1 || solver->info->status_val == 2) { // Solved
    if (solver->solution && solver->solution->x) {
       OSQPFloat* solution = solver->solution->x;
       if (x->size() != n) x->resize(n);
       for (int i=0; i<n; ++i) {
           (*x)[i] = static_cast<double>(solution[i]);
       }
       success = true;
    }
  }

  // Cleanup
  osqp_cleanup(solver);
  FreeOsqpSparseMatrix(P_osqp);
  FreeOsqpSparseMatrix(A_osqp);
  free(settings);

  return success;
}

bool OsqpInterface::Solve(
    const Eigen::MatrixXd& P,
    Eigen::Ref<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> q,
    const Eigen::MatrixXd& A,
    Eigen::Ref<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> l,
    Eigen::Ref<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> u,
    Eigen::VectorXd* x) {
  std::vector<OSQPFloat> P_data;
  std::vector<OSQPInt> P_indices;
  std::vector<OSQPInt> P_indptr;
  const Eigen::MatrixXd& P_upper = P.triangularView<Eigen::Upper>();
  DenseToCSCMatrix(P_upper, &P_data, &P_indices, &P_indptr);

  std::vector<OSQPFloat> A_data;
  std::vector<OSQPInt> A_indices;
  std::vector<OSQPInt> A_indptr;
  DenseToCSCMatrix(A, &A_data, &A_indices, &A_indptr);

  OSQPSettings* settings =
      static_cast<OSQPSettings*>(malloc(sizeof(OSQPSettings)));
  if (settings) osqp_set_default_settings(settings);
  
  settings->alpha = 1.0;
  settings->eps_abs = 1.0e-05;
  settings->eps_rel = 1.0e-05;
  settings->max_iter = 5000;
  settings->polishing = 1;
  settings->verbose = 0;

  OSQPInt m = A.rows();
  OSQPInt n = P.rows();

  OSQPCscMatrix P_osqp;
  P_osqp.m = n;
  P_osqp.n = n;
  P_osqp.nz = -1;
  P_osqp.nzmax = P_data.size();
  P_osqp.x = P_data.data();
  P_osqp.i = P_indices.data();
  P_osqp.p = P_indptr.data();

  OSQPCscMatrix A_osqp;
  A_osqp.m = m;
  A_osqp.n = n;
  A_osqp.nz = -1;
  A_osqp.nzmax = A_data.size();
  A_osqp.x = A_data.data();
  A_osqp.i = A_indices.data();
  A_osqp.p = A_indptr.data();

  OSQPSolver* solver = nullptr;
  OSQPInt exitflag = osqp_setup(&solver, &P_osqp, q.data(), &A_osqp, l.data(), u.data(), m, n, settings);

  if (exitflag != 0) {
      free(settings);
      return false;
  }

  osqp_solve(solver);
  
  bool success = false;
  if (solver->info->status_val == 1 || solver->info->status_val == 2) {
      if (solver->solution && solver->solution->x) {
          OSQPFloat* solution = solver->solution->x;
          if (x->size() != n) x->resize(n);
          for (int i=0; i<n; ++i) {
              (*x)[i] = static_cast<double>(solution[i]);
          }
          success = true;
      }
  }

  osqp_cleanup(solver);
  free(settings);

  return success;
}

}  // namespace common
