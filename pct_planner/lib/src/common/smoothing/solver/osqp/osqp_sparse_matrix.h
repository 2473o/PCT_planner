#pragma once

// #include <glog/logging.h>
#include <osqp/osqp.h>
#include <cstdlib>

#include <Eigen/Sparse>

namespace common {

template <typename Derived>
bool CreateOsqpSparseMatrix(
    const Eigen::SparseCompressedBase<Derived>& eigenSparseMatrix,
    OSQPCscMatrix*& osqpSparseMatrix) {
  // Copying into a new sparse matrix to be sure to use a CSC matrix
  Eigen::SparseMatrix<typename Derived::value_type, Eigen::ColMajor>
      colMajorCopy;

  // This may perform merory allocation, but this is already the case for
  // allocating the osqpSparseMatrix
  colMajorCopy = eigenSparseMatrix;

  // get number of row, columns and nonZeros from Eigen SparseMatrix
  OSQPInt rows = colMajorCopy.rows();
  OSQPInt cols = colMajorCopy.cols();
  OSQPInt numberOfNonZeroCoeff = colMajorCopy.nonZeros();

  // get innerr and outer index
  const int* outerIndexPtr = colMajorCopy.outerIndexPtr();
  const int* innerNonZerosPtr = colMajorCopy.innerNonZeroPtr();

  // instantiate csc matrix
  // MEMORY ALLOCATION!!
  if (osqpSparseMatrix != nullptr) {
    // LOG(ERROR) << "[OsqpEigen::SparseMatrixHelper::createOsqpSparseMatrix] "
    //               "osqpSparseMatrix pointer is not a null pointer! ";
    return false;
  }

  osqpSparseMatrix = (OSQPCscMatrix*)malloc(sizeof(OSQPCscMatrix));
  if (!osqpSparseMatrix) return false;
  
  osqpSparseMatrix->m = rows;
  osqpSparseMatrix->n = cols;
  osqpSparseMatrix->nz = -1;
  osqpSparseMatrix->nzmax = numberOfNonZeroCoeff;
  osqpSparseMatrix->p = (OSQPInt*)malloc((cols + 1) * sizeof(OSQPInt));
  osqpSparseMatrix->i = (OSQPInt*)malloc(numberOfNonZeroCoeff * sizeof(OSQPInt));
  osqpSparseMatrix->x = (OSQPFloat*)malloc(numberOfNonZeroCoeff * sizeof(OSQPFloat));

  if (!osqpSparseMatrix->p || !osqpSparseMatrix->i || !osqpSparseMatrix->x) {
    if (osqpSparseMatrix->p) free(osqpSparseMatrix->p);
    if (osqpSparseMatrix->i) free(osqpSparseMatrix->i);
    if (osqpSparseMatrix->x) free(osqpSparseMatrix->x);
    free(osqpSparseMatrix);
    osqpSparseMatrix = nullptr;
    return false;
  }

  int innerOsqpPosition = 0;
  for (int k = 0; k < cols; k++) {
    if (colMajorCopy.isCompressed()) {
      osqpSparseMatrix->p[k] = static_cast<OSQPInt>(outerIndexPtr[k]);
    } else {
      if (k == 0) {
        osqpSparseMatrix->p[k] = 0;
      } else {
        osqpSparseMatrix->p[k] =
            osqpSparseMatrix->p[k - 1] + innerNonZerosPtr[k - 1];
      }
    }
    for (typename Eigen::SparseMatrix<typename Derived::value_type,
                                      Eigen::ColMajor>::InnerIterator
             it(colMajorCopy, k);
         it; ++it) {
      osqpSparseMatrix->i[innerOsqpPosition] = static_cast<OSQPInt>(it.row());
      osqpSparseMatrix->x[innerOsqpPosition] = static_cast<OSQPFloat>(it.value());
      innerOsqpPosition++;
    }
  }
  osqpSparseMatrix->p[static_cast<int>(cols)] =
      static_cast<OSQPInt>(innerOsqpPosition);

  assert(innerOsqpPosition == numberOfNonZeroCoeff);

  return true;
}

template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N>& dense_matrix,
                      std::vector<T>* data, std::vector<D>* indices,
                      std::vector<D>* indptr) {
  static constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}
}  // namespace common