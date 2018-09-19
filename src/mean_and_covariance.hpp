#pragma once

#include <pcl/point_types.h>

namespace pcl
{

namespace experimental
{

/* Mathematics for a single pass covariance calculation from a set of XYZ points.
 * If the covariance matrix is structured like so:
 *
 *    | 0 1 2 |
 *    | 3 4 5 |
 *    | 6 7 8 |
 *
 * Then:
 *
 *    0 = mean(X*X) - sum(X) * sum(X)
 *    1 = mean(X*Y) - sum(X) * sum(Y)
 *    2 = mean(X*Z) - sum(X) * sum(Z)
 *    3 = 1 = mean(X*Y) - sum(X) * sum(Y)
 *    4 = mean(Y*Y) - sum(Y) * sum(Y)
 *    5 = mean(Y*Z) - sum(Y) * sum(Z)
 *    6 = 2 = mean(X*Z) - sum(X) * sum(Z)
 *    7 = 5 = mean(Y*Z) - sum(Y) * sum(Z)
 *    8 = mean(Z*Z) - sum(Z) * sum(Z)
 */
template <typename PointT, typename Scalar> inline unsigned int
computeMeanAndCovarianceMatrix (const pcl::PointCloud<PointT> &cloud,
                                const std::vector<int> &indices,
                                Eigen::Matrix<Scalar, 3, 3> &covariance_matrix,
                                Eigen::Matrix<Scalar, 4, 1> &centroid)
{
  // create the buffer on the stack which is much faster than using cloud[indices[i]] and centroid as a buffer
  Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor> accu = Eigen::Matrix<Scalar, 1, 9, Eigen::RowMajor>::Zero ();
  size_t point_count;
  point_count = indices.size ();
  Eigen::Vector3f approx_centroid = cloud[indices[0]].getVector3fMap();

  // Accumulate a sum of the values we need (e.g. X*X) in 'accu'
  for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
  {
    // First subtract the centroid to center the point at the origin
    const Eigen::Vector3f point = cloud[*iIt].getVector3fMap() - approx_centroid;
    accu [0] += point[0] * point[0];    //  X*X
    accu [1] += point[0] * point[1];    //  X*Y
    accu [2] += point[0] * point[2];    //  X*Z
    accu [3] += point[1] * point[1];    //  Y*Y
    accu [4] += point[1] * point[2];    //  Y*Z
    accu [5] += point[2] * point[2];    //  Z*Z
    accu [6] += point[0];               //  X
    accu [7] += point[1];               //  Y
    accu [8] += point[2];               //  Z
  }
  accu /= static_cast<Scalar> (point_count);

  // Calculate the actual centroid (the 'mean' of the point cloud)
  centroid[0] = accu[6] + approx_centroid[0];
  centroid[1] = accu[7] + approx_centroid[1];
  centroid[2] = accu[8] + approx_centroid[2];
  centroid[3] = 1;

  // Now calculate the covariance matrix
  covariance_matrix.coeffRef (0) = accu [0] - accu [6] * accu [6];
  covariance_matrix.coeffRef (1) = accu [1] - accu [6] * accu [7];
  covariance_matrix.coeffRef (2) = accu [2] - accu [6] * accu [8];
  covariance_matrix.coeffRef (4) = accu [3] - accu [7] * accu [7];
  covariance_matrix.coeffRef (5) = accu [4] - accu [7] * accu [8];
  covariance_matrix.coeffRef (8) = accu [5] - accu [8] * accu [8];
  covariance_matrix.coeffRef (3) = covariance_matrix.coeff (1);
  covariance_matrix.coeffRef (6) = covariance_matrix.coeff (2);
  covariance_matrix.coeffRef (7) = covariance_matrix.coeff (5);

  return (static_cast<unsigned int> (point_count));
}


int add_two_numbers(int a, int b)
{
  return (a+b);
}


} // namespace experimental

} // namespace pcl

