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
  Eigen::Vector3f approx_centroid = cloud[indices[0]].getVector3fMap();
  point_count = indices.size ();
  for (std::vector<int>::const_iterator iIt = indices.begin (); iIt != indices.end (); ++iIt)
  {
    const Eigen::Vector3f point = cloud[*iIt].getVector3fMap() - approx_centroid;
    accu [0] += point[0] * point[0];
    accu [1] += point[0] * point[1];
    accu [2] += point[0] * point[2];
    accu [3] += point[1] * point[1];
    accu [4] += point[1] * point[2];
    accu [5] += point[2] * point[2];
    accu [6] += point[0];
    accu [7] += point[1];
    accu [8] += point[2];
  }

  accu /= static_cast<Scalar> (point_count);
  //Eigen::Vector3f vec = accu.tail<3> ();
  //centroid.head<3> () = vec;//= accu.tail<3> ();
  //centroid.head<3> () = accu.tail<3> ();    -- does not compile with Clang 3.0
  centroid[0] = accu[6] + approx_centroid[0];
  centroid[1] = accu[7] + approx_centroid[1];
  centroid[2] = accu[8] + approx_centroid[2];
  centroid[3] = 1;
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

