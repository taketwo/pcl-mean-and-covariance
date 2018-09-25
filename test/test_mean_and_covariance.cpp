#include <gtest/gtest.h>
#include <pcl/point_types.h>              // PointXYZ, PointXYZRGB etc
#include <pcl/point_cloud.h>
#include <mean_and_covariance.hpp>        // Includes all the functions we want to test

using namespace pcl;
using namespace pcl::experimental;


TEST(MeanAndCovariance, SelfTest)
{
  // Create a super-basic cloud
  PointCloud<PointXYZ> c;
  PointXYZ p1 (1.0f, 1.0f, 0.0f), p2 (1.0f, -1.0f, 0.0f),
           p3 (-1.0f, -1.0f, 0.0f), p4 (-1.0f, 1.0f, 0.0f);
  c.push_back(p1); c.push_back(p2); c.push_back(p3); c.push_back(p4);

  // Create the rest of our input parameters and our expected results
  const std::vector<int> indices = {0,1,2,3};
  Eigen::Vector4f centroid_sp_f, centroid_dp_f, centroid_expected;
  Eigen::Vector4d centroid_sp_d, centroid_dp_d;
  Eigen::Matrix3f covariance_matrix_sp_f, covariance_matrix_dp_f, covariance_matrix_expected;
  Eigen::Matrix3d covariance_matrix_sp_d, covariance_matrix_dp_d;
  centroid_expected = {0,0,0,1};
  covariance_matrix_expected << 1.0, 0.0, 0.0,
                                0.0, 1.0, 0.0,
                                0.0, 0.0, 0.0;

  // Input to our covariance matrix functions
  computeMeanAndCovarianceMatrix(c, indices, covariance_matrix_sp_f, centroid_sp_f);
  computeMeanAndCovarianceMatrix(c, indices, covariance_matrix_sp_d, centroid_sp_d);
  computeMeanAndCovarianceMatrixDoublePass(c, indices, covariance_matrix_dp_f,
                                           centroid_dp_f);
  computeMeanAndCovarianceMatrixDoublePass(c, indices, covariance_matrix_dp_d,
                                           centroid_dp_d);

  // Check all centroids
  for (int i = 0; i < 4; i++)
  {
    EXPECT_EQ(centroid_expected[i], centroid_sp_f[i]) \
              << "Single-pass, single precision centroid differs at index " << i;
    EXPECT_EQ(centroid_expected[i], centroid_dp_f[i]) \
              << "Double-pass, single precision centroid differs at index " << i;
    EXPECT_EQ(centroid_expected[i], centroid_sp_d[i]) \
              << "Single-pass, double precision centroid differs at index " << i;
    EXPECT_EQ(centroid_expected[i], centroid_dp_d[i]) \
              << "Double-pass, double precision centroid differs at index " << i;
  }

  // Check all covariance-matrices
  for (int col = 0; col < 3; col++)
  {
    for (int row = 0; row < 3; row++)
    {
      EXPECT_EQ(covariance_matrix_sp_f(row, col), covariance_matrix_expected(row, col)) \
                << "Single-pass, single precision covariance matrix differs at row "
                << row << ", col " << col;
      EXPECT_EQ(covariance_matrix_dp_f(row, col), covariance_matrix_expected(row, col)) \
                << "Double-pass, single precision covariance matrix differs at row "
                << row << ", col " << col;
      EXPECT_EQ(covariance_matrix_sp_d(row, col), covariance_matrix_expected(row, col)) \
                << "Single-pass, double precision covariance matrix differs at row "
                << row << ", col " << col;
      EXPECT_EQ(covariance_matrix_dp_d(row, col), covariance_matrix_expected(row, col)) \
                << "Double-pass, double precision covariance matrix differs at row "
                << row << ", col " << col;
    }
  }
}


int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
