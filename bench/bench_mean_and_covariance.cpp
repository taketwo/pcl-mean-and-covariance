#include <celero/Celero.h>

#include <pcl/common/random.h>
#include <pcl/common/generate.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>

#include <mean_and_covariance.hpp>

#ifndef PRECISION
  #define PRECISION float
#endif

using Scalar = PRECISION;
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;
using Vector4 = Eigen::Matrix<Scalar, 4, 1>;

constexpr unsigned int CLOUD_SIZE = 640 * 480;
constexpr int SAMPLES = 20;
constexpr int ITERATIONS = 4;

template <typename PointT>
class MeanAndCovarianceFixture : public celero::TestFixture
{
public:
  MeanAndCovarianceFixture ()
  {
    using namespace pcl::common;

    cloud_in.reset (new pcl::PointCloud<PointT>);
    UniformGenerator<float>::Parameters params (-10.0f, 10.f, 12345);
    CloudGenerator<PointT, UniformGenerator<float>> generator (params);
    generator.fill (CLOUD_SIZE, 1, *cloud_in);

    pcl::search::KdTree<PointT> tree;
    tree.setInputCloud (cloud_in);

    int k = 10;
    std::vector<float> k_sqr_distances (k);
    neighborhoods.resize (cloud_in->size ());
    for (size_t i = 0; i < cloud_in->size (); ++i)
    {
      neighborhoods[i].resize (k);
      tree.nearestKSearch (cloud_in->at (i), k, neighborhoods[i], k_sqr_distances);
    }
  }

  typename pcl::PointCloud<PointT>::Ptr cloud_in;
  std::vector<std::vector<int>> neighborhoods;
};

CELERO_MAIN

#define COMPARE(Name, Type, Function, CloudIn) \
  BASELINE_F (Name, PCL, MeanAndCovarianceFixture<pcl::Type>, SAMPLES, ITERATIONS) \
  { \
    Matrix3 covariance; \
    Vector4 centroid; \
    for (size_t i = 0; i < CloudIn->size (); ++i) \
    { \
      pcl::Function <pcl::Type, Scalar> (*CloudIn, neighborhoods[i], covariance, centroid); \
    } \
  } \
  BENCHMARK_F (Name, Proposed, MeanAndCovarianceFixture<pcl::Type>, SAMPLES, ITERATIONS) \
  { \
    Matrix3 covariance; \
    Vector4 centroid; \
    for (size_t i = 0; i < CloudIn->size (); ++i) \
    { \
      EIGEN_ASM_COMMENT(#Name "::begin"); \
      pcl::experimental::Function <pcl::Type, Scalar> (*CloudIn, neighborhoods[i], covariance, centroid); \
      EIGEN_ASM_COMMENT(#Name "::end"); \
    } \
  }

COMPARE(XYZ, PointXYZ, computeMeanAndCovarianceMatrix, cloud_in)
COMPARE(XYZN, PointNormal, computeMeanAndCovarianceMatrix, cloud_in)
