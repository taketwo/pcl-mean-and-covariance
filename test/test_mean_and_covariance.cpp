#include <gtest/gtest.h>
#include <mean_and_covariance.hpp>

TEST(TestCaseName, TestCase)
{
  int a = 1;
  int b = 2;
  int c = pcl::experimental::add_two_numbers(a,b);
  EXPECT_EQ(c, 3);
}


int main (int argc, char** argv)
{
  testing::InitGoogleTest (&argc, argv);
  return (RUN_ALL_TESTS ());
}
