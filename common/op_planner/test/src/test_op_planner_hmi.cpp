#include <ros/ros.h>
#include <gtest/gtest.h>

#include "op_planner/hmi/HMIMSG.h"

class TestFix : public ::testing::Test
{
public:
	PlannerHNS::HMI_MSG msg;
	TestFix() {
	}
	~TestFix() {

	}
};

TEST_F(TestFix, TestEmptyMessage)
{
	PlannerHNS::HMI_MSG n_msg =  PlannerHNS::HMI_MSG::FromString("");
	EXPECT_TRUE(n_msg.type == PlannerHNS::UNKNOWN_MSG);
}

TEST_F(TestFix, TestConfirmation)
{
	PlannerHNS::HMI_MSG n_msg =  PlannerHNS::HMI_MSG::FromString("1,1,,0,0,0,,0,,,");
	EXPECT_FALSE(n_msg.type == PlannerHNS::CONFIRM_MSG);
}


int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
