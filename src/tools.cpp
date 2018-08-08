#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;
	VectorXd diff2;
	VectorXd diff;
	VectorXd mean(4);
	VectorXd sum2(4);
	sum2 << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	// ... your code here
	if (estimations.size() == 0)
	{
		std::cout << " Error: CalculateRMSE() - estimation vector is of 0 size " << std::endl;
		return rmse;
	}

	if (estimations.size() != ground_truth.size())
	{
		std::cout << " Error: CalculateRMSE() - estimation and ground_truth vectors are of different size " << std::endl;
		return rmse;
	}

	//accumulate squared residuals
	for (int i = 0; i < (int)estimations.size(); ++i)
	{
		diff = estimations[i] - ground_truth[i];
		diff2 = diff.array() * diff.array();
		sum2 = sum2 + diff2;
	}

	//calculate the mean
	mean = sum2 * (1.0 / estimations.size());

	//calculate the squared root
	rmse = mean.array().sqrt();

	//return the result
	return rmse;
}