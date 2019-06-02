#pragma once

#include "Minimizer.h"

#include <random>
#include <iostream>
using namespace std;

class RandomMinimizer : public Minimizer
{
public:
	RandomMinimizer(const VectorXd &upperLimit = VectorXd(), const VectorXd &lowerLimit = VectorXd(), double fBest = HUGE_VAL)
		: searchDomainMax(upperLimit), searchDomainMin(lowerLimit), fBest(fBest) {
		fBest = HUGE_VAL;

		// initial random device and set uniform distribution to [0, 1]
		rng.seed(std::random_device()());
		dist = std::uniform_real_distribution<>(0.0,1.0);
	}

	virtual ~RandomMinimizer() {}

	bool minimize(const ObjectiveFunction *function, VectorXd &x, bool log) const override {
		for (int i = 0; i < iterations; ++i) {
			cout << "--- --- Begining of Evulation ---: " << endl;
			// for each element of `x`, generate a random variable in the search region
			VectorXd xr(x.size());
			for (int i = 0; i < x.size(); ++i) {
				xr[i] = dist(rng) * (searchDomainMax[i] - searchDomainMin[i]) + searchDomainMin[i];
			}

			// if function value at new `x` is smaller, let's keep it
			double f = function->evaluate(xr);
			cout << "--- This Randomization Result --- " << endl;

			cout << "fBest= " << fBest << endl;
			if(f < fBest){
				x = xr;
				fBest = f;
				cout << "fThis= " << f << " (New Best)" << endl;
				cout << "New Best Joint Angles:" << endl;
				for (int i = 0; i < x.size(); ++i) {
					cout << xr[i] << endl;
				}
			}
			else {
				cout << "fThis= " << f << endl;
			}
			cout << "--- --- End of Evulation ---: " << endl << endl;

		}
		return false;
	}

public:
	int iterations = 1;
	VectorXd searchDomainMax, searchDomainMin;

	mutable double fBest;
	mutable std::uniform_real_distribution<double> dist;
	mutable std::mt19937 rng;
};
