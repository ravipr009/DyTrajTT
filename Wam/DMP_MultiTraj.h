/*
 * DMP_first.h
 *
 *  Created on: 28-Mar-2015
 *      Author: nilxwam
 */

#ifndef DMP_H_
#define DMP_H_


//#include <time.h>
#include <fstream>
//#include <iostream>

#include <barrett/math/traits.h>
//#include <list>
//#include <barrett/units.h>
#include <barrett/detail/ca_macro.h>
#include <barrett/systems/abstract/system.h>
//#include <eigen3/Eigen/Core>
#include <libconfig.h++>

#include <barrett/detail/ca_macro.h>
#include <barrett/math/traits.h>
#include <barrett/systems/abstract/execution_manager.h>
#include <barrett/systems/abstract/controller.h>
//#include <barrett/products/product_manager.h>

using namespace std;
using namespace barrett;
using namespace systems;

#define SAMPLING_TIME_DMP 0.002

//
//
//#include <time.h>
//#include <fstream>
//#include <iostream>
//
//#include <barrett/math/traits.h>
//#include <list>
//#include <barrett/units.h>
//#include <barrett/detail/ca_macro.h>
//#include <barrett/systems/abstract/system.h>
//#include <eigen3/Eigen/Core>
//#include <libconfig.h++>
//
//#include <barrett/detail/ca_macro.h>
//#include <barrett/math/traits.h>
//#include <barrett/systems/abstract/execution_manager.h>
//#include <barrett/systems/abstract/controller.h>
//#include <barrett/products/product_manager.h>
//
//using namespace std;
//using namespace barrett;
//using namespace systems;
//


template<size_t DOF>
class DMP: public System
{
	/* Torque*/
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

public:
	uint8_t Dofs; // number of DMP , individual trajectory we will follow
	uint8_t Dmps;
	uint8_t SelectedDmps;
	unsigned int Bfs; // Number of Radial basis function for non linear function
	float *Y0; // Initial state
	float *Y; // Calculated state
	float *Dy; // Calculated velocity
	float *DDy; // Calculated acceleration
	float **Goals; // Goal state
	double ***Weights; // Weights for function
	float ay; // PD controller constants
	float by; // PD controller constants
	float *tau;
	float *Centers; // Radial basis function centers
	float *Variance; // Radial basis function variance
	float *Psi; // Calculated Radial basis function value
	float force; // Non linear force to modify trajectory
	float CSx; // Canonical system value
	float CSax; // Canonical system time constant
	float dt;
	//timespec Time;
	//float PrevTime;
	bool IsGoal;
	bool RunRobot;

	jp_type ref_jp_tmp; // To move the hand
	jv_type ref_jv_tmp;

//public:
//	Input<jp_type> CurrentStatePos;
//	Input<jv_type> CurrentStateVel;

public:
	Output<jp_type> ref_jp; // To move the hand
	Output<jv_type> ref_jv;

	//Output<jt_type> controlOutput;
	//Output<jp_type> ref_jp;
	//Output<jp_type> ref_jv;
	//Output<jp_type> ref_ja;

	//void MakeMatrix(float ** &temp, unsigned rows, unsigned cols); // ONE  TIME

	void InitializeFromFile(const char *); // Set all parametes from config file

	void LoadWeights(); // ONE  TIME

	void InitializeSystem(const uint8_t, const unsigned short int, uint8_t, float, float, float, float, float, float*); // ONE  TIME

	// Set goal and initial value
	void SetInitialConditions(float*, float**); // ONE  TIME

	// Set Radial Basis Function Centers and Varience
	void GenerateGaussianCenters(float); // ONE  TIME

	// Calculate Radial Basis Function value
	float GenerateActivationFunction();
	// Run DMP system one time step
	void RunOneStep();

	void CheckGoalOffset(); // ONE  TIME

	void CheckGoal(); //

	// Reset DMP
	void ResetState();

	DMP(const unsigned short int dummyvariable);
	
	virtual ~DMP()
	{
		this->mandatoryCleanUp();
	}
	
protected:
	typename Output<jp_type>::Value* ref_jp_OutputValue;
	typename Output<jv_type>::Value* ref_jv_OutputValue;

	virtual void operate()
	{
		if(RunRobot)
		{
			if(!IsGoal)
			{
				RunOneStep();
				for(uint8_t i=0; i<Dofs; i++)
				{
					ref_jp_tmp[i] = Y[i];
					ref_jv_tmp[i] = Dy[i];
				}
				CheckGoal();
			}
			else
			{
				RunRobot = false;
				ResetState();
				CheckGoalOffset();
				IsGoal = false;
			}
		}
		ref_jp_OutputValue->setData(&ref_jp_tmp);
		ref_jv_OutputValue->setData(&ref_jv_tmp);
	}
};

#include <Detail/DMP_MultiTraj-inl.h>
#endif /* DMP_H_ */
