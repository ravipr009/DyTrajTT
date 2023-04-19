/*
 * DMP_first-inl.hpp
 *
 *  Created on: 28-Mar-2015
 *      Author: nilxwam
 */

#ifndef DMP_INL_H_
#define DMP_INL_H_
/*
template<size_t DOF>
void DMP<DOF>::MakeMatrix(float ** &temp, unsigned rows, unsigned cols) {
	temp = new float*[rows];

	for (unsigned int i = 0; i < rows; ++i)
		temp[i] = new float[cols];
}
*/


template<size_t DOF>
void DMP<DOF>::LoadWeights() 
{
	std::string line;
	//std::ifstream file( "w.csv" );
	std::ifstream file;
	char filenamelist[]="w0_nor.csv";
	uint8_t i, j;
	unsigned short int k;

	for(i=0; i<Dmps; i++)
	{
		filenamelist[1]=(char)(i+48);
		std::cout<<"       "<<filenamelist<<std::endl;
		file.open(filenamelist);
		if(file.good())
		{
			for(j=0; j<Dofs; j++)
			{
				for(k=0; k<Bfs; k++)
				{
					getline ( file, line, ',' );
					Weights[i][j][k] = atof(line.c_str());
					//std::cout<<Weights[i][j][k]<<"  "<<i+1<<","<<j+1<<","<< k+1<<" "<<std::endl;
					//std::cin >> temp;
				}
				//std::cout<<std::endl<<std::endl;
				//std::cin >> temp;
			}
			std::cout<<" 2.1 | Weights - "<<int(i)<<","<<int(j)<<std::endl;
		}
#ifdef print_error
		else
		{
			std::cout<<"Cannot find "<<filename<<std::endl;
			exit(0);
		}
#endif
		file.close();
	}

	//for (unsigned int i=0; i<Dofs; i++)
	//	for(unsigned int j=0;j<Bfs;j++)
		//	Weights[i][j] = data[i][j]; 
}

template<size_t DOF>
void DMP<DOF>::InitializeFromFile(const char *filename)
{
	uint8_t dofs = 0, i, j, dmps;
	unsigned short int bfs = 0;
	float a = 0;
	float b = 0;
	float runtime = 0;
	float tolerance = 0;
	float samplingtime = 0;
	float *tau;
	float *y0;
	float **goal;
	std::ifstream inputfile(filename);
	std::string line;

	if(inputfile.good())
	{
		getline (inputfile, line, ',');
		dofs = atoi(line.c_str());
		std::cout<<" 2.1 | dof "<<int(dofs)<<std::endl;

		getline (inputfile, line, ',');
		bfs = atoi(line.c_str());
		std::cout<<" 2.1 | bfs "<<bfs<<std::endl;

		getline (inputfile, line, ',');
		dmps = atoi(line.c_str());
		std::cout<<" 2.1 | dmps "<<int(dmps)<<std::endl;

		getline (inputfile, line, ',');
		a = atof(line.c_str());
		std::cout<<" 2.1 | a "<<a<<std::endl;

		b = a/4;
		std::cout<<" 2.1 | b "<<b<<std::endl;

		getline (inputfile, line, ',');
		runtime = atof(line.c_str());
		std::cout<<" 2.1 | runtime "<<runtime<<std::endl;

		getline (inputfile, line, ',');
		tolerance = atof(line.c_str());
		std::cout<<" 2.1 | tolerance "<<tolerance<<std::endl;

		getline (inputfile, line, ',');
		samplingtime = atof(line.c_str());
		std::cout<<" 2.1 | samplingtime "<<samplingtime<<std::endl;

		tau = new float [dmps];
		std::cout<<" 2.1 | tau - ";
		for(i=0; i<dmps; i++)
		{
			getline (inputfile, line, ',');
			tau[i] = atof(line.c_str());
			std::cout<<tau[i]<<", ";
		}
		std::cout<<std::endl;

		std::cout<<" 2.1 | y0 - ";
		y0 = new float [dofs];		
		for(i=0; i<dofs; i++)
		{
			getline (inputfile, line, ',');
			y0[i] = atof(line.c_str());
			std::cout<<y0[i]<<", ";
		}
		std::cout<<std::endl;

		goal = new float* [dmps];
		for(i=0; i<dmps; i++)
		{
			std::cout<<" 2.1 | Goal "<<int(i)<<" - ";
			goal[i] = new float[dofs];
			for(j=0; j<dofs; j++)
			{
				getline (inputfile, line, ',');
				goal[i][j] = atof(line.c_str());
				std::cout<<goal[i][j]<<", ";
			}
			std::cout<<std::endl;
		}

		InitializeSystem(dofs, bfs, dmps, a, b, runtime, tolerance, samplingtime, tau);
		SetInitialConditions(y0, goal);
	}
	#ifdef print_error
	else
	{
		std::cout<<" 2.1 | Cannot find "<<filename<<std::endl;
		exit(0);
	}
	#endif
}

// Set goal and initial value
template<size_t DOF>
void DMP<DOF>::SetInitialConditions(float* y0, float** goal) 
{
	uint8_t i, j;
	for(i=0; i<Dofs; i++)
	{
		Y0[i] = y0[i];
		Y[i] = y0[i];
		for(j=0; j<Dmps; j++)
			Goals[j][i] = goal[j][i];
	}
}

// Set Radial Basis Function Centers and Varience
template<size_t DOF>
void DMP<DOF>::GenerateGaussianCenters(float runtime) 
{
	unsigned int i;
	float *des_c = new float[Bfs];
	des_c[0] = exp(-CSax * runtime);
	des_c[Bfs - 1] = 1.05 - des_c[0];
	float tempdiff = (des_c[Bfs - 1] - des_c[0]) / (Bfs - 1);

	for (i = 1; i < Bfs; i++)
		des_c[i] = des_c[i - 1] + tempdiff;
	
	for (i = 0; i < Bfs; i++) 
	{
		// x = exp(-c), solving for c
		// Centers are distributed evenly in time axis
		// Variance accordint to Sachaal 2012 paper
		Centers[i] = -std::log(des_c[i]); // Set centeres
		Variance[i] = (pow(Bfs, 1.5)) / (Centers[i] + 0.001); // Set variance
	}
}

// Calculate Radial Basis Function value
template<size_t DOF>
float DMP<DOF>::GenerateActivationFunction() 
{
	float sum = 0.0;
	for (unsigned int i = 0; i < Bfs; i++)
	{
		Psi[i] = exp(-Variance[i] * pow((CSx - Centers[i]), 2)); // Calculate Radial Basis Function value
		sum = sum + Psi[i];
	}
	return sum;
}

template<size_t DOF>
void DMP<DOF>::InitializeSystem(const uint8_t dofs, const unsigned short int bfs, uint8_t dmps, float a, float b, float runtime, float tolerance, float sampling_time, float* time_scaling)
{
	uint8_t i, k;
	unsigned short int j;

	dt = sampling_time;
	IsGoal = false;
	RunRobot = false;
	SelectedDmps = 0;
	Dofs = dofs; // Set number of DMPs
	Dmps = dmps; // Set number of DMPs
	Bfs = bfs;
	Y0 = new float [dofs]; // Set initial value
	Y = new float [dofs]; // Set initial value
	Dy = new float [dofs]; // Set initial value
	DDy = new float [dofs]; // Set initial value
	//Goal = new float [Dmps]; // Set goal value
	Centers = new float [Bfs]; // Allocate memory for Radial Basis Function centers
	Variance = new float [Bfs]; // Allocate memory for Radial Basis Function variance
	Psi = new float [Bfs];  // Allocate memory for Radial Basis Function output
	tau = new float [dmps];

	//MakeMatrix(Weights,Dmps,Bfs); // Allocate memory for weights
	//ay = new float [Dmps]; // Allocate memory for PD controller parameters
	//by = new float [Dmps]; // Allocate memory for PD controller parameters

    	CSax = -std::log(tolerance)/runtime; // Time constant such that trajectory goes to 95% or (1-tolerance) in runtime
    	//CSax = -log(0.05)/; // Sachaal 2012 paper values to verify code
	CSx = 1.0; // Canonicalsystem initialize
	force = 0.0;

	ay = a; // Set value
	by = b; // Set value

	Goals = new float* [dmps];
	for(i=0; i<dmps; i++)
		Goals[i] = new float [dofs];

	Weights = new double** [dmps];
	for(i=0; i<dmps; i++)
	{
		Weights[i] = new double* [dofs];
		for(j=0; j<dofs; j++)
			Weights[i][j] = new double [bfs];
	}

	for(i=0; i<dmps; i++)
		tau[i] = time_scaling[i];

	for (i=0; i<dofs; i++)
	{

		Y[i] = 0.0; // Empty for safety
		Dy[i] = 0.0; // Empty for safety
		DDy[i] = 0.0; // Empty for safety

		for(k=0; k<dmps; k++)
			for(j=0; j<bfs; j++)
				Weights[k][i][j] = 0.0; // Empty weight vector for safety
	}
	GenerateGaussianCenters(runtime); // Set Radial Basis Function Centers and Varience
}


// Run DMP system one time step
template<size_t DOF>
void DMP<DOF>::RunOneStep() 
{
	CSx = CSx - CSax * CSx * dt / tau[SelectedDmps]; // Run Canonical system one time step
	float sum = GenerateActivationFunction(); // Calculate Radial Basis Function value
	//float sum = 0.0;
	float w_phi = 0.0;
	uint8_t i;
	unsigned short int j;
	for (i = 0; i<Dofs; i++) 
	{
		w_phi = 0.0;
		// force = sumation(weight*RBF value)/sumation(RBF value)
		for (j=0; j<Bfs; j++) 
		{
			w_phi = w_phi + Weights[SelectedDmps][i][j] * Psi[j];
			//sum = sum + Psi[j];
		}

		force = (w_phi * CSx) / sum;
		// acceleration = ay*( by*(error) - deriavtive_error) + force
		// stable dynamics as force is zero when canonical system goes to zero
		DDy[i] = (ay * (by * (Goals[SelectedDmps][i] - Y[i]) - Dy[i]) + force)/(tau[SelectedDmps]*tau[SelectedDmps]);
		Dy[i] = Dy[i] + DDy[i] * dt;
		Y[i] = Y[i] + Dy[i] * dt;
		//Y[i] = Y0[i] + 0.5*DDy[i]*DDy[i]*dt*dt;
	}
	
}

template<size_t DOF>
void DMP<DOF>::CheckGoalOffset() //
{
	// Check to see if initial position and goal are the same
	// If they are, offset slightly so that the forcing term is not 0
	for (uint8_t i = 0; i<Dofs; i++)
		if (Y0[i] == Goals[SelectedDmps][i])
			Goals[SelectedDmps][i] = Goals[SelectedDmps][i] + 0.001;
}

template<size_t DOF>
void DMP<DOF>::CheckGoal() //
{
	// Check to see if initial position and goal are the same
	// If they are, offset slightly so that the forcing term is not 0
/*
	IsGoal = true;
	for (unsigned short int i = 0; i < Dmps; i++) 
	{
		if (abs(Y[i] - Goal[i]) < 0.005) 
		{
			//printf(" 1 ");
			IsGoal &= true;
		} 
		else 
		{
			//printf(" 0 ");
			IsGoal &= false;
		}

	}
	//if(IsGoal)
	//	printf(" = %d \n", IsGoal);

*/
	IsGoal = false;
	if(CSx<0.05)
	{
		IsGoal = true;
	}
}

// Reset DMP
template<size_t DOF>
void DMP<DOF>::ResetState() 
{
	CSx = 1.0;
	//CSax = 0.0;
	
	IsGoal = false;
	for (uint8_t  i=0; i<Dofs; i++) 
	{
		Y[i] = Y0[i];
		Dy[i] = 0;
		DDy[i] = 0;
	}
}

template<size_t DOF>
DMP<DOF>::DMP(unsigned short int dummyvariable):
		ref_jp(this, &ref_jp_OutputValue) ,ref_jv(this, &ref_jv_OutputValue)
{
	//InitDmpSys(dmps, bfs, a, b, runtime, tolerance);
	//SetDMPConditions(y0, goal); // Initial conditions
	//CheckDMPGaolOffset();
	//LoadWeights();
		
	InitializeFromFile("configdata.csv");
	ResetState();
	LoadWeights();
	
	for(uint8_t i = 0; i<Dofs; i++)
		ref_jp_tmp[i] = Y0[i];

	ref_jv_tmp(0.0);
}

#endif /* DMP_INL_HPP_ */
