/*
 * DMP_PID_Comm.cpp
 *
 *  Created on: 18-Jul-2015
 *      Author: Raj Nayan Samant
 */

//--------------------------------------------------
// HEADER FILES

#include <unistd.h>
#include <iostream>
#include <string>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/log.h>
#include <barrett/standard_main_function.h>
#include <boost/thread.hpp>
//#include <boost/thread.hpp>
//#include <barrett/thread/null_mutex.h>

using namespace barrett;
using detail::waitForEnter;
//#include <samlibs.h>

//#include <Dynamics.hpp>
//#include <Sliding_mode_4dof.hpp>

//#include <dummy_system.hpp>
//#include <GMM.hpp>
//#include <differentiator.hpp>
//#include <second_differentiator.hpp>
//#include <torque_observer.hpp>
//#include <dummy_system.hpp>
//#include <DMP_first.h>
//
//
////--------------------------------------------------
//#include <unistd.h>
//#include <iostream>
//#include <string>
//#include <barrett/units.h>
//#include <barrett/systems.h>
//#include <barrett/products/product_manager.h>
//#include <barrett/detail/stl_utils.h>
//#include <barrett/log.h>
//#include <barrett/standard_main_function.h>
//#include <samlibs.h>
#include <DMP_MultiTraj.h>
#include <TCP_MultiTraj.h>
//--------------------------------------------------

//--------------------------------------------------
// HEADER FILES
//--------------------------------------------------
//using namespace barrett;
//using detail::waitForEnter;
//--------------------------------------------------

//--------------------------------------------------
// MAIN
//--------------------------------------------------
template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm,
		systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);
//--------------------------------------------------

cout<<" 1   | Initializing Logger... "<<endl;

//--------------------------------------------------
// WAM INITIALIZATION
//--------------------------------------------------
	typedef boost::tuple<double, jp_type, jv_type, jp_type, jv_type> tuple_type;
	typedef systems::TupleGrouper<double, jp_type, jv_type, jp_type, jv_type> tg_type;
	tg_type tg;
	char tmpFile[] = "btXXXXXX";
	if (mkstemp(tmpFile) == -1) {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
	const double TRANSITION_DURATION = 0.5;
//--------------------------------------------------

cout<<" 1   | Logger Done... "<<endl;
cout<<" 2   | Initializing DMP... "<<endl;

//--------------------------------------------------
// DMP INITIALIZATION
//--------------------------------------------------
	//float a = 25.0; // PD values
	//float b = 25.0 / 4.0; // PD values
	//float y0[4] = { -0.00237402, -1.59189, -0.00670498, 3.1387 }; // Initial state [x0,y0,z0]
	//float goal[4] = { -0.00777947, -1.54302, -0.00214377, 3.1387};
	DMP<DOF> DMP(4);
	//DMP_first<DOF> DMP_first(DMPs, BFs, Ay, By, Final_Time, Steady_State_Tolerance, Initial_Pose, Final_Pose);

	//float a[7] = { 25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0 }; // PD values
	//float b[7] = { 25.0/4.0, 25.0/4.0, 25.0/4.0, 25.0/4.0, 25.0/4.0, 25.0/4.0, 25.0/4.0 }; // PD values
	//float y0[7] = { 0.00211835, -1.28781, -0.0433771, 3.15259, -0.0234051, -0.166366, -0.00811684 }; // Initial state [x0,y0,z0]
	//float goal[7] = { 0.00233749, 0.123696, -0.0396825, 3.15335, -0.0227725, -0.154505, -0.00739763 };
	//DMP<DOF> DMP(7, 100, a, b, 4.796, 0.05, y0, goal);
//--------------------------------------------------

cout<<" 2   | Initializing DMP Done... "<<endl;
cout<<" 3   | Initializing Communication... "<<endl;



//--------------------------------------------------
//Communication Initialization
//--------------------------------------------------
	Server Wam(atoi(argv[2]));
	Wam.Robot_DMP_Number = &DMP.SelectedDmps;
	Wam.RunRobot = &DMP.RunRobot;
	Wam.Robot_Total_DMP_Number = &DMP.Dmps;
	if(!Wam.WaitForClients())
	{
		cout << "ERROR client not connected... \n";
		return 0;
	}
	else
		cout<<"Connected to client..."<<endl;
	
	boost::thread t1(&Server::RecieveDataWrapper, &Wam);
//--------------------------------------------------

cout<<" 3   | Initializing Communication Done... "<<endl;
cout<<" 4   | Homing Robot... "<<endl;

//--------------------------------------------------
// ROBOT HOMING
//--------------------------------------------------
	jp_type startpos(0.0);
	startpos[0] = DMP.Y0[0];
	startpos[1] = DMP.Y0[1];
	startpos[2] = DMP.Y0[2];
	startpos[3] = DMP.Y0[3];
	//startpos[4] = y0[4]; 
	//startpos[5] = y0[5];
	//startpos[6] = y0[6];

	wam.gravityCompensate();
	printf("Press [Enter] to go to Home Position");
	waitForEnter();

	wam.moveTo(startpos);
//--------------------------------------------------

cout<<" 4   | Homing Robot Done... "<<endl;
cout<<" 5   | Initialization Timer... "<<endl;

//--------------------------------------------------
// TIME MANAGER
//--------------------------------------------------
	systems::Ramp time(pm.getExecutionManager(), 1.0);

	const size_t PERIOD_MULTIPLIER = 1;

	systems::PeriodicDataLogger<tuple_type> logger(pm.getExecutionManager(),
			new log::RealTimeWriter<tuple_type>(tmpFile,
					PERIOD_MULTIPLIER * pm.getExecutionManager()->getPeriod()),
			PERIOD_MULTIPLIER);

	systems::connect(tg.output, logger.input);
//--------------------------------------------------

cout<<" 5   | Timer Done... "<<endl;
cout<<" 4   | Initialization Timer... "<<endl;

	printf("Press [Enter] to Run your Code");
	waitForEnter();

//--------------------------------------------------
// MAIN CONNECTION
//--------------------------------------------------
	//systems::connect(tg.output, logger.input);
	//systems::connect(wam.jpOutput, DMP.CurrentStatePos);
	//systems::connect(wam.jvOutput, DMP.CurrentStateVel);
	wam.trackReferenceSignal(DMP.ref_jp);
//--------------------------------------------------



//--------------------------------------------------
// LOGGER CODE
//--------------------------------------------------
	systems::connect(time.output, tg.template getInput<0>());
	systems::connect(DMP.ref_jp, tg.template getInput<1>());
	systems::connect(DMP.ref_jv, tg.template getInput<2>());
	systems::connect(wam.jpOutput, tg.template getInput<3>());
	systems::connect(wam.jvOutput, tg.template getInput<4>());
//--------------------------------------------------



//--------------------------------------------------
// CLOSING CODE
//--------------------------------------------------
	time.smoothStart(TRANSITION_DURATION);
	printf("Press [Enter] to stop.");
	waitForEnter();
	logger.closeLog();
	time.smoothStop(TRANSITION_DURATION);
	wam.idle();
	pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	log::Reader<boost::tuple<tuple_type> > lr(tmpFile);
	lr.exportCSV(argv[1]);
	printf("Output written to %s.\n", argv[1]);
	std::remove(tmpFile);
	lr.close();
//--------------------------------------------------

	return 0;
}

