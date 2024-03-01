

#include "b3RobotSimulatorClientAPI.h"

#include "b3RobotSimulatorClientAPI_NoGUI.h"

//#include "btCustom.h"

#include "../Utils/b3Clock.h"

#include <string.h>
#include <stdio.h>
#include <assert.h>

#include <fstream>
#include <sstream>
#include <iostream>
//#include "libxl.h"


#define ASSERT_EQ(a, b) assert((a) == (b));
#include "MinitaurSetup.h"

//Start class btcustom
//This needs to be refactored if it gets any further use

#include <vector>
#include <string>
#include <array>

float Deg2Rad(float deg) {
	return deg * (3.14159265359 / 180);
}

class btCustom
{
private:
	b3RobotSimulatorClientAPI_NoGUI* m_sim;

public:

	btCustom(bool GUI)
	{

		//This if - else statement somehow breaks it

		/*
		if (GUI) {
			b3RobotSimulatorClientAPI_NoGUI* m_sim = new b3RobotSimulatorClientAPI();
			m_sim->connect(eCONNECT_GUI);
		}
		else {
			b3RobotSimulatorClientAPI_NoGUI* m_sim = new b3RobotSimulatorClientAPI_NoGUI();
			m_sim->connect(eCONNECT_DIRECT);
		}
		printf("\n");
		printf("Simulation Connected\n");
		
		*/

		//b3RobotSimulatorClientAPI* m_sim = new b3RobotSimulatorClientAPI();
		//bool isConnected = m_sim->connect(eCONNECT_GUI);

		b3RobotSimulatorClientAPI_NoGUI* m_sim = new b3RobotSimulatorClientAPI_NoGUI();
		bool isConnected = m_sim->connect(eCONNECT_DIRECT);

		printf("Connected!\n");

		//#####################################################


		m_sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
		m_sim->setTimeOut(10);
		m_sim->syncBodies();
		btScalar fixedTimeStep = 1. / 240.;

		m_sim->setRealTimeSimulation(false);
		m_sim->setTimeStep(fixedTimeStep);
		m_sim->setGravity(btVector3(0, 0, -9.8));

		//#############################################################
		//Resetting world
		b3RobotSimulatorLoadUrdfFileArgs planePos;
		planePos.m_startPosition = btVector3(0, 0, -0.2);
		planePos.m_globalScaling = btVector3(1, 1, 1);
		m_sim->loadURDF("plane.urdf", planePos);

		b3RobotSimulatorCreateCollisionShapeArgs palletShape;
		b3RobotSimulatorCreateMultiBodyArgs mPalletShape;
		int palletID;
		int mPalletID;

		palletShape.m_halfExtents = btVector3(0.4, 0.6, 0.1);

		palletID = m_sim->createCollisionShape(3, palletShape); //first int is shapeType, in this case 3 = cube 
		mPalletShape.m_baseMass = 0;
		mPalletShape.m_baseCollisionShapeIndex = palletID;
		mPalletShape.m_basePosition = btVector3(0.4, 0.6, -0.1);
		mPalletID = m_sim->createMultiBody(mPalletShape);

		/*                                            
		b3RobotSimulatorCreateCollisionShapeArgs shape;       //Gare debug meetlat, kan weg
		b3RobotSimulatorCreateMultiBodyArgs body;
		int shapeID;
		int bodyID;

		shape.m_halfExtents = btVector3(0.01, 0.01, 1);

		shapeID = m_sim->createCollisionShape(3, shape); //first int is shapeType, in this case 3 = cube 

		body.m_baseMass = 0;
		body.m_baseCollisionShapeIndex = shapeID;
		body.m_basePosition = btVector3(-0.01, -0.01, 1);
		bodyID = m_sim->createMultiBody(body);
		*/
		

		//#############################################################
		//Reading an excel

		/*

		libxl::Book* book = xlCreateBook();

		if (book->loadSheet("stacks_003000.xls", 5))
		{
			printf("that was easy!");
		}
		*/

		//#############################################################
		//Reading an csv


		//Hyperparameters

		int maxsteps = 240;
		float tolerance = 0.05;
		float maxBoxes_range[6] = { 35, 36, 37, 38, 39, 40 };
		float theta_range[14] = { 0, 1, 2, 3, 4, 5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 10 };
		float phi_range[4] = { 0, 90 , 180, 270 };

		//Timing parameters

		b3Clock time;

		double globalStartTime = time.getTimeInSeconds();


		//for different numbers of boxes 
		for (int maxBoxes : maxBoxes_range) {

			//stack specific parameters
			std::vector<float> angles;
			std::vector<int> boxes;
			int numberOfBoxes = 0;


			//Find the csv file
			std::string filename{ "resources/stacks_003000.csv" };   // csv is from "stacks_003000.xls/pallet 2"
			std::fstream input{ filename };

			if (!input.is_open()) {
				//printf("Couldn't read file");
			}
			else {
				//printf("found the file");
			}


			//read csv file
			for (std::string line; std::getline(input, line);) {
				//if the maximum number of boxes hasn't been spawned yet, make one
				if (numberOfBoxes < maxBoxes) {

					std::istringstream ss(std::move(line));
					std::vector<float> data;

					//printf("analyzing line for box ");
					//std::cout << numberOfBoxes << "\n";

					for (std::string cel; std::getline(ss, cel, ',');) {
						float value = std::stof(cel);                       //conversion to float
						data.push_back(std::move(value));
						//printf("analyzed value: ");
						//std::cout << value << '\n';
					}

					//data: index, x, y, z, w, l, h, orient, mass
					//        0  , 1, 2, 3, 4, 5, 6,   7   , 8

					//printf("file read \n");

					float x = data[1];
					float y = data[2];
					float z = data[3];
					float w = data[4];
					float l = data[5];
					float h = data[6];

					float x0 = x + (w / 2);
					float y0 = y + (l / 2);
					float z0 = z + (h / 2);

					if (true) {  //simple boxes

						b3RobotSimulatorCreateCollisionShapeArgs boxSize;
						b3RobotSimulatorCreateMultiBodyArgs boxParam;
						int cBoxID;
						int boxID;

						boxSize.m_halfExtents = btVector3(w / 2, l / 2, h / 2);

						cBoxID = m_sim->createCollisionShape(3, boxSize); //first int is shapeType, in this case 3 = cube 
						boxParam.m_baseMass = 1;
						boxParam.m_baseCollisionShapeIndex = cBoxID;
						boxParam.m_basePosition = btVector3(x0, y0, z0);
						boxID = m_sim->createMultiBody(boxParam);

						boxes.push_back(boxID);
					}
					else {    // Pre-deformed boxes
						b3RobotSimulatorLoadUrdfFileArgs args;
						args.m_startPosition = btVector3(x0, y0, z0);
						args.m_globalScaling = btVector3(w, l, h);

						int boxID = m_sim->loadURDF("resources/unitBox.urdf", args);
					}

					numberOfBoxes++;
				}
			}
			printf("done loading boxes, starting simulation\n");



			//start timing
			double startTime = time.getTimeInSeconds();

			int bcount = boxes.size();
			std::vector<btVector3> loc_0;

			//save starting locations
			for (unsigned int i = 0; i < boxes.size(); i++) { 
				btVector3 position;
				btQuaternion orientation;
				m_sim->getBasePositionAndOrientation(boxes[i], position, orientation);
				loc_0.push_back(position);
			}
						int state = m_sim->saveStateToMemory();
			//printf("starting state saved\n");

			//for north, eastm south, west...
			for (float phi : phi_range) {								// direction  angle of tilt

				float fallAngle = 15;

				//test each angle from the list 
				for (float theta : theta_range) {                       // iterating through list of angles

					float x = 10 * sin(Deg2Rad(theta)) * sin(Deg2Rad(phi));
					float y = 10 * sin(Deg2Rad(theta)) * cos(Deg2Rad(phi));
					float z = -10 * cos(Deg2Rad(theta));

					m_sim->setGravity(btVector3(x, y, z));                //Change gravity

					//Simulate for a bit
					for (unsigned int i = 0; i < maxsteps; i++) { 
						m_sim->stepSimulation();
						//printf("simulating...\n");
					}

					//checking displacements

					//Get locations
					std::vector<btVector3> loc;
					for (unsigned int i = 0; i < boxes.size(); i++) {
						btVector3 position;
						btQuaternion orientation;
						m_sim->getBasePositionAndOrientation(boxes[i], position, orientation);
						loc.push_back(position);
					}

					//record largest distance for printing
					float largestDist = 0;

					//Calculate displacement
			

					for (unsigned int j = 0; j < loc.size(); j++) {

						float dx = pow(loc[j][0] - loc_0[j][0], 2);
						float dy = pow(loc[j][1] - loc_0[j][1], 2);
						float dz = pow(loc[j][2] - loc_0[j][2], 2);
						float dist = pow(dx + dy + dz, 0.5);


						if (dist > largestDist) {
							largestDist = dist;
						}
						//Check if displacement is smaller than tolerace
						if (dist > tolerance) {
							fallAngle = theta;
							goto thetaIsDone;							        //break theta loop
						}

					}
					//debug print
					//std::cout << "boxes = " << maxBoxes << ", phi = " << phi << ", theta = " << theta << ", maxdist = " << largestDist << ". \n";
			

				}
				thetaIsDone:

				//record angle and reset state
				angles.push_back(fallAngle);
				//std::cout << "for phi = " << phi << ", theta = " << fallAngle << "\n";
				m_sim->restoreStateFromMemory(state);

			}

			//reset world and remove state
			m_sim->removeState(state);
			for (unsigned int i = 0; i < boxes.size(); i++) {
				m_sim->removeBody(boxes[i]);
			}
			m_sim->setGravity(btVector3(0, 0, 10));

			double calcTime = time.getTimeInSeconds() - startTime;

			std::cout << "stack done; " << maxBoxes << " boxes, time: " << calcTime;
			std::cout << " s, angles: [ " << angles[0] << ", " << angles[1] << ", ";
			std::cout << angles[2] << ", " << angles[3] << "] \n";


		}

		double totalTime = time.getTimeInSeconds() - globalStartTime;

		std::cout << "Done with " << sizeof(maxBoxes_range) << "stacks, total time: " << totalTime << "s \n";
		//#############################################################
		/*
		
		b3RobotSimulatorCreateCollisionShapeArgs boxShape;

		b3RobotSimulatorCreateMultiBodyArgs bodyShape;
		int boxInt;
		int box;

		boxShape.m_halfExtents = btVector3(0.5, 0.5, 1);

		boxInt = m_sim->createCollisionShape(3, boxShape); //first int is shapeType, in this case 3 = cube 
		bodyShape.m_baseMass = 1;
		bodyShape.m_baseCollisionShapeIndex = boxInt;
		bodyShape.m_basePosition = btVector3(1, 1, 1);
		box = m_sim->createMultiBody(bodyShape);
		*/

		//#############################################################

		b3Clock clock;
		double startTime = clock.getTimeInSeconds();
		double simWallClockSeconds = 200.;
#if 1
		while (clock.getTimeInSeconds() - startTime < simWallClockSeconds)
		{
			m_sim->stepSimulation();
		}
#endif
		m_sim->setRealTimeSimulation(false);
		int vidLogId = -1;
		int minitaurLogId = -1;
		int rotateCamera = 0;


		printf("sim->disconnect\n");

		m_sim->disconnect();

		printf("delete sim\n");
		delete m_sim;

		printf("exit\n");

	}

	~btCustom()
	{
		m_sim->disconnect();
		delete m_sim;
	}

	enum boxtype { deform, soft, simple };

	void btCustom::resetWorld()  //(bool deformable)
	{

		/*

		m_sim->resetSimulation(); //Check if flags can be supplied for deformable world
		
		b3RobotSimulatorCreateCollisionShapeArgs arg ;
		arg.m_halfExtents = btVector3(0.4,  0.6,  0.1);
		int palletShape = m_sim->createCollisionShape(GEOM_BOX, arg);
		b3RobotSimulatorCreateMultiBodyArgs args;
		args.m_baseMass = 1;
		args.m_baseCollisionShapeIndex = palletShape;
		args.m_basePosition = btVector3(0.4, 0.6, -0.1);
		int pallet = m_sim->createMultiBody(args);

		*/


	}

	int btCustom::loadBox(btCustom::boxtype boxtype, float w, float l, float h, float x, float y, float z, float mass)
	{
		//b3RobotSimulatorLoadUrdfFileArgs boxshape;
		//boxshape.m_startPosition = btVector3(x0, y0, z0);
		//boxshape.

		float x0 = x + w / 2;
		float y0 = y + l / 2;
		float z0 = z + h / 2;

		b3RobotSimulatorCreateCollisionShapeArgs boxShape;

		b3RobotSimulatorCreateMultiBodyArgs bodyShape;
		int boxInt;
		int box;

		switch (boxtype)
		{
		case btCustom::deform:
			break;
		case btCustom::soft:
			break;
		case btCustom::simple:
			
			boxShape.m_halfExtents = btVector3(w/2, l/2, h/2);

			boxInt = m_sim->createCollisionShape(3, boxShape); //first int is shapeType, in this case 3 = cube
			bodyShape.m_baseMass = mass;
			bodyShape.m_baseCollisionShapeIndex = boxInt;
			bodyShape.m_basePosition = btVector3(x0, y0, z0);
			box = m_sim->createMultiBody(bodyShape);
			break;

		default:
			//throw an error?
			break;
		}
		
		return box;
	}

	std::vector<int> btCustom::infoFromExcel(std::string filename, int sheetnumber)
	{
		return std::vector<int>();
	}

	std::vector<int> btCustom::loadFromExcel(btCustom::boxtype boxtype, std::string filename, int sheetnumber, int maxboxes)
	{
		
		
		
		return std::vector<int>();
	}

	std::vector<int> btCustom::getAngles(btCustom::boxtype boxtype, std::string filename, int sheetnumber, int maxboxes, bool logging)
	{
		return std::vector<int>();
	}

	

};


//End class btCustom


int main(int argc, char* argv[])
{


	btCustom p = btCustom(true);

	//p.resetWorld();

	

	/*

	b3RobotSimulatorClientAPI* sim = new b3RobotSimulatorClientAPI();
	bool isConnected = sim->connect(eCONNECT_GUI);

	//b3RobotSimulatorClientAPI_NoGUI* sim = new b3RobotSimulatorClientAPI_NoGUI();
	//bool isConnected = sim->connect(eCONNECT_DIRECT);

	if (!isConnected)
	{
		printf("Cannot connect\n");
		return -1;
	}
	printf("Connected!");


	sim->configureDebugVisualizer(COV_ENABLE_GUI, 0);
	sim->setTimeOut(10);
	sim->syncBodies();
	btScalar fixedTimeStep = 1. / 240.;

	sim->setTimeStep(fixedTimeStep);

	btQuaternion q = sim->getQuaternionFromEuler(btVector3(0.1, 0.2, 0.3));
	btVector3 rpy;
	rpy = sim->getEulerFromQuaternion(q);

	sim->setGravity(btVector3(0, 0, -9.8));

	//int blockId = sim->loadURDF("cube.urdf");
	//b3BodyInfo bodyInfo;
	//sim->getBodyInfo(blockId,&bodyInfo);

	sim->loadURDF("plane.urdf");

	//#############################################################

	b3RobotSimulatorCreateCollisionShapeArgs boxShape;

	b3RobotSimulatorCreateMultiBodyArgs bodyShape;
	int boxInt;
	int box;

	boxShape.m_halfExtents = btVector3(0.5, 0.5, 2);

	boxInt = sim->createCollisionShape(3, boxShape); //first int is shapeType, in this case 3 = cube 
	bodyShape.m_baseMass = 1;
	bodyShape.m_baseCollisionShapeIndex = boxInt;
	bodyShape.m_basePosition = btVector3(1, 1, 1);
	box = sim->createMultiBody(bodyShape);

	//#############################################################

	b3Clock clock;
	double startTime = clock.getTimeInSeconds();
	double simWallClockSeconds = 20.;
#if 1
	while (clock.getTimeInSeconds()-startTime < simWallClockSeconds)
	{
		sim->stepSimulation();
	}
#endif
	sim->setRealTimeSimulation(false);
	int vidLogId = -1;
	int minitaurLogId = -1;
	int rotateCamera = 0;

	
	printf("sim->disconnect\n");

	sim->disconnect();

	printf("delete sim\n");
	delete sim;

	printf("exit\n");

	*/

}
