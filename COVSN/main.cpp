#include <iostream>
#include "PlyLoader.h"
#include "Camera.h"
#include "Mymath.h"
#include "GDSS.h"
#include "GreedySolver.h"
#include "GASolver.h"
#include "PSOSolver.h"
#include "DESolver.h"
#include "BIPSolver.h"
#include "HClock.h"
void main()
{
	char* file = "SCtest600.ply";
	vector<Vector3f> vertex;					 //vertexes of the model
	vector<Triangle> triangle;					 //triangles of the model
	float A, f, Ca, ZS, su, sv, Ra, u0, v0, w, h;//internal parameters of cameras
	A = 2;
	f = 8;
	su = 0.0053;
	sv = 0.0053;
	ZS = 1200;
	Ca = 1.5;
	w = 1600;
	h = 1280;
	u0 = 800;
	v0 = 640;
	Ra = 1.0 / 3.0;
	float Zn = ZS*f*f / (f*f + A*Ca*min(su, sv)*ZS);
	float Zf = ZS*f*f / (f*f - A*Ca*min(su, sv)*ZS);
	int D = 7;					//dimension of the solution
	int NG,NGA,NPSO,NDE,NBIP;	//number of triangles covered by the D cameras
	int *cam;					//interger array which records the indices of selected cameras
	cam = new int[D];
	int popGA = 30;
	int popPSO = 20;
	int popDE = 30;
	int maxGenerationGA=4000;
	int maxGenerationPSO = 4500;
	int maxGenerationDE = 12000;//when GA,PSO and DE adopt their maxGeneration respectively, they consum roughly equal time.
	vector<Camera> camera;						 //external parameters of cameras, including there coordinates and orientations
	vector<vector<int>>CF;					//n rows n columns camera-triangle matrix
	CPLYLoader plyloader;
	plyloader.LoadModel(file,vertex,triangle);
	GenerateCamera(vertex, triangle, camera, A, f, Ca, ZS, su, sv, Ra, u0, v0, w, h, Zn, Zf);

	HClock timeSGDSS, timePGDSS, timeDE, timeGA, timePSO, timeG, timeBIP;

	cout << "S-GDSS start" << endl;
	timeSGDSS.StartClock();
	SGDSS(vertex, triangle, camera, CF, A, f, Ca, ZS, su, sv, Ra, u0, v0, w, h, Zn, Zf);
	timeSGDSS.EndClock();
	cout << "S-GDSS complete" << endl;

	cout << "P-GDSS start" << endl;
	timePGDSS.StartClock();
	PGDSS(vertex, triangle, camera, CF, A, f, Ca, ZS, su, sv, Ra, u0, v0, w, h, Zn, Zf);
	timePGDSS.EndClock();
	cout << "P-GDSS complete" << endl;
	
	cout << "G-SC start" << endl;
	timeG.StartClock();
	CGreedSolver Gsolver(D, triangle.size(), CF);
	NG=Gsolver.solve(cam);
	timeG.EndClock();
	cout << "G-SC complete" << endl;
	
	cout << "GA-SC start" << endl;
	timeGA.StartClock();
	CGASolver GAsolver(D, popGA, triangle.size(), CF);
	NGA=GAsolver.solve(maxGenerationGA, cam);
	timeGA.EndClock();
	cout << "GA-SC complete" << endl;

	cout << "PSO-SC start" << endl;
	timePSO.StartClock();
	CPSOSolver PSOsolver(D, popPSO, triangle.size(), CF);
	NPSO = PSOsolver.solve(maxGenerationPSO, cam);
	timePSO.EndClock();
	cout << "PSO-SC complete" << endl;
	
	cout << "DE-SC start" << endl;
	timeDE.StartClock();
	CDESolver DEsolver(D, popDE, triangle.size(), CF);
	NDE = DEsolver.solve(maxGenerationDE, cam);
	timeDE.EndClock();
	cout << "DE-SC complete" << endl;

	cout << "BIP-SC start" << endl;
	timeBIP.StartClock();
	CBIPSolver BIPsolver(triangle.size(), CF, D);
	NBIP = BIPsolver.solve(cam);
	timeBIP.EndClock();
	cout << "BIP-SC complete" << endl;

	cout << endl << endl;
	cout << "time cost of S-GDSS is:   " << timeSGDSS.GetTime() << "ms" << endl;
	cout << "time cost of P-GDSS is:   " << timePGDSS.GetTime() << "ms" << endl;
	cout << "time cost of G-SC is    " << timeG.GetTime() <<"ms"<< endl;
	cout << "number of triangles covered is:   " << NG  << endl;
	cout << "time cost of GA-SC is    " << timeGA.GetTime() << "ms" << endl;
	cout << "number of triangles covered is:   " << NGA  << endl;
	cout << "time cost of PSO-SC is    " << timePSO.GetTime() << "ms" << endl;
	cout << "number of triangles covered is:   " << NPSO  << endl;
	cout << "time cost of DE-SC is   " << timeDE.GetTime() << "ms" << endl;
	cout << "number of triangles covered is:   " << NDE  << endl;
	cout << "time cost of BIP-SC is   " << timeBIP.GetTime() << "ms" << endl;
	cout << "number of triangles covered is:   " << NBIP  << endl;
	int i = 1;
}
