#ifndef DESOLVER_H_
#define DESOLVER_H_
//#define CAMERA 380
using namespace std;
#include <vector>
class CDESolver   //Differential Evolution Algorithm of selecting cameras
{
public:
	CDESolver(int D, int pop, int num, vector<vector<int>>&CF);
	int solve(int maxGenerations, int *cam);
	int **cf;
	int *zuida;
	int nDim;//��ռ�ά����8��
	int nPop;//��Ⱥ�и�����
	int generations;//��������
	int Min;//����������Сֵ��0��
	int Max;//�����������ֵ��379��
	int *trialSolution;
	int *bestSolution;
	int *popEnergy;
	int *population;
	int *populationnext;
	int bestEnergy;//����������ܿ�������������
	int trialEnergy;//ѡ������
	//int CF3[CAMERA][CAMERA];
	float F;//��������
	float CR;//��������
protected:
	void strategy(int candidate);
	int EnergyFunction(int testSolution[]);
	void SelectSamples(int candidate, int *r1, int *r2, int *r3);
};
#endif