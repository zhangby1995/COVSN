#ifndef GASOLVER_H_
#define GASOLVER_H_
#include <vector>
using namespace std;
class CGASolver   //Genetic Algorithm of selecting cameras
{
public:
	CGASolver(int D, int pop, int Num, vector<vector<int>>&CF);
	int solve(int maxGenerations, int *cam);
	int b;//�������ۺ���ֵ
	int d;//����ڵڼ���
	int *code;
	int **cf;
	int nDim;//��ռ�ά����8��
	//int longth;//������볤��
	int nPop;//��Ⱥ�и�����
	//int generations;//��������
	int Min;//����������Сֵ��0��
	int Max;//�����������ֵ��379��
	int *c;//���Ž�
	int *trialSolution2;
	int *trialSolution;
	int *worstSolution;
	int *bestSolution;
	int *popEnergy;
	int *population;
	int *populationnext;
	int *zuida;
	int *worstEnegy;//���������ܿ�������������
	int bestEnergy;//����������ܿ�������������
	int trialEnergy2;
	int trialEnergy;//ѡ������
	//int CF3[CAMERA][CAMERA];
	float pc;//crossover probability
	float pm;//mutation probability
	//float F;//��������
	float CR;//ѡ������
protected:
	int select(int sum);
	/*void strategy(int candidate);*/
	int EnergyFunction(int testSolution[]);
	/*void SelectSamples(int candidate,int *r1,int *r2,int *r3);*/
};
#endif