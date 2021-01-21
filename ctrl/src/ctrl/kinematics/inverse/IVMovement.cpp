#include"IVMovement.h"

IVMovement::IVMovement()
{
	invK = new InvKinematics();
	trajectory = new Trajectory();
}

IVMovement::~IVMovement()
{
}

Trajectory * IVMovement::getMovement(vector<SixDPos*>* _positions, Configuration * start_cfg)
{
	Configuration* prevConfig = start_cfg; 
	Configuration* correctConfig;
	vector<Configuration*>* configs;


	for (int i = 0; i< _positions->size(); i++)
	{
		configs = invK->get_inv_kinematics(_positions->at(i));
		correctConfig =  GetClosestConfiguration(configs, prevConfig);
		prevConfig = correctConfig;
		trajectory->add_configuration(correctConfig);
	}

	return trajectory;
}
Configuration* IVMovement::GetClosestConfiguration(vector<Configuration*>* _configs, Configuration* _prevConfig) {
	vector<Configuration*>::size_type NbConfigs = _configs->size();
	double minDist = 1000;
	int minConfig = 0;
	for (int i = 0; i < NbConfigs; i++) {
		Configuration* ActConfig = _configs->at(i);
		double actDist = 0;
		for (int j = 0; j > 6; j++) {
			actDist += pow((*ActConfig)[j] - (*_prevConfig)[j], 2);
		}
		actDist = sqrt(actDist);


		if (actDist < minDist) {
			minDist = actDist;
			minConfig = i;
		}
	}
	return _configs->at(minConfig);

}