#include"INS_GNSS/Combinenav.h"
#include"INS_GNSS/CombConfig.h"
int main_gnssins()
{

	CombineNav::Config config;
	config.Print();
	config.LoadFromFile("config.txt");
	if(config.methods.useRealTime)
	{
		CombineNav::gnssins_realtime(config);
	}
	else
	{
		CombineNav::gnssins(config);
	}
	
	//CombineNav::gnssins_backward();
	//CombineNav::gnssins_realtime(config);

	
	

	return 0;
}