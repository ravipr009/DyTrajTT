#ifndef Search_H_
#define Search_H_

#define NUMBEROFTRAJECTORIES 1

class Search
{
	public:
		unsigned short int DataBase[NUMBEROFTRAJECTORIES][3];
		char *Index;
		void NearestDMP(unsigned short int point[3], char*);
		void LoadDataBase();
};

void Search::LoadDataBase()
{
	ifstream File("Data.csv");
	unsigned short int i, j;
	string line;
	
	if(File.good())
	{
		cout<<" DataBase : "<<endl;
		for(i=0; i<NUMBEROFTRAJECTORIES; i++)
		{
			cout<<" ";
			for(j=0; j<3; j++)
			{
				getline(File, line, ',');
				DataBase[i][j] = atoi(line.c_str());
				cout<<DataBase[i][j]<<",";
			}
			cout<<endl;
		}
		File.close();
	}
	else
	{
		cout<<" Cannot find Data.csv"<<endl;
		exit(0);
	}	
}

void Search::NearestDMP(unsigned short int point[3], char* index)
{
	uint8_t i, j;
	float dist;
	float minimumdist = 200;
	*index = 10;
	
	for ( i=0; i<NUMBEROFTRAJECTORIES; i++)
	{
		dist = 0;
		for(j=0; j<3; j++)
		{
	 		dist += (DataBase[i][j] - point[j])*(DataBase[i][j] - point[j]);
		}
		//cout<<dist<<endl;
		dist = sqrt(dist);
		if(dist < minimumdist)
		{
			*index = i;
			minimumdist = dist;
		}		
	}
	//*index = 0;
	#ifdef DEBUG
	cout<<"DMP Number : "<<int(*index)<<endl;


	#endif
}


#endif
