#ifndef Communication_H_
#define Communication_H_

#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

#define DATA_SIZE 2

class Client
{
	public:
		int Socket;
		int Port_Number, n;
		struct sockaddr_in Server_Address;
		struct hostent *Server;
		char Send_Data_Buffer[DATA_SIZE];
		char Recieve_Data_Buffer[DATA_SIZE];
		int Read_Result, Send_Result;
		bool RECIEVE_DATA;
		bool SEND_DATA;
		

		Client(char* serverip,int portnumber)
		{
			Port_Number = portnumber;

			Socket = socket(AF_INET, SOCK_STREAM, 0);

			if (Socket < 0)
				cout << "Error opening socket \n";

			Server = gethostbyname(serverip);

			if(Server == NULL)
				cout << "NO SUCH HOST \n";

			bzero((char *)&Server_Address, sizeof(Server_Address));
			Server_Address.sin_family = AF_INET;
			bcopy((char *)Server->h_addr,(char *)&Server_Address.sin_addr,Server->h_length);

			Server_Address.sin_port = htons(Port_Number);
		}

		bool ConnectToServer()
		{
			if (connect(Socket,(sockaddr*)&Server_Address,sizeof(Server_Address)) < 0)
				return 0;
			else
				return 1;
		}

		bool RecieveData()
		{
			bzero(Recieve_Data_Buffer, DATA_SIZE);
			Read_Result = read(Socket,Recieve_Data_Buffer, DATA_SIZE);

			if (Read_Result < 0)
				return 0;
			else
				return 1;
		}

		bool SendData()
		{
			Send_Result = write(Socket,Send_Data_Buffer,strlen(Send_Data_Buffer));
			if (Send_Result < 0)
				return 0;
			else
				return 1;
		}

		bool Communicate(char dmpnumber)
		{
			bool status;
			Send_Data_Buffer[0] = dmpnumber + 48;
			cout<<int(Send_Data_Buffer[0])<<", "<<int(dmpnumber)<<endl;
			status = SendData();
//cout<<"com"<<endl;
			if(!status)
			{
				cout << "ERROR sending data to server... \n";
				return 0;
			}
			else
			{
				cout<<"send"<<endl;
			}
	//cout<<"wait rec"<<endl;
			status = RecieveData();
			if(!status)
			{
				cout << "ERROR recieving acknowledgment... \n";
				return 0;
			}
			else
			{
				switch(Recieve_Data_Buffer[0])
				{
					case 'A':
						cout<<"Recieved | Not moving | Will move"<<endl;
						break;
					case 'B':
						cout<<"Recieved | Not moving | Wont move"<<endl;
						break;
					case 'C':
						cout<<"Recieved | Moving | Continue move"<<endl;
						break;
				}
			}
		}

};

/*
int main(int argc, char *argv[])
{


	bool status;
	Client PC(argv[1], atoi(argv[2]));
	cout<<"Connecting to server..."<<endl;
	status = PC.ConnectToServer();
	if(!status)
	{
		cout << "ERROR connecting to server... \n";
		return 0;
	}
	else
		cout<<"Connected to server..."<<endl;
	
	
	//strcpy(Ace.Recieve_Data_Buffer, "hi");
	uint8_t r;
	char data;
	bzero(PC.Send_Data_Buffer, DATA_SIZE);
	while(r!='q')
	{
		PC.Recieve_Data_Buffer[0] = 'q';
		cin>>r;
		//data = r;
		//bzero(PC.Send_Data_Buffer, DATA_SIZE);
		PC.Send_Data_Buffer[0] = r;
		
		//strcpy(PC.Send_Data_Buffer, &data);
		status = PC.SendData();
		if(!status)
		{
			cout << "ERROR sending data to server... \n";
			return 0;
		}

		status = PC.RecieveData();
		if(!status)
		{
			cout << "ERROR recieving acknowledgment... \n";
			return 0;
		}
		else
		{
			switch(PC.Recieve_Data_Buffer[0])
			{
				case 'A':
					cout<<"Recieved | Not moving | Will move"<<endl;
					break;
				case 'B':
					cout<<"Recieved | Not moving | Wont move"<<endl;
					break;
				case 'C':
					cout<<"Recieved | Moving | Continue move"<<endl;
					break;
			}
		}

		/*if(!strcmp(Ace.Recieve_Data_Buffer, "T\n"))
			strcpy(Ace.Send_Data_Buffer, "Torque = 10");
		else if(!strcmp(Ace.Recieve_Data_Buffer, "X\n"))
			strcpy(Ace.Send_Data_Buffer, "X = 12");
		else
			strcpy(Ace.Send_Data_Buffer, "Wrong Keyword");

		Ace.SendData();*/
	//}
//	close(PC.Socket);

//	return 0;
//}
#endif
