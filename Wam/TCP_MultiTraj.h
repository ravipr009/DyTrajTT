#ifndef TCP_COMM_H_
#define TCP_COMM_H_

#include <iostream>
#include <stdlib.h>
//#include <stdio.h>
#include <string.h>
//#include <unistd.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
//#include <pthread.h>
#include <boost/thread.hpp>

#define DATA_SIZE 2		

using namespace std;

class Server
{
	public:
		int Socket;
		int Port_Number;
		unsigned int New_Socket;
		unsigned int Client_Length;
		char Recieve_Data_Buffer[DATA_SIZE];
		char Send_Data_Buffer[DATA_SIZE];
		struct sockaddr_in Client_Address;
		struct sockaddr_in Server_Address;
		int Read_Result, Send_Result;
		bool RECIEVE_DATA;
		bool SEND_DATA;
		bool* RunRobot;
		uint8_t* Robot_DMP_Number;
		uint8_t* Robot_Total_DMP_Number;

		Server(int portnumber)
		{
			RECIEVE_DATA = true;
			Socket = socket(AF_INET, SOCK_STREAM, 0);

			if (Socket < 0)
				cout << "Error opening socket , check port number... \n";

			bzero((char *) &Server_Address, sizeof(Server_Address));
			Port_Number = portnumber;
			Server_Address.sin_family = AF_INET;
			Server_Address.sin_addr.s_addr = INADDR_ANY;
			Server_Address.sin_port = htons(Port_Number);
		
			if (bind(Socket, (struct sockaddr *) &Server_Address,sizeof(Server_Address)) < 0)
				cout << "Error on binding socket, check port number... \n";
			
		}

		bool WaitForClients()
		{
			listen(Socket,5);
			Client_Length = sizeof(Client_Address);

			New_Socket = accept(Socket,(struct sockaddr *) &Client_Address, &Client_Length);

			if (New_Socket < 0)
				return 0;
			else
				return 1;
		}

		bool RecieveData()
		{
			bzero(Recieve_Data_Buffer, DATA_SIZE);
			Read_Result = read(New_Socket, Recieve_Data_Buffer, DATA_SIZE);
			//cout<<Read_Result<<endl;
			if (Read_Result < 0)
				return 0;
			else
				return 1;
		}

		bool SendData()
		{
			Send_Result = write(New_Socket, Send_Data_Buffer, strlen(Send_Data_Buffer));
			if (Send_Result < 0)
				return 0;
			else
				return 1;
		}

    		static void *RecieveDataWrapper(void *wam)
    		{
			((Server *)wam)->CommunicateWithAcknowledgement();
			return 0;
    		}
		void CommunicateWithAcknowledgement()
		{
			//unsigned short int data = 0;
			bool status;
			while(Recieve_Data_Buffer[0] != 'q')
			{
				//strcpy(Send_Data_Buffer, "B");
				status = RecieveData();
				if(status)
				{
					//data = Recieve_Data_Buffer[0];
					//cout<<"data - "<<data<<Recieve_Data_Buffer[0]<<endl;
					if(!(*RunRobot))
					{
						if(uint8_t(Recieve_Data_Buffer[0]) < (*Robot_Total_DMP_Number))
						{
							//cout<<"RUN DMP number "<<(data-48)<<endl;
							*Robot_DMP_Number = uint8_t(Recieve_Data_Buffer[0]) - 48;
							*RunRobot = true;
							strcpy(Send_Data_Buffer, "A");
						}
						else
						{
							//cout<<"DONT RUN "<<(data-48)<<endl;
							strcpy(Send_Data_Buffer, "B");
						}
					}
					else
					{
						strcpy(Send_Data_Buffer, "C");
					}
					SendData();
				}
				else
				{
					//cout<<"error in recieving..."<<endl;
				}
				status = false;
			}
		}
};
#endif /* TCP_COMM_H_ */
