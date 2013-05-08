/*
main.cpp
********
The main component has four main responsibilities: 
  -Configuration and initialization of the system upon start up
	-Provide an command line interface with the user
	-Build messages for agent(s) based on user specifications
	-Send new programs via TCP/IP

Struct robot:
Contains attributes:
		int id: contains numerical addressing id for agent
		char[] type: contains agent type (software, hardware, etc)
		char[] name: contains agent name (display)
		char[] driver_pkg: contains driver location for agent
		char[] driver_file: contains driver file name for agent
		char[] program: initial program for agent
		int state: integer representation of possible states

Method sysInfo():
		function: display status information
		input: id of agent, robot list	
		return: void 

Method buildMsg():
		function: build message 
		input: id of agent, robot list
		return: void 
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <sys/types.h>
#include <signal.h>

#define MIN_ARGS 2
#define MAX_LINE 100
#define MAX_ATTR_LEN 50
#define MAX_FILE_TYPES 2

FILE *fp;

//agent struct
typedef struct{
    int id;
    char type[MAX_ATTR_LEN];
    char name[MAX_ATTR_LEN];
    char driver_pkg[MAX_ATTR_LEN];
    char driver_file[MAX_ATTR_LEN];
    char program[MAX_ATTR_LEN];
    int state;
}robot;

//sysInfo() - print status information for all agents or individual
void sysInfo(int id, robot *bots);

//buildMsg() - build message to public to rostopic
void buildMsg(int target, char *message)

//main - initialize main, provide interface to agents via command 
//line menu
int main(int argc, char *argv[]){

	//check args
    if (argc < MIN_ARGS){
        printf("\nERROR: Usage: masc [config file], exiting.\n");
        exit(0);
    }

	//validate configuration file's type and existence
    int len = strlen(argv[1]);
    if (strcmp(argv[1] + len - 4, ".txt") != 0){
        printf("\nERROR: Config file must be of type '.txt', exiting.\n");
        exit(1);
    }

    fp = fopen(argv[1], "r");
    if (fp == NULL){
        perror(argv[1]);
        printf("\nERROR: Configuration file failed to open, exiting.\n");
        exit(1);
    }

	//if here, file exists.  build session information from file. 

    char line[MAX_LINE];
    int numBots;
    int ctr = 0;

    char *pch;

    char *t=fgets(line, sizeof line, fp);
    numBots = atoi(line);
    robot bots[numBots];

	//parse each line
    while (fgets(line, sizeof line, fp) != NULL){

		//id
        pch = strtok(line, "~");
        bots[ctr].id = atoi(pch);

		//agent type
        pch = strtok(NULL, "~");
        strcpy(bots[ctr].type, pch);

		//agent name
        pch = strtok(NULL, "~");
        strcpy(bots[ctr].name, pch);

		//agent driver package
        pch = strtok(NULL, "~");
        strcpy(bots[ctr].driver_pkg, pch);

		//agent driver file
        pch = strtok(NULL, "~");
        strcpy(bots[ctr].driver_file, pch);

        pch = strtok(NULL, "~");

        int n = strlen(pch);
        pch[n] = '\0';

        strcpy(bots[ctr].program, pch);

        bots[ctr].state = 1;
        ctr++;

		//exit if agents exceeds allowable agents for this session
        if (ctr > numBots){
            printf("\nERROR: Number of entries exceeds number of agents, please check config file.  Exiting.\n");
            exit(0);
        }
    }

	//reduce number of agents in system to reflect submitted entries
    if (ctr < numBots){
        numBots = ctr;
        printf("\nNumber of agents exceeds number of entries, updating system data\n");
    }

    fclose(fp);
    fflush(stdin);
    fflush(stdout);

	//begin ROS publisher logic
    ros::init(argc, argv, "masc_main");
    if(!ros::master::check()){
        printf("\nroscore not available, quitting.\n");
        exit(0);
    }

    ros::NodeHandle n;
    ros::Publisher masc_pub = n.advertise<std_msgs::String>("masc",1024);

    int option, msgTgt, msgType;
    std_msgs::String msg,c_line;

	//menu logic
    while(1){
        int su;
		char in[512];

		//how to interact with the system:
			//(info) - enter menu to get status information for the system 
			//(msg) - enter menu to send messages through the system
			//(exit) - send message for agents to exit, then exit normally 

        printf("\n(info) - SYSTEM INFO\n(msg) - SEND MESSAGE\n(exit) - EXIT\n\nACTION: ");
        su = scanf("%s", &in);

		//(exit) - send message for agents to exit, then exit normally 
        if (strcmp(in,"exit") == 0 && ros::ok()){

            std::stringstream ss;
            ss << "-1,-1";
            msg.data = ss.str();
            masc_pub.publish(msg);

            printf("\nExiting at user request.\n");
            exit(0);

		//(msg) - enter menu to send messages through the system
        }else if (strcmp(in, "msg") == 0 && ros::ok()){

            printf("\n(all) - SYSTEM\n(one) - AGENT\n\nTARGET: ");

            su = scanf("%s", &in);

			//"-1" is recognized by all agents as a universal address
            if (strcmp(in, "all") == 0){ 
				msgTgt = -1;
			}

			//which agent id to address
            else{
                printf("\nAGENT ID: ");
                int id;
                su = scanf("%s", &in);
                id = atoi(in);

				//ensure agent exists
                while (id <= 0 || id > numBots){
                    printf("Invalid input.  Enter ID of target: ");
                    su = scanf("%s", &in);
                    id = atoi(in);
                    fflush(stdin);
                }

				//assign agent id to message target variable 
                msgTgt = id;
            }

			//submenu - choose message type:
				//(start) - execute code
				//(end) - stop execution
				//(new) - notify agent of new program

            printf("\n(start) - EXECUTE CODE\n(end) - STOP EXECUTION\n(new) - NOTIFY NEW PROGRAM\n\nMESSAGE: ");
            su = scanf("%s", &in);

			//(new) - notify agent of new program
            if(strcmp(in,"new") == 0){

				//submenu - choose update type
					//(new) - send new program to compile
					//(change) - change to another existing executable in the agent's workspace
				printf("\n(new) or (change) working program?: ");
				su = scanf("%s",&in);

				//(new) - send new program to compile
				if(strcmp(in,"new") == 0){

					//generate a port number for use by main and agent for file transfer via TCP/IP
		            srand((int)time(NULL));
		            int port = (rand() % 2000 + 1024);
		            msgType = port;

		            char file[256];
		
					//get file name from user and check it exists
		            printf("\nFILE NAME ('program.ext'): ");
		            su = scanf("%s", file);

		            FILE *fp;
		            fp = fopen(file, "r");
		            while (fp == NULL){
		                perror(file);
		                printf("\nFile doesn't exist, please re-enter: ");
		                su = scanf("%s", file);
		                fp = fopen(file, "r");
		            }
					fclose(fp);

					//get file name from user that agent should write new data to
		            printf("\nEnter filename to write to: ");
		            char to[256];
		            su = scanf("%s", to);

					//tell agent to open client
					//publish msg (agent),(port number)
		            std::stringstream ss;	
		            ss << msgTgt << "," << msgType;
		            msg.data = ss.str();
		            masc_pub.publish(msg);

					//run server using menu specifications
					char command[512];
		            memset(command, '\0', sizeof(512));
					sprintf(command, "rosrun paper server %d %s %s", port, file, to);
			
		            int ret = system(command);
		            if (ret != 0) printf("ERROR: Server failure\n");

				//(change) - change to another existing executable in the agent's workspace
				}else if(strcmp(in, "change") == 0){

					//user submits name of existing executable in agent's workspace
					printf("\nenter program name:");
					su = scanf("%s", &in);									
											
					//publish msg (agent),(executable name)
					std::stringstream ss;
		            ss << msgTgt << "," << in;
		            msg.data = ss.str();
		            masc_pub.publish(msg);
					
				}

			//(start) - execute code
            }else if(strcmp(in, "start") == 0){

				//publish msg (agent),(start msg id)
                msgType = 1;
                std::stringstream ss;
                ss << msgTgt << "," << msgType;
                msg.data = ss.str();
                masc_pub.publish(msg);

			//(end) - stop execution
            }else if(strcmp(in, "end") == 0){

				//publish msg (agent),(end msg id)
                msgType = 2;
                std::stringstream ss;
                ss << msgTgt << "," << msgType;
                msg.data = ss.str();
                masc_pub.publish(msg);

			//something went wrong somewhere.  clean slate!: start over at the main menu.
            }else{
                printf("\nERROR: Invalid input.  Returning to main menu\n");
            }

		//(info) - enter menu to get status information for the system 
			//(all) - get info for all agents in system
			//(one) - get info for individual/group in system
        }else if (strcmp(in, "info") == 0){

            printf("\n(all) - SYSTEM\n(one) - AGENT\n\nFETCH FOR: ");
            su = scanf("%s", &in);

			//(all) - get info for all agents in system
            if (strcmp(in, "all") == 0){
                int i;
                for (i = 1; i <= numBots; i++)
                   sysInfo(i, bots);

			//(one) - get info for individual/group in system
            }else if (strcmp(in, "one") == 0){

                int id;
                printf("\nEnter agent ID: ");
                su = scanf("%d", &id);
                sysInfo(id, bots);
            }

		//unrecognized command, return to main menu
        }else{
            printf("\nERROR: Unrecognized command.  Returning to main menu.\n");
        }
    }

    return 0;
}

void buildMsg(int target, char *message){

}

//sysInfo() - print status information for all agents or individual
void sysInfo(int id, robot *bots){

    printf("\nID: %d TYPE: %s PROG: %sSTATE:",bots[id-1].id, bots[id-1].type, bots[id-1].program);
    switch (bots[id-1].state){
        case 1:printf("READY");break;
        case 2:printf("RUNNING");break;
        case 3:printf("WAITING");break;
        case 4:printf("STOPPED");break;
        default:break;
   }
   printf("\n\n");
}

