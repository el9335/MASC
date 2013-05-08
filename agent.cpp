/*
agent.cpp
*********
The agent component has two main responsibilities (including client):
  -Interpret and execute commands received from main application
	-Receive new programs, write to local file, and compile
	
String(C) my_program:	
		contains name of active program for agent
	
String(C) backup_program:
		contains name of last working active program in case 

method mascCallback():
		function: validates message (type and target), exit normally 					  if “exit” command is received
		    	  if message is valid, send to handleMsg() 
		input: address of received string
		return: void

method handleMsg():
		function: agent logic for processing and executing commands
		input: validated message from main
*/


#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sys/types.h>
#include <signal.h>
#include <cstdlib>
#include <cstring>
#include <cstdio>

int uid;
char *id;
int state;
char my_program[256];
int i=sprintf(my_program, "./fib");
char backup_prog[256];

pid_t pid;

//handleMsg() - agent logic
void handleMsg(const std_msgs::String::ConstPtr& msg);

//mascCallback() - callback function for message topic
void mascCallback(const std_msgs::String::ConstPtr& msg){

	//if exit message is received, kill any working process and exit normally.
    if(strcmp("-1,-1", msg->data.c_str()) == 0){
        printf("Main requested exit.\n");
        if(pid != NULL)kill(pid, SIGKILL);
        exit(0);
    }
	//else if message in intended for this agent (individual, or all), then send to handleMsg() agent logic function
    if(strncmp(id, msg->data.c_str(), 1) == 0 || strncmp("-1", msg->data.c_str(),2) ==0){
        handleMsg(msg);
    }
}

//main - initialize agent, wait for messages
int main(int argc, char **argv){

	//check then assign args
    if (argc < 2){
        printf("ERROR: Usage: robo [id] [node]");
        exit(0);
    }

    id = argv[1];
    uid = atoi(id);
    state = 1;
    printf("ID: %s STATE: %d\n", id, state);

	//begin ROS subscriber logic
    ros::init(argc, argv, argv[2]);
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("masc", 1024, mascCallback);

    ros::spin();

    return 0;
}

//handleMsg() - agent logic
void handleMsg(const std_msgs::String::ConstPtr& msg){

    int m;
    char *pch;
    char data[10];
    FILE *fp;

	//copy and parse incoming message
    strcpy(data,msg->data.c_str());

    pch = strtok(data, ",");
    pch = strtok(NULL, ",");

	char buffer[256];

    m = atoi(pch);
	printf("\nmsg: %s\n", pch);

	//if message doesn't convert to integer, 
	//then it is the name of a different (EXISTING!) program to be run 
    if (m == 0){
        
        //display new program name.
        bzero(buffer, sizeof(buffer));
		sprintf(buffer, "%s", pch);
		printf("\nchange working program to: %s\n",buffer);

		//save backup program in case executable does not work
		bzero(backup_prog, sizeof(backup_prog));
		sprintf(backup_prog, "%s", my_program);

		//change agent's program to incoming 
        bzero(my_program, sizeof(my_program));
		sprintf(my_program,"%s",buffer);
		printf("\nworking program: %s\n",my_program); 

	//messages are validated by main, however it is simple and practical to check again 
	//here before attempting to execute the received command
    }else if(m < 1 || (m > 3 && m < 1024)){ printf("ERROR: Invalid message, discarding.\n");}

	//message is an integer, either message type:
		//(m == 1) Begin Execution: Start active program
		//(m == 2) End Execution: End active program
		//(m >= 1024) Port Number: To be used by the agent receive a new program via TCP/IP

	//execute received command
    else{

		//(m == 1) Begin Execution: Start active program
        if (m == 1){

            printf("Begin execution.\n");
            state = 2;
			
			//save process id so that user can request process termination
            pid = fork();

			//build args, run executable on forked process
            if(pid < 0)printf("ERROR: fork fail\n");
            else if(pid == 0){

				printf("my_program: %s\n", my_program);
	        	char *args[] = {"xterm", "-e", my_program, (char*)0};
                execvp("xterm", args);

				//could not execute program, revert to backup (last working executable)
				printf("ERROR: %s is not executable from here.  Check agent has required files.\n",my_program);
				
				bzero(my_program, sizeof(my_program));
				sprintf(my_program, "%s", backup_prog);
				printf("Reverted to last working executable\n");
            }
            
		//(m == 2) End Execution: End active program
        }else if(m == 2){
			
			//use saved pid to terminate active program 
            printf("End execution.\n");
            if (pid != NULL){kill(pid,SIGKILL);
			pid = NULL;}
            else printf("No program open, continuing.\n");
            state = 4;

		//(m >= 1024) Port Number: To be used by the agent receive a new program via TCP/IP
        }else if (m >= 1024){

            state = 1;

			//build command, run client (see clientA.cpp)
            char command[512];
            memset(command, '\0', sizeof(512));
		    sprintf(command, "rosrun paper client localhost %d", m);	
	
		    int ret = system(command);
            if (ret != 0) printf("ERROR: Client failure\n");

			//THIS IS BULLSHIT FIX IT
			//assign new working program
			bzero(my_program, sizeof(my_program));
			sprintf(my_program, "./a.out");

		//extra safety check it would seem
        }else {
            printf("ERROR: Unknown message type\n");
        }
    }
}
