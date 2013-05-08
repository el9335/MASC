/*
client.cpp
**********

This is a basic C based implementation of a client program.  It's only additional functionality is to receive a new file from the server via TCP/IP and write the data to a file locally.
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 

//print error message for connection problems
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

//main - connect using given hostname/IP and port
int main(int argc, char *argv[])
{

  //check then assign args
    if (argc < 3) {
       fprintf(stderr,"usage %s hostname port\n", argv[0]);
       exit(0);
    }
    int portno = atoi(argv[2]);
    char buffer[256];

	//connection stuff
    int sockfd, n;
    struct sockaddr_in serv_addr;
    struct hostent *server;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(argv[1]);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0){ 
        error("ERROR connecting");
		exit(0);
    }

	/*-----------------------*/    

	 //receive destination file from server, open with write permission
     bzero(buffer,256);
     n = read(sockfd,buffer,255);
     if (n < 0){ 
		error("ERROR reading from socket");
		exit(0);
     }
	 FILE *fp;
	 fp = fopen(buffer, "w");

	 //confirm with server ready to accept code
     printf("Write to: %s\n",buffer);

	 char *ready="ready";
     n = write(sockfd,ready,sizeof(ready));
     if (n < 0){
		error("ERROR writing to socket");
		exit(0);
     }

	 char line[512];
     bzero(line, 512);
     
	 //receive new file line by line from server and
	 //print to destination file previously defined
     while (read(sockfd, line, 512)){
     
		fprintf(fp,"%s",line);
        bzero(line, 512);
     }

    fclose(fp);
    close(sockfd);							
 
	//compile new file
    char command[512];
    memset(command, '\0', sizeof(512));
	//CHANGE THIS ASAP
	sprintf(command, "g++ %s", buffer);	
	
	int ret = system(command);
    if (ret != 0) printf("g++ failure\n");
	
    return 0;
}
