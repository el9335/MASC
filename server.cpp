/*
server.cpp
**********

This is a basic C based implementation of a server.  It's only additional functionality is to send an existing file to the client via TCP/IP.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

//print error message for connection problems
void error(const char *msg)
{
    perror(msg);
    exit(1);
}

//main - listen on given port, transmit new text file
int main(int argc, char *argv[])
{

   //check then assign args
     if (argc < 4) {
         fprintf(stderr,"usage: [port] [file] [destination]\n");
         exit(1);
     }

	 char buffer[256];
     char new_program[256];
     bzero(new_program,256);
     char dest[256];
	 bzero(dest, 256);

     sprintf(new_program, "%s", argv[2]);
     printf("filename: %s\n",new_program);
	 sprintf(dest, "%s", argv[3]);
     printf("destination: %s\n", dest);

     //check file exists
     FILE *fp;
	 fp = fopen(new_program, "r");

	 if(fp == NULL){
		perror(new_program);
	    printf("\n[SERVER]: ERROR: File failed to open, returning \n");
		exit(1);
	 }

	 printf("%s exists and is ready to send\n", new_program);
 
     //connection stuff
     int sockfd, newsockfd, portno;
     socklen_t clilen;

     struct sockaddr_in serv_addr, cli_addr;
     int n;

     sockfd = socket(AF_INET, SOCK_STREAM, 0);
     if (sockfd < 0){ 
        error("ERROR opening socket");
		exit(1);
	 }
     bzero((char *) &serv_addr, sizeof(serv_addr));
     portno = atoi(argv[1]);
     serv_addr.sin_family = AF_INET;
     serv_addr.sin_addr.s_addr = INADDR_ANY;
     serv_addr.sin_port = htons(portno);
     if (bind(sockfd, (struct sockaddr *) &serv_addr,
              sizeof(serv_addr)) < 0){ 
              error("ERROR on binding");
			  exit(1);
	 }
     listen(sockfd,5);
     clilen = sizeof(cli_addr);
     newsockfd = accept(sockfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newsockfd < 0){ 
          error("ERROR on accept");
		  exit(1);
	}

	/*-----------------------*/

	//write destination file to client
    n = write(newsockfd,dest,strlen(dest));
    if (n < 0){ 
         error("ERROR writing to socket");
		exit(1);
	}

	//listen for ready signal from client
    bzero(buffer,256);
    n = read(newsockfd,buffer,255);
    if (n < 0){
        error("ERROR reading from socket");
		exit(1);
	}
    printf("received [%s] from client\n",buffer);

	//send new file line by line to client
	if (strcmp(buffer,"ready") == 0){
    	printf("write file to client\n");

		bzero(buffer, strlen(buffer));
         while(!feof(fp)){
			 printf("write");
             fgets(buffer, strlen(buffer), fp);

             while (fgets(buffer, sizeof(buffer), fp)) {
                write(newsockfd, buffer, strlen(buffer));
		    }
          }
	}

     fclose(fp);
     close(newsockfd);
     close(sockfd);

     return 0; 
}
