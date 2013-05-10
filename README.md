MASC
====

Multi-Agent Systems Controller

Author: Emily LeBlanc 2013


NOTE: If running on different machines, consult the following tutorial prior to starting MASC:

Running ROS across multiple machines:
http://www.ros.org/wiki/ROS/Tutorials/MultipleMachines

Quickstart:
===========

1.  Make 'build.sh' and 'make.sh' executable
2.  Run build.sh to make ros package 'paper'
3.  Copy 'main.cpp' and 'server.cpp' to primary machine.
4.  Copy 'agent.cpp' and 'client.cpp' to any agent machine (or to main if testing on single machine)
5.  Copy sample 'config.txt' to working directory.
6.  To run main application:


      rosrun paper masc config.txt
      
7.  To run agent application(s):
      

      rosrun paper agent [id] [node] [host IP]

      ex. rosrun paper agent 1 tomservo localhost
      
8.  Follow menu prompts to interact with the agents.  Consult user manual for detailed descriptions of functionality.
