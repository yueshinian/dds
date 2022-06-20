#include <cmath>
#include <mutex>
#include <memory>
#include <thread>
#include <iostream>
#include <chrono>
#include<unistd.h>

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <errno.h>
#include <stdlib.h>
#include <arpa/inet.h>

using namespace std;

class udpComm
{
private:
  struct sockaddr_in sendServerAddr;
  int sendSock;
  socklen_t sendAddrLen;
  char sendBuff[128];
  unsigned int sendServerPort;
  char *serverIp;

  struct sockaddr_in serverAddr;
  struct sockaddr_in clientAddr;
  int recvSock;
  socklen_t clientAddrLength;
  char recvBuff[128];
  unsigned int recvPort;
  size_t recvLength;

public:
  udpComm(unsigned int sendServerPort_ = 7838 , unsigned int recvPort_ =7838):sendServerPort(sendServerPort_),recvPort(recvPort_)
  {
    sendServerPort = 7838;
    recvPort = 7838;
    serverIp = "127.0.0.1";
  }

  ~udpComm() {}

  void initSend()
  {
    /* 创建 socket , 关键在于这个 SOCK_DGRAM */
    if ((sendSock = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
    {
      perror("socket");
      exit(errno);
    }
    else
    {
      std::cout << "create send sock" << std::endl;
    }

    memset(&sendServerAddr, 0, sizeof(sendServerAddr));
    serverIp = "127.0.0.1";
    /* 设置对方地址和端口信息 */
    sendServerAddr.sin_family = AF_INET;
    sendServerAddr.sin_port = htons(sendServerPort);
    sendServerAddr.sin_addr.s_addr = inet_addr(serverIp);
  }

  void initRecv()
  {
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(recvPort);
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);

    if ((recvSock = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
      perror("socket");
      exit(1);
    }else{
      printf("creat recv sock");
    }
    //port bind to server
    if (bind(recvSock, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
      perror("bind");
      exit(1);
    }

    memset(&clientAddr, 0, sizeof(clientAddr));
    clientAddrLength = sizeof(clientAddr);
  }
  void recvMsg()
  {
    while (1)
    {
      memset(recvBuff, 0, sizeof(recvBuff));
      recvLength = recvfrom(recvSock, recvBuff, sizeof(recvBuff)-1, 0, (struct sockaddr *)&clientAddr, &clientAddrLength);
      if (recvLength < 0)
      {
        perror("recvfrom");
        exit(errno);
      }else{
        std::cout<<recvBuff<<std::endl;
      }
    }
  }
  int pubMsg(char *msg)
  {
    /* 发送UDP消息 */
    strcpy(sendBuff, msg);
    return sendto(sendSock, sendBuff, strlen(sendBuff), 0, (struct sockaddr *)&sendServerAddr, sizeof(sendServerAddr));
  }
};

int main()
{
  auto pub = make_shared<udpComm>();
  pub->initSend();
  char* msg = new char[128];
  msg[0]='1';
  for(int i=0;i<127;++i){
    //msg[i]=char(i);
    if(pub->pubMsg(msg)<0){
      std::cout<<"send error"<<std::endl;
    }else{
      std::cout<<"send msg "<<msg<<std::endl;
    }
    
    sleep(1);
  }
    
  return 0;
}