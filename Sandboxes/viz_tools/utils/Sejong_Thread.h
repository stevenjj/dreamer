#ifndef SEJONG_THREAD
#define SEJONG_THREAD

#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
 
class Sejong_Thread{
protected:
    pthread_t sejong_thread;
    bool firstLoopFlag;
    bool isRunning;


    void terminate();
    bool isFirstLoop();

public:
    Sejong_Thread();
    virtual ~Sejong_Thread(void);
    virtual void run(void) = 0;

    void start();
};
void *runData(void * arg);
void sigint(int signo);



#endif
