#include "Sejong_Thread.h"

Sejong_Thread::Sejong_Thread() :
    sejong_thread(),
    firstLoopFlag(false),
    isRunning(false)
{}

Sejong_Thread::~Sejong_Thread()
{
    pthread_cancel(sejong_thread);
    pthread_join(sejong_thread, NULL);
}
void Sejong_Thread::terminate()
{
    printf("terminating thread\n");
    isRunning = false;
}
bool Sejong_Thread::isFirstLoop()
{
    return firstLoopFlag;
}

void *runThread(void * arg)
{
    ((Sejong_Thread*) arg)->run();
    return NULL;
}

void sigint(int signo)
{
    (void) signo;
}
void Sejong_Thread::start()
{
    if(!isRunning){
        sigset_t sigset, oldset;
        sigemptyset(&sigset);
        sigaddset(&sigset, SIGINT);
        pthread_sigmask(SIG_BLOCK, &sigset, &oldset);
        pthread_create(&sejong_thread, NULL, runThread, this);
        struct sigaction s;
        s.sa_handler = sigint;
        sigemptyset(&s.sa_mask);
        s.sa_flags = 0;
        sigaction(SIGINT, &s, NULL);
        pthread_sigmask(SIG_SETMASK, &oldset, NULL);

        isRunning = true;
    }
}

