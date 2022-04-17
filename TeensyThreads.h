/*
 * Threads.h - Library for threading on the Teensy.
 *
 *******************
 *
 * Copyright 2017 by Fernando Trias.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 *******************
 *
 * Multithreading library for Teensy board.
 * See Threads.cpp for explanation of internal functions.
 *
 * A global variable "threads" of type Threads will be created
 * to provide all threading functions. See example below:
 *
 *   #include <Threads.h>
 *
 *   volatile int count = 0;
 *
 *   void thread_func(int data){
 *     while(1) count++;
 *   }
 *
 *   void setup() {
 *     threads.addThread(thread_func, 0);
 *   }
 *
 *   void loop() {
 *     Serial.print(count);
 *   }
 *
 * Alternatively, you can use the std::threads class defined
 * by C++11
 *
 *   #include <Threads.h>
 *
 *   volatile int count = 0;
 *
 *   void thread_func(){
 *     while(1) count++;
 *   }
 *
 *   void setup() {
 *     std::thead th1(thread_func);
 *     th1.detach();
 *   }
 *
 *   void loop() {
 *     Serial.print(count);
 *   }
 *
 */

#ifndef _THREADS_H
#define _THREADS_H

#include <stddef.h>
#include <stdint.h>

// #define DEBUG

#ifndef TEENSY_MAX_THREADS
#define TEENSY_MAX_THREADS 12
#endif

extern "C" {
/**
 * @brief Header for the overflow interrupt function.
 * By default, it simply ends the errored thread.
 * This function can be defined elsewhere to override it.
 */
void stack_overflow_isr(void);
}

extern "C" void unused_isr(void);

namespace Thread {

/**
 * @brief  The maximum number of threads is hard-coded to simplify
 * the implementation. See notes of ThreadInfo.
 */
static int DEFAULT_TICKS = 8;
static int DEFAULT_STACK_SIZE = 2048;
static const int SVC_NUMBER = 0x21;
static const int SVC_NUMBER_ACTIVE = 0x22;
static const int MAX_THREADS = TEENSY_MAX_THREADS;
static const int DEFAULT_STACK0_SIZE = 8192; // Stack size for the main thread
static const int DEFAULT_TICK_MICROSECONDS = 100;
static const int UTIL_STATE_NAME_DESCRIPTION_LENGTH = 24;
static const int UTIL_THREADS_BUFFER_LENGTH = 64 + (72 * MAX_THREADS);

static const char *NIL_NAME = "NIL NAME";

/**
 * @brief  State of threading system
 */
enum SystemState : const int {
    STARTED = 1,
    STOPPED,
    FIRST_RUN,
};

/**
 * @brief  State of individual threads
 */
enum ThreadState : const int {
    EMPTY,
    RUNNING,
    ENDED,
    ENDING,
    SUSPENDED,
    GROWING,
};

typedef void (*ThreadFunction)(void *);
typedef void (*ThreadFunctionInt)(int);
typedef void (*ThreadFunctionNone)();
typedef int (*ThreadFunctionSleep)(int);
typedef void (*IsrFunction)();

// For debugging
extern IsrFunction save_systick_isr;
extern IsrFunction save_svcall_isr;

/**
 * @brief  The stack frame saved by the interrupt
 */
typedef struct {
    uint32_t r0;
    uint32_t r1;
    uint32_t r2;
    uint32_t r3;
    uint32_t r12;
    uint32_t lr;
    uint32_t pc;
    uint32_t xpsr;
} interrupt_stack_t;

/**
 * @brief  The stack frame saved by the context switch
 */
typedef struct {
    uint32_t r4;
    uint32_t r5;
    uint32_t r6;
    uint32_t r7;
    uint32_t r8;
    uint32_t r9;
    uint32_t r10;
    uint32_t r11;
    uint32_t lr;
#ifdef __ARM_PCS_VFP
    uint32_t s0;
    uint32_t s1;
    uint32_t s2;
    uint32_t s3;
    uint32_t s4;
    uint32_t s5;
    uint32_t s6;
    uint32_t s7;
    uint32_t s8;
    uint32_t s9;
    uint32_t s10;
    uint32_t s11;
    uint32_t s12;
    uint32_t s13;
    uint32_t s14;
    uint32_t s15;
    uint32_t s16;
    uint32_t s17;
    uint32_t s18;
    uint32_t s19;
    uint32_t s20;
    uint32_t s21;
    uint32_t s22;
    uint32_t s23;
    uint32_t s24;
    uint32_t s25;
    uint32_t s26;
    uint32_t s27;
    uint32_t s28;
    uint32_t s29;
    uint32_t s30;
    uint32_t s31;
    uint32_t fpscr;
#endif
} software_stack_t;

/**
 * @brief  The state of each thread (including thread 0)
 */
struct ThreadInfo {
    volatile int flags = 0;
    uint8_t *stack = nullptr;
    int stack_size = 0;
    int ticks;
    void *sp = nullptr;
    int my_stack = 0;
    software_stack_t save;
    volatile int sleep_time_till_end_tick; // Per-task sleep time
#ifdef DEBUG
    const char *name = nullptr; // Name is only stored in debug mode, only used for infoString
    unsigned long cyclesStart;  // On T_4 the CycCnt is always active - on T_3.x it currently is not - unless Audio starts it AFAIK
    unsigned long cyclesAccum;
#endif
    inline bool invalid() { return stack == 0; }
};

/**
 * @brief Creates a new thread using the function "func" with the form "void p(void *)", passing it the argument "arg".
 * If stack is 0, the stack is allocated on heap.
 *
 * @param func Function to use
 * @param arg argument to pass, 0 by default
 * @param stack_size size of the stack, -1 by default
 * @param stack pointer to the stack that should be used, 0 by default
 * @param name Name of the thread
 * @return int ID of the newly created thread, -1 if it failed
 */
int addThread(ThreadFunction func, void *arg = 0, int stack_size = -1, void *stack = 0, const char *name = NIL_NAME);

/**
 * @brief Creates a new thread using the function "func" with the form "void p(int)", passing it the integer "arg".
 * If stack is 0, the stack is allocated on heap.
 *
 * @param func Function to use
 * @param arg argument to pass, 0 by default
 * @param stack_size size of the stack, -1 by default
 * @param stack pointer to the stack that should be used, 0 by default
 * @param name Name of the thread
 * @return int ID of the newly created thread, -1 if it failed
 */
int addThread(ThreadFunctionInt func, int arg = 0, int stack_size = -1, void *stack = 0, const char *name = NIL_NAME);

/**
 * @brief Creates a new thread using the function "func" with the form "void p()".
 * If stack is 0, the stack is allocated on heap.
 *
 * @param func Function to use
 * @param stack_size size of the stack, -1 by default
 * @param stack pointer to the stack that should be used, 0 by default
 * @param name Name of the thread
 * @return int ID of the newly created thread, -1 if it failed
 */
int addThread(ThreadFunctionNone func, int stack_size = -1, void *stack = 0, const char *name = NIL_NAME);

/**
 * @brief Get the state of a thread
 * 
 * @param id ID of the thread
 * @return int See class constants. Can be EMPTY, RUNNING, etc.
 */
int getState(int id);

/**
 * @brief  Explicityly set a state. See getState(). Call with care.
 */
int setState(int id, int state);

/**
 * @brief  Wait until thread returns up to timeout_ms milliseconds. If ms is 0, wait indefinitely.
 */
int wait(int id, unsigned int timeout_ms = 0);

/**
 * @brief  If using sleep, please run this in infinite loop
 */
void idle();

/**
 * @brief  Suspend execution of current thread for ms milliseconds
 */
void sleep(int ms);

/**
 * @brief  Permanently stop a running thread. Thread will end on the next thread slice tick.
 */
int kill(int id);

/**
 * @brief  Suspend a thread (on the next slice tick). Can be restarted with restart().
 */
int suspend(int id);

/**
 * @brief  Restart a suspended thread.
 */
int restart(int id);

/**
 * @brief  Set the slice length time in ticks for a thread (1 tick = 1 millisecond, unless using MicroTimer)
 */
void setTimeSlice(int id, unsigned int ticks);

/**
 * @brief  Set the slice length time in ticks for all new threads (1 tick = 1 millisecond, unless using MicroTimer)
 */
void setDefaultTimeSlice(unsigned int ticks);

/**
 * @brief  Set the stack size for new threads in bytes
 */
void setDefaultStackSize(unsigned int bytes_size);

/**
 * @brief Use the microsecond timer provided by IntervalTimer & PIT; instead of 1 tick = 1 millisecond,
 * 1 tick will be the number of microseconds provided (default is 100 microseconds)
 */
int setMicroTimer(int tick_microseconds = DEFAULT_TICK_MICROSECONDS);

/**
 * @brief  Simple function to set each time slice to be 'milliseconds' long
 */
int setSliceMillis(int milliseconds);

/**
 * @brief  Set each time slice to be 'microseconds' long
 */
int setSliceMicros(int microseconds);

/**
 * @brief  Set sleep callback function
 */
void setSleepCallback(ThreadFunctionSleep callback);

/**
 * @brief  Get the id of the currently running thread
 */
int id();

/**
 * @brief Grow the stack of the current thread, WIP
 *
 * @warning Not safe to use, must ensure registers don't point to anywhere in stack
 *
 * @param size bytes to grow by
 * @return int -1 if failed
 */
int growStack(int size);

/**
 * @brief Grow the stack of a running thread, WIP
 *
 * @warning This function is a WIP, not at all safe to use
 *
 * @param id Id of the thread
 * @param size bytes to grow by
 * @return int -1 if failed
 */
int growStack(int id, int size);

int getStackUsed(int id);
int getStackRemaining(int id);
void printStack(int id);
char *infoString(void);
#ifdef DEBUG
unsigned long getCyclesUsed(int id);
#endif

/**
 * @brief  Yield current thread's remaining time slice to the next thread, causing immediate context switch
 */
void yield();

/**
 * @brief  Wait for milliseconds using yield(), giving other slices your wait time
 */
void delay(int millisecond);

/**
 * @brief  Wait for microseconds using yield(), giving other slices your wait time
 */
void delay_us(int microsecond);

/**
 * @brief  Start/restart threading system; returns previous state: STARTED, STOPPED, FIRST_RUN
 * can pass the previous state to restore
 */
int start(int old_state = -1);

/**
 * @brief  Stop threading system; returns previous state: STARTED, STOPPED, FIRST_RUN
 */
int stop();

/**
 * @brief  Test all stack markers; if ok return 0; problems, return -1 and set *threadid to id
 */
int testStackMarkers(int *threadid = NULL);

class Mutex {
private:
    volatile int state = 0;
    volatile int waitthread = -1;
    volatile int waitcount = 0;

public:
    int getState();                        // get the lock state; 1=locked; 0=unlocked
    int lock(unsigned int timeout_ms = 0); // lock, optionally waiting up to timeout_ms milliseconds
    int try_lock();                        // if lock available, get it and return 1; otherwise return 0
    int unlock();                          // unlock if locked
};

class Scope {
private:
    Mutex *r;

public:
    Scope(Mutex &m) {
        r = &m;
        r->lock();
    }
    ~Scope() { r->unlock(); }
};

class Suspend {
private:
    int save_state;

public:
    Suspend();  // Stop threads and save thread state
    ~Suspend(); // Restore saved state
};

template <class C>
class GrabTemp {
private:
    Mutex *lkp;

public:
    C *me;
    GrabTemp(C *obj, Mutex *lk) {
        me = obj;
        lkp = lk;
        lkp->lock();
    }
    ~GrabTemp() { lkp->unlock(); }
    C &get() { return *me; }
};

template <class T>
class Grab {
private:
    Mutex lk;
    T *me;

public:
    Grab(T &t) { me = &t; }
    GrabTemp<T> grab() { return GrabTemp<T>(me, &lk); }
    operator T &() { return grab().get(); }
    T *operator->() { return grab().me; }
    Mutex &getLock() { return lk; }
};

#define ThreadWrap(OLDOBJ, NEWOBJ) Thread::Grab<decltype(OLDOBJ)> NEWOBJ(OLDOBJ);
#define ThreadClone(NEWOBJ) (NEWOBJ.grab().get())
} // namespace Thread

/*
 * Rudimentary compliance to C++11 class
 *
 * See http://www.cplusplus.com/reference/thread/thread/
 *
 * Example:
 * int x;
 * void thread_func() { x++; }
 * int main() {
 *   std::thread(thread_func);
 * }
 *
 */
namespace std {
class thread {
private:
    int id;      // internal thread id
    int destroy; // flag to kill thread on instance destruction
public:
    /**
     * @brief  By casting all (args...) to (void*), if there are more than one args,
     * the compiler will fail to find a matching function.
     * This fancy template just allows any kind of function to match.
     */
    template <class F, class... Args>
    explicit thread(F &&f, Args &&...args) {
        id = Thread::addThread((Thread::ThreadFunction)f, (void *)args...);
        destroy = 1;
    }

    /**
     * @brief  If thread has not been detached when destructor called, then thread must end
     */
    ~thread() {
        if (destroy)
            Thread::kill(id);
    }

    /**
     * @brief  Threads are joinable until detached per definition, but in this implementation
     * that's not so. We emulate expected behavior anyway.
     */
    bool joinable() { return destroy == 1; }

    /**
     * @brief  Once detach() is called, thread runs until it terminates; otherwise it terminates
     * when destructor called.
     */
    void detach() { destroy = 0; }

    /**
     * @brief  In theory, the thread merges with the running thread; if we just wait until
     * termination, it's basically the same thing except it's slower because
     * there are two threads running instead of one. Close enough.
     */
    void join() { Thread::wait(id); }

    /**
     * @brief  Get the unique thread id.
     */
    int get_id() { return id; }
};

class mutex {
private:
    Thread::Mutex mx;

public:
    void lock() { mx.lock(); }
    bool try_lock() { return mx.try_lock(); }
    void unlock() { mx.unlock(); }
};

template <class cMutex>
class lock_guard {
private:
    cMutex *r;

public:
    explicit lock_guard(cMutex &m) {
        r = &m;
        r->lock();
    }
    ~lock_guard() { r->unlock(); }
};
} // namespace std

#endif
