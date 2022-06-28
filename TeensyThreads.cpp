/*
 * Threads.cpp - Library for threading on the Teensy.
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
 */
#include "TeensyThreads.h"
#include <Arduino.h>
#include <map>
#include <string.h>

#ifndef __IMXRT1062__

#include <IntervalTimer.h>
namespace Thread {
IntervalTimer context_timer;
} // namespace Thread
#endif

/**
 * @brief The main thread 0 stack
 */
extern char *_estack;

Thread::ThreadInfo threads[Thread::MAX_THREADS] = {
    {Thread::RUNNING, (uint8_t *)&_estack - Thread::DEFAULT_STACK0_SIZE, Thread::DEFAULT_STACK0_SIZE, Thread::DEFAULT_TICKS, 1},
};

/*
 * Store the PIT timer flag register for use in assembly
 */
volatile uint32_t *context_timer_flag;
extern volatile uint32_t systick_millis_count;

#define __flush_cpu() __asm__ volatile("DMB");

// These variables are used by the assembly context_switch() function.
// They are copies or pointers to data in Thread and ThreadInfo
// and put here seperately in order to simplify the code.
extern "C" {
#ifdef __IMXRT1062__
int currentUseSystick = 0; // using Systick vs PIT/GPT
#else
int currentUseSystick = 1; // using Systick vs PIT/GPT
#endif
int currentActive = Thread::FIRST_RUN; // state of the system (first, start, stop)
int currentCount = Thread::DEFAULT_TICKS;
void *currentSave = &threads[0].save;
int currentMSP = 1; // Stack pointers to save
void *currentSP = 0;
Thread::ThreadInfo *currentThread = threads;
} // the thread currently running

extern "C" void stack_overflow_default_isr() {
    currentThread->flags = Thread::ENDED;
}

extern "C" void stack_overflow_isr(void) __attribute__((weak, alias("stack_overflow_default_isr")));

namespace Thread {

int current_thread = 0;
int thread_count = 1;
int thread_error = 0;
/*
 * Set a marker at memory so we can detect memory overruns
 */
const uint32_t stackMarker = 0xEFBEADDE;
const int overflow_stack_size = 8;

IsrFunction save_systick_isr;
IsrFunction save_svcall_isr;
ThreadFunctionSleep enter_sleep_callback = NULL;

void getNextThread();
void *loadstack(ThreadFunction p, void *arg, void *stackaddr, int stack_size);
void force_switch_isr();
void setStackMarker(void *stack);
void del_process(void);
void yield_and_start();

unsigned int time_start;
unsigned int time_end;

static std::map<uint8_t, uint> priorities;

extern "C" {
void loadNextThread() {
    getNextThread();
}
}

/*
 * Teensy 3:
 * Replace the SysTick interrupt for our context switching. Note that
 * this function is "naked" meaning it does not save it's registers
 * on the stack. This is so we can preserve the stack of the caller.
 *
 * Interrupts will save r0-r4 in the stack and since this function
 * is short and simple, it should only use those registers. In the
 * future, this should be coded in assembly to make sure.
 */

extern "C" void systick_isr();
void __attribute((naked, noinline)) threads_systick_isr(void) {
    if (save_systick_isr) {
        asm volatile("push {r0-r4,lr}");
        (*save_systick_isr)();
        asm volatile("pop {r0-r4,lr}");
    }

    // TODO: Teensyduino 1.38 calls MillisTimer::runFromTimer() from SysTick
    if (currentUseSystick) {
        // we branch in order to preserve LR and the stack
        __asm volatile("b context_switch");
    }
    __asm volatile("bx lr");
}

void __attribute((naked, noinline)) threads_svcall_isr(void) {
    if (save_svcall_isr) {
        asm volatile("push {r0-r4,lr}");
        (*save_svcall_isr)();
        asm volatile("pop {r0-r4,lr}");
    }

    // Get the right stack so we can extract the PC (next instruction)
    // and then see the SVC calling instruction number
    __asm volatile("TST lr, #4 \n"
                   "ITE EQ \n"
                   "MRSEQ r0, msp \n"
                   "MRSNE r0, psp \n");
    register unsigned int *rsp __asm("r0");
    unsigned int svc = ((uint8_t *)rsp[6])[-2];
    if (svc == SVC_NUMBER) {
        __asm volatile("b context_switch_direct");
    } else if (svc == SVC_NUMBER_ACTIVE) {
        currentActive = STARTED;
        __asm volatile("b context_switch_direct_active");
    }
    __asm volatile("bx lr");
}

#ifdef __IMXRT1062__

/*
 * Teensy 4:
 * Use unused GPT timers for context switching
 */
extern "C" void unused_interrupt_vector(void);

static void __attribute((naked, noinline)) gpt1_isr() {
    GPT1_SR |= GPT_SR_OF1; // clear set bit
    __asm volatile("dsb"); // see github bug #20 by manitou48
    __asm volatile("b context_switch");
}

static void __attribute((naked, noinline)) gpt2_isr() {
    GPT2_SR |= GPT_SR_OF1; // clear set bit
    __asm volatile("dsb"); // see github bug #20 by manitou48
    __asm volatile("b context_switch");
}

bool gtp1_init(unsigned int microseconds) {
    // Initialization code derived from @manitou48.
    // See https://github.com/manitou48/teensy4/blob/master/gpt_isr.ino
    // See https://forum.pjrc.com/threads/54265-Teensy-4-testing-mbed-NXP-MXRT1050-EVKB-(600-Mhz-M7)?p=193217&viewfull=1#post193217

    // keep track of which GPT timer we are using
    static int gpt_number = 0;

    // not configured yet, so find an inactive GPT timer
    if (gpt_number == 0) {
        if (!NVIC_IS_ENABLED(IRQ_GPT1)) {
            attachInterruptVector(IRQ_GPT1, &gpt1_isr);
            NVIC_SET_PRIORITY(IRQ_GPT1, 255);
            NVIC_ENABLE_IRQ(IRQ_GPT1);
            gpt_number = 1;
        } else if (!NVIC_IS_ENABLED(IRQ_GPT2)) {
            attachInterruptVector(IRQ_GPT2, &gpt2_isr);
            NVIC_SET_PRIORITY(IRQ_GPT2, 255);
            NVIC_ENABLE_IRQ(IRQ_GPT2);
            gpt_number = 2;
        } else {
            // if neither timer is free, we fail
            return false;
        }
    }

    switch (gpt_number) {
    case 1:
        CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON); // enable GPT1 module
        GPT1_CR = 0;                                  // disable timer
        GPT1_PR = 23;                                 // prescale: divide by 24 so 1 tick = 1 microsecond at 24MHz
        GPT1_OCR1 = microseconds - 1;                 // compare value
        GPT1_SR = 0x3F;                               // clear all prior status
        GPT1_IR = GPT_IR_OF1IE;                       // use first timer
        GPT1_CR = GPT_CR_EN | GPT_CR_CLKSRC(1);       // set to peripheral clock (24MHz)
        break;
    case 2:
        CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON); // enable GPT1 module
        GPT2_CR = 0;                                  // disable timer
        GPT2_PR = 23;                                 // prescale: divide by 24 so 1 tick = 1 microsecond at 24MHz
        GPT2_OCR1 = microseconds - 1;                 // compare value
        GPT2_SR = 0x3F;                               // clear all prior status
        GPT2_IR = GPT_IR_OF1IE;                       // use first timer
        GPT2_CR = GPT_CR_EN | GPT_CR_CLKSRC(1);       // set to peripheral clock (24MHz)
        break;
    default:
        return false;
    }

    return true;
}

#endif

/*************************************************/
/**\name UTILITIES FUNCTIONS                     */
/*************************************************/
/**
 * \brief Convert thead state to printable string
 */
char *_util_state_2_string(int state) {
    static char _state[UTIL_STATE_NAME_DESCRIPTION_LENGTH];
    memset(_state, 0, sizeof(_state));

    switch (state) {
    case 0:
        sprintf(_state, "EMPTY");
        break;
    case 1:
        sprintf(_state, "RUNNING");
        break;
    case 2:
        sprintf(_state, "ENDED");
        break;
    case 3:
        sprintf(_state, "ENDING");
        break;
    case 4:
        sprintf(_state, "SUSPENDED");
        break;
    case 5:
        sprintf(_state, "GROWING");
        break;
    default:
        sprintf(_state, "%d", state);
        break;
    }

    return _state;
}

static const struct initer { // TODO: remove
    initer() {
        setStackMarker(threads[0].stack);
#ifdef __IMXRT1062__
        // commandeer SVCall & use GTP1 Interrupt
        save_svcall_isr = _VectorsRam[11];
        if (save_svcall_isr == unused_interrupt_vector)
            save_svcall_isr = 0;
        _VectorsRam[11] = threads_svcall_isr;

        gtp1_init(DEFAULT_TICK_MICROSECONDS); // tick every millisecond
#else

        // currentUseSystick = 1;

        // Commandeer the SVCall &SysTick Exceptions
        save_svcall_isr = _VectorsRam[11];
        if (save_svcall_isr == unused_isr)
            save_svcall_isr = 0;
        _VectorsRam[11] = threads_svcall_isr;

        save_systick_isr = _VectorsRam[15];
        if (save_systick_isr == unused_isr)
            save_systick_isr = 0;
        _VectorsRam[15] = threads_systick_isr;

        setMicroTimer(DEFAULT_TICK_MICROSECONDS);

#ifdef DEBUG
#if defined(__MK20DX256__) || defined(__MK20DX128__)
        ARM_DEMCR |= ARM_DEMCR_TRCENA; // Make ssure Cycle Counter active
        ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
#endif
#endif

#endif
    }
} initer;

/*
 * start() - Begin threading
 */
int start(int prev_state) {
    __disable_irq();
    int old_state = currentActive;
    if (prev_state == -1)
        prev_state = STARTED;
    currentActive = prev_state;
    __enable_irq();
    return old_state;
}

/*
 * stop() - Stop threading, even if active.
 *
 * If threads have already started, this should be called sparingly
 * because it could destabalize the system if thread 0 is stopped.
 */
int stop() {
    __disable_irq();
    int old_state = currentActive;
    currentActive = STOPPED;
    __enable_irq();
    return old_state;
}

void _printStack(int id, void *sp, uint8_t *stack, int size) {
    Serial.printf("\n----[ Stack %d:%d %p @ %p ]----\n", id, size, stack, sp);
    for (int i = 0; i < size; i++) {
        if (sp == stack + i)
            Serial.printf(">> %02X ", (stack)[i]);
        else
            Serial.printf("%02X ", (stack)[i]);

        if (i && !((i + 1) % 16))
            Serial.print('\n');
    }
}

/*
 * getNextThread() - Find next running thread
 *
 * This will also set the context_switcher() state variables
 */
void getNextThread() {

#ifdef DEBUG
    // Keep track of the number of cycles expended by each thread.
    // See @dfragster: https://forum.pjrc.com/threads/41504-Teensy-3-x-multithreading-library-first-release?p=213086#post213086
    currentThread->cyclesAccum += ARM_DWT_CYCCNT - currentThread->cyclesStart;
#endif

    // First, save the currentSP set by context_switch
    currentThread->sp = currentSP;

    // did we overflow the stack (don't check thread 0)?
    // allow an extra 8 bytes for a call to the ISR and one additional call or variable
    if (current_thread && ((uint8_t *)currentThread->sp - currentThread->stack <= overflow_stack_size)) {
        stack_overflow_isr();
    }

    // Find the next running thread
    while (1) {
        current_thread++;
        if (current_thread >= MAX_THREADS) {
            current_thread = 0; // thread 0 is MSP; always active so return
            break;
        }

        ThreadInfo *tp = threads + current_thread;

        if (tp->invalid())
            continue;

        if (tp->flags != ENDED && *((uint32_t *)tp->stack) != stackMarker) {
            // TODO: Notify of fault
            kill(current_thread);
            continue;
        }

        if (tp->flags == RUNNING)
            break;
    }

    currentCount = threads[current_thread].ticks;

    currentThread = threads + current_thread;
    currentSave = &threads[current_thread].save;
    currentMSP = !current_thread;
    currentSP = threads[current_thread].sp;

#ifdef DEBUG
    currentThread->cyclesStart = ARM_DWT_CYCCNT;
#endif
}

#ifndef __IMXRT1062__
/*
 * Empty placeholder for IntervalTimer class
 */
static void context_pit_empty() {}
#endif

/*
 * Defined in assembly code
 */
extern "C" void context_switch_pit_isr();

/*
 * Stop using the SysTick interrupt and start using
 * the IntervalTimer timer. The parameter is the number of microseconds
 * for each tick.
 */
int setMicroTimer(int tick_microseconds) {
#ifdef __IMXRT1062__

    gtp1_init(tick_microseconds);

#else

    /*
     * Implementation strategy suggested by @tni in Teensy Forums; see
     * https://forum.pjrc.com/threads/41504-Teensy-3-x-multithreading-library-first-release
     */

    // lowest priority so we don't interrupt other interrupts
    context_timer.priority(255);
    // start timer with dummy fuction
    if (context_timer.begin(context_pit_empty, tick_microseconds) == 0) {
        // failed to set the timer!
        return 0;
    }
    currentUseSystick = 0; // disable Systick calls

    // get the PIT number [0-3] (IntervalTimer overrides IRQ_NUMBER_t op)
    int number = (IRQ_NUMBER_t)context_timer - IRQ_PIT_CH0;
    // calculate number of uint32_t per PIT; should be 4.
    // Not hard-coded in case this changes in future CPUs.
    const int width = (PIT_TFLG1 - PIT_TFLG0) / sizeof(uint32_t);
    // get the right flag to ackowledge PIT interrupt
    context_timer_flag = &PIT_TFLG0 + (width * number);
    attachInterruptVector(context_timer, context_switch_pit_isr);

#endif

    return 1;
}

/*
 * Set each time slice to be 'microseconds' long
 */
int setSliceMicros(int microseconds) {
    setMicroTimer(microseconds);
    setDefaultTimeSlice(1);
    return 1;
}

/*
 * Set each time slice to be 'milliseconds' long
 */
int setSliceMillis(int milliseconds) {
    if (currentUseSystick) {
        setDefaultTimeSlice(milliseconds);
    } else {
        // if we're using the PIT, we should probably really disable it and
        // re-establish the systick timer; but this is easier for now
        setSliceMicros(milliseconds * 1000);
    }
    return 1;
}

void resetPriorities() {
    for (ThreadInfo &thread : threads) {
        uint i = priorities[thread.priority];
        uint8_t p = thread.priority;
        int n = thread_count;
        int t = TICK_BUDGET;
        thread.ticks = (4 * n * t - n * i * p - i * t) / (3 * n * n); // \frac{4nt-nip-it}{3n^{2}}
        thread.ticks += !thread.ticks;                                // Add one tick if zero
    }
}

void endThread(ThreadInfo *thread) {
    thread->flags = ENDED; // clear the flags so thread can stop and be reused
    priorities[thread->priority]--;
    thread_count--;
    resetPriorities();
}

/*
 * del_process() - This is called when the task returns
 *
 * Turns thread off. Thread continues running until next call to
 * context_switch() at which point it all stops. The while(1) statement
 * just stalls until such time.
 */
void del_process(void) {
    int old_state = stop();
    ThreadInfo *me = threads + current_thread;
    // Would love to delete stack here but the thread doesn't
    // end now. It continues until the next tick.
    // if (me->my_stack) {
    //   delete[] me->stack;
    //   me->stack = 0;
    // }
    // Serial.print("del:");
    // Serial.println(id());
    endThread(me);
    start(old_state);
    while (1)
        ; // just in case, keep working until context change when execution will not return to this thread
}

void setStackMarker(void *stack) {
    uint32_t *m = (uint32_t *)stack;
    *m = stackMarker;
}

/*
 * Users call this function to see if stack has been corrupted
 */
int testStackMarkers(int *threadid) {
    for (int i = 0; i < MAX_THREADS; i++) {
        if (threads[i].invalid())
            continue;
        if (threads[i].flags == RUNNING) {
            uint32_t *m = (uint32_t *)threads[i].stack;
            if (*m != stackMarker) {
                if (threadid)
                    *threadid = i;
                return -1;
            }
        }
    }
    return 0;
}

/*
 * Initializes a thread's stack. Called when thread is created
 */
void *loadstack(ThreadFunction p, void *arg, void *stackaddr, int stack_size) {
    interrupt_stack_t *process_frame = (interrupt_stack_t *)((uint8_t *)stackaddr + stack_size - sizeof(interrupt_stack_t) - overflow_stack_size);
    process_frame->r0 = (uint32_t)arg;
    process_frame->r1 = 0;
    process_frame->r2 = 0;
    process_frame->r3 = 0;
    process_frame->r12 = 0;
    process_frame->lr = (uint32_t)del_process;
    process_frame->pc = ((uint32_t)p);
    process_frame->xpsr = 0x1000000;
    uint8_t *ret = (uint8_t *)process_frame;
    // ret -= sizeof(software_stack_t); // uncomment this if we are saving R4-R11 to the stack
    return (void *)ret;
}

/*
 * Add a new thread to the queue.
 *    add_thread(fund, arg)
 *
 *    fund : is a function pointer. The function prototype is:
 *           void *func(void *param)
 *    arg  : is a void pointer that is passed as the first parameter
 *           of the function. In the example above, arg is passed
 *           as param.
 *    stack_size : the size of the buffer pointed to by stack. If
 *           it is 0, then "stack" must also be 0. If so, the function
 *           will allocate the default stack size of the heap using new().
 *    stack : pointer to new data stack of size stack_size. If this is 0,
 *           then it will allocate a stack on the heap using new() of size
 *           stack_size. If stack_size is 0, a default size will be used.
 *    return: an integer ID to be used for other calls
 */
int addThread(ThreadFunction p, void *arg, int stack_size, void *stack, const char *name, uint8_t priority) {
    int old_state = stop();
    if (stack_size <= -1)
        stack_size = DEFAULT_STACK_SIZE;
    for (int i = 1; i < MAX_THREADS; i++) {
        if (threads[i].flags == ENDED || threads[i].flags == EMPTY) {                       // free thread
            ThreadInfo *tp = threads + i;                                                   // working on this thread
            if (tp->stack && tp->my_stack && (tp->stack_size < stack_size || stack != 0)) { // TODO: Check that difference isn't huge
                delete[] tp->stack;
                tp->stack = 0;
                tp->stack_size = 0;
            }
            if (stack == nullptr) {
                if (tp->stack && tp->my_stack) {
                    stack = tp->stack;
                    stack_size = tp->stack_size;
                } else {
                    stack = new uint8_t[stack_size];
                    // memset(stack, 0, stack_size);
                    tp->my_stack = 1;
                }
            } else {
                tp->my_stack = 0;
            }

            setStackMarker(stack);
            tp->stack = (uint8_t *)stack;
            tp->stack_size = stack_size;
            void *psp = loadstack(p, arg, tp->stack, tp->stack_size);
            tp->sp = psp;
            tp->priority = priority;
            priorities[priority]++;
            tp->flags = RUNNING;
            tp->save.lr = 0xFFFFFFF9;
#ifdef DEBUG
            tp->name = name;
            tp->cyclesStart = ARM_DWT_CYCCNT;
            tp->cyclesAccum = 0;
#endif
            currentActive = old_state;
            thread_count++;

            resetPriorities();

            if (old_state == STARTED || old_state == FIRST_RUN)
                start();
            return i;
        }
    }
    if (old_state == STARTED)
        start();
    return -1;
}

int addThread(ThreadFunctionInt p, int arg, int stack_size, void *stack, const char *name, uint8_t priority) { return addThread((ThreadFunction)p, (void *)arg, stack_size, stack, name, priority); }
int addThread(ThreadFunctionNone p, int stack_size, void *stack, const char *name, uint8_t priority) { return addThread((ThreadFunction)p, 0, stack_size, stack, name, priority); }

int growStack(int size) {
    threads[current_thread].flags = GROWING;
    yield();
    return 1;
}

int growStack(int id, int size) {
    if (size > 0 && threads[id].flags == GROWING) {
        // int old_state = stop();

        ThreadInfo *tp = threads + id;

        int stack_size = size + tp->stack_size;
        uint8_t *old_stack = tp->stack;
        uint8_t *new_stack = new uint8_t[stack_size];

        memset(new_stack, 0, stack_size);

        setStackMarker(new_stack);
        int used = tp->stack_size - ((int)tp->sp - (int)old_stack);
        memcpy(new_stack + stack_size - used, tp->sp, used);

        // FIXME: I believe registers can still be pointing to old stack
        // interrupt_stack_t *new_pf = (interrupt_stack_t *)((uint8_t *)new_stack + stack_size - sizeof(interrupt_stack_t) - overflow_stack_size);
        // Serial.printf("%p %p:%p\n", tp->sp, old_stack, new_stack);
        // Serial.printf("%p %p %p %p %p %p %p %p %p\n", tp->save.lr, tp->save.r4, tp->save.r5, tp->save.r6, tp->save.r7, tp->save.r8, tp->save.r9, tp->save.r10, tp->save.r11);
        // Serial.printf("%p %p %p %p %p %p %p %p\n", new_pf->r0, new_pf->r1, new_pf->r2, new_pf->r3, new_pf->r12, new_pf->lr, new_pf->pc, new_pf->xpsr);

        memset(old_stack, 0, tp->stack_size); // Remove

        tp->stack = new_stack;
        tp->stack_size = stack_size;
        tp->sp = tp->stack + tp->stack_size - used;

        if (tp->my_stack)
            delete[] old_stack;
        tp->my_stack = 1;
        tp->flags = RUNNING;
        // start(old_state);
        return id;
    }
    return -1;
}

int getState(int id) {
    return threads[id].flags;
}

int setState(int id, int state) {
    threads[id].flags = state;
    return state;
}

int wait(int id, unsigned int timeout_ms) {
    unsigned int start = millis();
    // need to store state in temp volatile memory for optimizer.
    // "while (thread[id].flags != RUNNING)" will be optimized away
    volatile int state;
    while (1) {
        if (timeout_ms != 0 && millis() - start > timeout_ms)
            return -1;
        state = threads[id].flags;
        if (state != RUNNING)
            break;
        yield();
    }
    return id;
}

int kill(int id) {
    if (threads[id].flags != ENDED) {
        int old_state = stop();
        endThread(threads + id);
        start(old_state);
    }
    return id;
}

int suspend(int id) {
    threads[id].flags = SUSPENDED;
    return id;
}

int restart(int id) {
    threads[id].flags = RUNNING;
    return id;
}

void setTimeSlice(int id, unsigned int ticks) {
    threads[id].ticks = ticks - 1;
}

void setDefaultTimeSlice(unsigned int ticks) {
    DEFAULT_TICKS = ticks - 1;
}

void setDefaultStackSize(unsigned int bytes_size) {
    DEFAULT_STACK_SIZE = bytes_size;
}

void yield() {
    __asm volatile("svc %0"
                   :
                   : "i"(SVC_NUMBER));
}

void yield_and_start() {
    __asm volatile("svc %0"
                   :
                   : "i"(SVC_NUMBER_ACTIVE));
}

void delay(int millisecond) {
    int mx = millis();
    while ((int)millis() - mx < millisecond)
        yield();
}

/*
 * Experimental code for putting CPU into sleep mode during delays
 */

void setSleepCallback(ThreadFunctionSleep callback) {
    enter_sleep_callback = callback;
}

void delay_us(int microsecond) {
    int mx = micros();
    while ((int)micros() - mx < microsecond)
        yield();
}

void idle() {
    volatile bool needs_run[thread_count];
    volatile int i, j;
    volatile int task_id_ends;

    if (enter_sleep_callback == NULL)
        return;

    __disable_irq();
    task_id_ends = -1;
    // get lowest sleep interval from sleeping tasks into task_id_ends
    for (i = 0; i < thread_count; i++) {
        if (threads[i].invalid())
            continue;
        // sort by ending time first
        for (j = i + 1; j < thread_count; ++j) {
            if (threads[i].sleep_time_till_end_tick > threads[j].sleep_time_till_end_tick) {
                // if end time soonest
                if (getState(i + 1) == SUSPENDED) {
                    task_id_ends = j; // store next task
                }
            }
        }
    }
    if (task_id_ends == -1)
        return;

    // set the sleeping time to substractor
    int subtractor = threads[task_id_ends].sleep_time_till_end_tick;

    if (subtractor > 0) {
        // if sleep is needed
        volatile int time_spent_asleep = enter_sleep_callback(subtractor);
        // store new data based on time spent asleep
        for (i = 0; i < thread_count; i++) {
            if (threads[i].invalid())
                continue;
            needs_run[i] = 0;
            if (getState(i + 1) == SUSPENDED) {
                threads[i].sleep_time_till_end_tick -= time_spent_asleep; // substract sleep time
                // time to run?
                if (threads[i].sleep_time_till_end_tick <= 0) {
                    needs_run[i] = 1;
                } else {
                    needs_run[i] = 0;
                }
            }
        }
        // for each thread when slept, resume if needed
        for (i = 0; i < thread_count; i++) {
            if (threads[i].invalid())
                continue;
            if (needs_run[i]) {
                setState(i + 1, RUNNING);
                threads[i].sleep_time_till_end_tick = 60000;
            }
        }
    }
    __enable_irq();
    yield();
}

void sleep(int ms) {
    int i = id();
    if (getState(i) == RUNNING) {
        __disable_irq();
        threads[i - 1].sleep_time_till_end_tick = ms;
        setState(i, SUSPENDED);
        __enable_irq();
        yield();
    }
}

/* End of experimental code */

int id() {
    volatile int ret;
    __disable_irq();
    ret = current_thread;
    __enable_irq();
    return ret;
}

int getStackUsed(int id) {
    return threads[id].stack + threads[id].stack_size - (uint8_t *)threads[id].sp;
}

int getStackRemaining(int id) {
    return (uint8_t *)threads[id].sp - threads[id].stack;
}

void printStack(int id) {
    int old_state = stop();
    _printStack(id, threads[id].sp, threads[id].stack, threads[id].stack_size);
    start(old_state);
}

char *infoString(void) {
    static char _buffer[UTIL_THREADS_BUFFER_LENGTH];
    uint _buffer_cursor = sprintf(_buffer, "\n----[ Thread Info %d/%d ]----\n", thread_count, MAX_THREADS);
    uint ts = 0;
    for (int each_thread = 0; each_thread < MAX_THREADS; each_thread++) {
        if (threads[each_thread].invalid())
            continue;
        char *_thread_state = _util_state_2_string(threads[each_thread].flags);
        int used = getStackUsed(each_thread);
        int avlb = threads[each_thread].stack_size;
#ifdef DEBUG
        if (threads[each_thread].name != nullptr)
            _buffer_cursor += sprintf(_buffer + _buffer_cursor, " %-8s", threads[each_thread].name);
#endif
        _buffer_cursor += sprintf(_buffer + _buffer_cursor, " [%01d] %-9s | p:%d t:%d sz: %d/%d", each_thread, _thread_state, threads[each_thread].priority, threads[each_thread].ticks, used, avlb);
        ts += threads[each_thread].ticks;
        if (avlb) {
            _buffer_cursor += sprintf(_buffer + _buffer_cursor, " %.2f%%", 100.0f * used / avlb);
        }

#ifdef DEBUG
        _buffer_cursor += sprintf(_buffer + _buffer_cursor, " | cycles:%lu\n", threads[each_thread].cyclesAccum);
#else
        _buffer_cursor += sprintf(_buffer + _buffer_cursor, "\n");
#endif
    }
    _buffer_cursor += sprintf(_buffer + _buffer_cursor, "    ps: ");

    for (auto const &[k, v] : priorities)
        _buffer_cursor += sprintf(_buffer + _buffer_cursor, "%d/%d ", k, v);

    _buffer_cursor += sprintf(_buffer + _buffer_cursor, "| ts: %d", ts);

    return _buffer;
}

#ifdef DEBUG
unsigned long getCyclesUsed(int id) {
    stop();
    unsigned long ret = threads[id].cyclesAccum;
    start();
    return ret;
}
#endif

/*
 * On creation, stop threading and save state
 */
Suspend::Suspend() {
    __disable_irq();
    save_state = currentActive;
    currentActive = 0;
    __enable_irq();
}

/*
 * On destruction, restore threading state
 */
Suspend::~Suspend() {
    __disable_irq();
    currentActive = save_state;
    __enable_irq();
}

int Mutex::getState() {
    int p = stop();
    int ret = state;
    start(p);
    return ret;
}

int __attribute__((noinline)) Mutex::lock(unsigned int timeout_ms) {
    if (try_lock())
        return 1; // we're good, so avoid more checks

    uint32_t start = systick_millis_count;
    while (1) {
        if (try_lock())
            return 1;
        if (timeout_ms && (systick_millis_count - start > timeout_ms))
            return 0;
        if (waitthread == -1) { // can hold 1 thread suspend until unlock
            int p = stop();
            waitthread = current_thread;
            waitcount = currentCount;
            suspend(waitthread);
            Thread::start(p);
        }
        yield();
    }
    __flush_cpu();
    return 0;
}

int Mutex::try_lock() {
    int p = stop();
    if (state == 0) {
        state = 1;
        start(p);
        return 1;
    }
    start(p);
    return 0;
}

int __attribute__((noinline)) Mutex::unlock() {
    int p = stop();
    if (state == 1) {
        state = 0;
        if (waitthread >= 0) { // reanimate a suspended thread waiting for unlock
            restart(waitthread);
            waitthread = -1;
            __flush_cpu();
            yield_and_start();
            return 1;
        }
    }
    __flush_cpu();
    start(p);
    return 1;
}

} // namespace Thread