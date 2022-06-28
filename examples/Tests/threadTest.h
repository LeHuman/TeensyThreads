#include "TeensyThreads.h"
#include <Arduino.h>

volatile int p1 = 0;
volatile int p2 = 0;
volatile int p3 = 0;
volatile int p4 = 0;

static const char *FAIL_STR = "-*-*-*-*-*-*-*-*-*-[ FAIL ]-*-*-*-*-*-*-*-*-*-";
static const char *OKAY_STR = "-------------------[ OKAY ]-------------------";

void my_priv_func1(int data) {
    p1 = 0;
    data *= 1000; // convert sec to ms;
    int mx = millis();
    while (1) {
        p1++;
        if ((int)millis() - mx > data)
            break;
    }
}

void my_priv_func2() {
    p2 = 0;
    while (1)
        p2++;
}

void my_priv_func3() {
    p3 = 0;
    while (1)
        p3++;
}

void my_priv_func_lock(void *lock) {
    Thread::Mutex *m = (Thread::Mutex *)lock;
    p4 = 0;
    m->lock();
    uint32_t mx = millis();
    while (millis() - mx < 500)
        p1++;
    m->unlock();
}

Thread::Mutex count_lock;
volatile int count1 = 0;
volatile int count2 = 0;
volatile int count3 = 0;

void lock_test1() {
    while (1) {
        count_lock.lock();
        for (int i = 0; i < 500; i++)
            count1++;
        count_lock.unlock();
    }
}

void lock_test2() {
    while (1) {
        count_lock.lock();
        for (int i = 0; i < 1000; i++)
            count2++;
        count_lock.unlock();
    }
}

void lock_test3() {
    while (1) {
        count_lock.lock();
        for (int i = 0; i < 100; i++)
            count3++;
        count_lock.unlock();
    }
}

int ratio_test(int a, int b, float r) {
    float f = (float)a / (float)b;
    if (a < b)
        f = 1.0 / f;
    if (f > r)
        return 1;
    return 0;
}

void showp() {
    Serial.print(p1);
    Serial.print(" ");
    Serial.print(p2);
    Serial.print(" ");
    Serial.print(p3);
    Serial.println();
}

int id1, id2, id3;

void delay2(uint32_t ms) {
    int mx = millis();
    while (millis() - mx < ms)
        ;
}

#define delayx delay

class subtest {
public:
    int value;
    void h(int x) { value = x; }
    int test(Thread::Mutex *lk) { return lk->getState(); }
    int getValue() { return value; }
} subinst;

class WireTest {
public:
    bool beginTransaction() { return 1; }
    bool endTransaction() { return 1; }
    bool other() { return 1; }
};

int fault_id = 0;
int stack_fault = 0;
int stack_id = 0;

void stack_overflow_isr(void) {
    stack_fault = 1;
    fault_id = Thread::id();
    Serial.printf("Overflow %d\n", fault_id);
    Thread::kill(fault_id);
    Thread::printStack(fault_id);
}

// turn off optimizations or the optimizer will remove the recursion because
// it is smart enough to know it doesn't do anything
void __attribute__((optimize("O0"))) recursive_thread(int level) {
    if (stack_fault)
        return;
    char x[128]; // use up some stack space
    delay(20);
    recursive_thread(level + 1);
}

void runtest() {
    int save_p;
    int save_time;
    float rate;

    // benchmark with no threading
    my_priv_func1(1);
    save_time = p1;

    Serial.print("CPU speed consistency ");
    my_priv_func1(1);
    rate = (float)p1 / (float)save_time;
    if (rate < 1.2 && rate > 0.8)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread start ");
    id1 = Thread::addThread(my_priv_func1, 1);
    delayx(300);
    if (p1 != 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread run state ");
    if (Thread::getState(id1) == Thread::RUNNING)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread return ");
    delayx(1000);
    save_p = p1;
    delayx(300);
    if (p1 != 0 && p1 == save_p)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread speed ");
    rate = (float)p1 / (float)save_time;
    if (rate < 0.7 && rate > 0.3)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Speed no threads: ");
    Serial.println(save_time);
    Serial.print("Speed 1 thread: ");
    Serial.println(p1);
    Serial.print("Ratio: ");
    Serial.println(rate);

    Serial.print("Test set time slice ");
    id1 = Thread::addThread(my_priv_func1, 1);
    delayx(2000);
    save_p = p1;
    id1 = Thread::addThread(my_priv_func1, 1);
    Thread::setTimeSlice(id1, 200);
    delayx(2000);
    float expected = (float)save_p * 2.0 * 200.0 / ((float)Thread::DEFAULT_TICKS + 200.0);
    rate = (float)p1 / (float)expected;
    if (rate > 0.9 && rate < 1.1)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Speed default ticks: ");
    Serial.println(save_p);
    Serial.print("Speed 200 ticks: ");
    Serial.println(p1);
    Serial.print("Expected: ");
    Serial.println(expected);
    Serial.print("Ratio with expected: ");
    Serial.println(rate);

    Serial.print("Test delay yield ");
    id1 = Thread::addThread(my_priv_func1, 1);
    Thread::delay(1100);
    rate = (float)p1 / (float)save_time;
    if (rate > 0.7 && rate < 1.4)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Yield wait ratio: ");
    Serial.println(rate);

    Serial.print("Test thread end state ");
    if (Thread::getState(id1) == Thread::ENDED)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread reinitialize ");
    p2 = 0;
    id2 = Thread::addThread(my_priv_func2);
    delayx(200);
    if (p2 != 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test stack usage ");
    int sz = Thread::getStackUsed(id2);
    // Seria.println(sz);
    if (sz >= 40 && sz <= 48)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread suspend ");
    delayx(200);
    Thread::suspend(id2);
    delayx(200);
    save_p = p2;
    delayx(200);
    if (p2 != 0 && p2 == save_p)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread restart ");
    p2 = 0;
    Thread::restart(id2);
    delayx(1000);
    if (p2 != 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    // Thread::kill(id2);
    delayx(200);

    Serial.print("Test thread stop ");
    Thread::stop();
    delayx(200);
    p2 = 0;
    delayx(200);
    if (p2 == 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread start ");
    Thread::start();
    delayx(200);
    if (p2 != 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Thread::kill(id2);

    Serial.print("Test thread wait ");
    id3 = Thread::addThread(my_priv_func1, 1);
    int time = millis();
    int r = Thread::wait(id3);
    delayx(100);
    time = millis() - time;
    if (r == id3)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread wait time ");
    if (time > 1000 && time < 2000)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread kill ");
    id3 = Thread::addThread(my_priv_func1, 2);
    delayx(300);
    Thread::kill(id3);
    delayx(300);
    save_p = p1;
    delayx(300);
    if (save_p == p1)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    delayx(1000);

    Serial.print("Test std::thread scope ");
    {
        std::thread th2(my_priv_func3);
        delayx(500);
    }
    delayx(500);
    save_p = p3;
    delayx(500);
    if (p3 != 0 && p3 == save_p)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test basic lock ");
    id1 = Thread::addThread(my_priv_func1, 2);
    delayx(500);
    {
        Thread::Suspend lock;
        save_p = p1;
        delayx(500);
        if (save_p == p1)
            Serial.println(OKAY_STR);
        else
            Serial.println(FAIL_STR);
    }

    Serial.print("Test basic unlock ");
    delayx(500);
    if (save_p != p1)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test mutex lock state ");
    Thread::Mutex mx;
    mx.lock();
    r = mx.try_lock();
    if (r == 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test mutex lock thread ");
    id1 = Thread::addThread(my_priv_func_lock, &mx);
    delayx(200);
    if (p4 == 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test mutex unlock ");
    mx.unlock();
    delayx(500);
    if (p1 != 0)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test fast locks ");
    id1 = Thread::addThread(lock_test1);
    id2 = Thread::addThread(lock_test2);
    id3 = Thread::addThread(lock_test3);
    delayx(3000);
    Thread::kill(id1);
    Thread::kill(id2);
    Thread::kill(id3);
    if (ratio_test(count1, count2, 1.2))
        Serial.println(FAIL_STR);
    // else if (ratio_test(count1, count3, 1.2)) Serial.println(FAIL_STR);
    else
        Serial.println(OKAY_STR);

    Serial.print(count1);
    Serial.print(" ");
    Serial.print(count2);
    Serial.print(" ");
    Serial.print(count3);
    Serial.println();

    Serial.print("Test std::mutex lock ");
    std::mutex g_mutex;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_mutex.try_lock() == 0)
            Serial.println(OKAY_STR);
        else
            Serial.println(FAIL_STR);
    }

    Serial.print("Test std::mutex unlock ");
    if (g_mutex.try_lock() == 1)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);
    g_mutex.unlock();

    Serial.print("Test Grab init ");
    subinst.h(10);
    ThreadWrap(subinst, sub2);
#define subinst ThreadClone(sub2)
    if (subinst.getValue() == 10)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test Grab set ");
    subinst.h(25);
    if (subinst.getValue() == 25)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test Grab lock ");
    if (subinst.test(&(sub2.getLock())) == 1)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);

    Serial.print("Test thread stack overflow ");
    uint8_t *mstack = new uint8_t[1024];
    stack_id = Thread::addThread(recursive_thread, 0, 512, mstack + 512);
    Thread::delay(2000);
    Thread::kill(stack_id);
    if (stack_fault)
        Serial.println(OKAY_STR);
    else
        Serial.println(FAIL_STR);
    delete[] mstack;
}

void runloop() {
    static int timeloop = millis();
    static int mx = 0;
    static int count = 0;
    if (millis() - mx > 5000) {
        Serial.print(count);
        Serial.print(": ");
        Serial.print((millis() - timeloop) / 1000);
        Serial.print(" sec ");
        showp();
        count++;
        mx = millis();
    }
}

extern unsigned long _heap_start;
extern unsigned long _heap_end;
extern char *__brkval;
int heap_size = (int)((char *)&_heap_end - __brkval);

auto hMux = Thread::Mutex();

float heapUsage() {
    Thread::Scope lock(hMux);
    return 100.0f * (heap_size - (int)((char *)&_heap_end - __brkval)) / heap_size;
}

// #include "malloc.h"

// // extern volatile void *__bss_end;

// int heapSize() {
//     // return __bss_end;
//     return mallinfo().arena - mallinfo().keepcost;
// }

volatile double *m;

void threadTest() {
    // auto t = Thread::addThread([](void *) {
    //     static int s = 0;
    //     while (1) {
    //         Thread::delay(1000);
    //         s += 8;
    //         char a[s]={0};
    //         if (s == 8)
    //             Thread::growStack(5);
    //         Serial.println("H");
    //         memset(&a, 0xEE, s);
    //     } }, 0, 256);
    // Thread::addThread([](int a) {
    //     while (1) {
    //         Serial.print(Thread::infoString());
    //         Thread::delay(500);
    //     } }, 0, 2048);
    Thread::addThread([](int a) {
        while (1) {
            if (heapUsage() >= 75)
                break;
            m = new double(2);
            m = new double(2);
            m = new double(2);
            m = new double(2);
            // m = (double *)malloc(2048);
            if (m != NULL)
                *m = 5.0;
            m = 0;
            Thread::yield();
        } }, 0, 1024, 0, "TEST4", 4);
    Thread::addThread([](int a) {
        static auto usage = heapUsage();
        while (1) {
            if (usage != heapUsage())
                Serial.println(usage = heapUsage());
            // Serial.println(mallinfo().arena);
            // Serial.println((int)(char *)&_heap_end);
            // Serial.println(mallinfo().fordblks);
            // Thread::delay(1);
            Thread::yield();
        } }, 0, 2048, 0, "TEST2", 2);
    Thread::addThread([](int a) {
        while (1) {
            // SerialUSB1.println(Thread::infoString()); // Use USB_DUAL_SERIAL
            Serial.println(Thread::infoString());
            Thread::delay(250);
        } }, 0, 2048, 0, "TEST2", 2);
    // while (1) {
    //     Thread::delay(100);
    //     Thread::printStack(1);
    //     if (Thread::growStack(t, 8) != -1) {
    //         Thread::printStack(1);
    //         Serial.println("Growing");
    //         break;
    //     }
    // }

    // Thread::wait(t);

    runtest();
    // Thread::kill(t);
    Serial.println("Test infinite loop (will not end)");
    while (1) {
        runloop();
    }
}