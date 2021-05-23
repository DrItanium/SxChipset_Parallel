//
// Created by jwscoggins on 5/2/21.
//
#include <iostream>
#include "IODevice.h"
int wait(int count) {
    int result = 0;
    for (int i = 0; i < count; ++i) {
        result += i;
    }
    return result;
}
int main() {
    BuiltinLED theLed(0);
    uint8_t pwmIndex = 127;
    volatile uint16_t& con = memory<uint16_t>(getIOBase0Address(0x100));
    volatile uint16_t& pwm = memory<uint16_t>(getIOBase0Address(0x200));
    const char* msg = "hello, world\n";
    for(const char* ptr = msg; *ptr; ++ptr) {
        con = *ptr;
    }
    con = 'd';
    con = 'o';
    con = 'n';
    con = 'u';
    con = 't';
    con = 's';
    con = '\n';
    while(true) {
        pwm = pwmIndex;
        theLed.toggle();
        wait(1000);
        theLed.toggle();
        wait(1000);
        ++pwmIndex;
    }
    return 0;
}

extern "C"
int atexit(void (*function)(void))
{

}

extern "C"
ssize_t write(int fildes, const void* buf, size_t nbyte) {
    const char* theBuf = (const char*)buf;
    volatile uint16_t& con = memory<uint16_t>(getIOBase0Address(0x100));
    for (size_t i = 0; i < nbyte; ++i) {
        con = (uint16_t)theBuf[i];
    }
    return nbyte;
}

extern "C"
ssize_t read(int fildes, void* buf, size_t nbyte) {
    char* theBuf = (char*)buf;
    volatile uint16_t& con = memory<uint16_t>(getIOBase0Address(0x100));
    for (int i = 0; i < nbyte; ++i) {
        theBuf[i] = (char)con;
    }
    return nbyte;
}
extern "C"
int open(const char* path, int oflag, ...) {
    // fake it for now
    return 0;
}

#if 0
extern "C"
void abort() {
    std::cout << "Aborting Execution!" << std::endl;
    while (true) {
        // do thing
    }
    // do nothing
}
extern "C"
off_t lseek(int fildes, off_t offset, int whence) {
    return 0;
}

extern "C"
int fstat(int fildes, struct stat* buf) {
   return 0;
}


extern "C"
void* sbrk(intptr_t increment) {
    return 0;
}

extern "C"
int close(int fildes) {
    return 0;
}

extern "C"
int isatty(int fd) {
    return 0;
}
#endif
