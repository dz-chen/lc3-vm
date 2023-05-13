#include<stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <fcntl.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/termios.h>
#include <sys/mman.h>
/********************************************************************************
 * LC3指令集:https://www.jmeiners.com/lc3-vm/supplies/lc3-isa.pdf
 *  - 共10个寄存器: 8个通用寄存器(R0-R7); 1个程序计数器(PC); 1个条件标志位寄存器(COND)
 *  - 每个寄存器 16bit
 *  - LC3是大端(低地址,高字节); 若运行lc3-vm的机器为小端,则需进行转换；
 * 代码中多数 uint16_t 的变量只是用于表达位模式,实际存的数字可能是负数
 * 
 * ******************************************************************************/

//////////////////////////////////////////////////////////////////////////// definitions

enum REG{
    R_R0 = 0,
    R_R1,
    R_R2,
    R_R3,
    R_R4,
    R_R5,
    R_R6,
    R_R7,
    R_PC,       /* program counter */
    R_COND,     /* condition flags */
    R_COUNT
};
uint16_t reg[R_COUNT];

// operation code
enum OP{
    OP_BR = 0, /* 0:branch */
    OP_ADD,    /* add  */
    OP_LD,     /* load */
    OP_ST,     /* store */
    OP_JSR,    /* jump register */
    OP_AND,    /* bitwise and */
    OP_LDR,    /* load register */
    OP_STR,    /* store register */
    OP_RTI,    /* unused */
    OP_NOT,    /* bitwise not */
    OP_LDI,    /* load indirect */
    OP_STI,    /* store indirect */
    OP_JMP,    /* jump */
    OP_RES,    /* reserved (unused) */
    OP_LEA,    /* load effective address */
    OP_TRAP    /* 15:execute trap */
};

enum FLAG{
    FL_POS = 1 << 0, /* P, positive */
    FL_ZRO = 1 << 1, /* Z, zero     */
    FL_NEG = 1 << 2, /* N, negative */
};

/**
 * traps, see Table A.2
 */
enum {
    TRAP_GETC = 0x20,  /* get character from keyboard, not echoed onto the terminal */
    TRAP_OUT = 0x21,   /* output a character */
    TRAP_PUTS = 0x22,  /* output a word string */
    TRAP_IN = 0x23,    /* get character from keyboard, echoed onto the terminal */
    TRAP_PUTSP = 0x24, /* output a byte string */
    TRAP_HALT = 0x25   /* halt the program */
};

/**
 * Memory address space 16 bits, corresponding to 216 locations, each
 * containing one word (16 bits). Addresses are numbered from 0 (i.e, x0000)
 * to 65,535 (i.e., xFFFF).
 * 注意 LC3 这里是按机器字编址!!
 */
uint16_t memory[__UINT16_MAX__];


/* Memory Mapped Registers */
enum {
    MR_KBSR = 0xFE00, /* keyboard status */
    MR_KBDR = 0xFE02  /* keyboard data */
};


//////////////////////////////////////////////////////////////////////////// tool func
/** 
 * extend lowest cnt bits to uint16
 * refer to csapp for sign extend: copy highest bit to all higher bits
 */
uint16_t sign_exetend(uint16_t x, int cnt){
    if((x>>(cnt-1)) & 0x1){   /* negative */
        return x | (0xffff<<cnt);
    } else{                 /* positive or zero */
        return x;
    }
}

/* change little endian to big endian */
uint16_t swap16(uint16_t x){
    return (x << 8) | (x >> 8);
}

/* x: result of the operation before...*/
void update_flag(uint16_t x){
    if(x==0) {
        reg[R_COND] = FL_ZRO;
    } else if(x >>15){   
        reg[R_COND] = FL_NEG;
    } else {
        reg[R_COND] = FL_POS;
    }
}

/**
 * TODO:这里读取可执行程序为何如此简单,与ELF 差异这么大?
 * load executable image file into lc3 memory
 */ 
void read_image_file(const char* path){
    FILE* file = fopen(path,"rb");
    if(!file){
        printf("error: failed to read image file %s",path);
        return;
    }

    uint16_t origin;        /* where in memory to place the image */
    fread(&origin, sizeof(origin), 1, file);
    origin = swap16(origin);

    /* we know the maximum file size so we only need one fread */
    uint16_t max_read = __UINT16_MAX__ - origin;
    uint16_t *p = memory + origin;
    size_t read = fread(p, sizeof(uint16_t), max_read, file);

    /* swap to little endian */
    while (read--) {
        *p = swap16(*p);
        ++p;
    }

    fclose(file);
}





uint16_t check_key() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    return select(1, &readfds, NULL, NULL, &timeout) != 0;
}

/**
 * lc3 instruction read memory 
 * 注意memory是按机器字(uint16_t)为单位编址,addr 代表取第addr个 uint16_t
 */
uint16_t mem_read(uint16_t address){
    /**
     * MMR
     * 某些特殊类型的寄存器是无法从常规寄存器表（register table）中访问;
     * 内存中为这些寄存器预留了特殊的地址，要读写这些寄存器，只需要读写相应的内存地址;
     * 这些称为 内存映射寄存器(MMR).内存映射寄存器通常用于处理与特殊硬件的交互;
     * LC-3 有两个内存映射寄存器需要实现:
     * KBSR: 键盘状态寄存器（keyboard status register），表示是否有键按下
     * KBDR: 键盘数据寄存器（keyboard data register），表示哪个键按下了
     */ 
    if (address == MR_KBSR) {
        if (check_key()) {
            memory[MR_KBSR] = (1 << 15);
            memory[MR_KBDR] = getchar();
        } else {
            memory[MR_KBSR] = 0;
        }
    }
    return memory[address];
}

/*  lc3 instruction write memory */
void mem_write(uint16_t address,uint16_t val){
    memory[address] = val;
}

/////////  misc funcs ....
/* Input Buffering */
struct termios original_tio;

void disable_input_buffering() {
    tcgetattr(STDIN_FILENO, &original_tio);
    struct termios new_tio = original_tio;
    new_tio.c_lflag &= ~ICANON & ~ECHO;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
}

void restore_input_buffering() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_tio);
}

/* Handle Interrupt */
void handle_interrupt(int signal) {
    restore_input_buffering();
    printf("\n");
    exit(-2);
}


//////////////////////////////////////////////////////////////////////////// op func
uint16_t instr;

/* will uint + uint overflow? */
void do_add(uint16_t instr){
    uint16_t dr = (instr>>9) & 0x7;     /* 0x7 is unsigned int */
    uint16_t sr1 = (instr>>6) & 0x7;

    if((instr>>5) & 0x1) {        /* imm mode */
        /* imm may be negative or zero or positive*/
        uint16_t imm = sign_exetend(instr & 0x1f, 5);
        reg[dr] = reg[sr1] + imm;
    } else{                      /* reg mode */
        uint16_t sr2 = instr &0x7;
        reg[dr] = reg[sr1] + reg[sr2];
    }

    update_flag(reg[dr]);
}

/* bitwise and*/
void do_and(uint16_t instr){
    uint16_t dr = (instr>>9) & 0x7;     /* 0x7 is unsigned int */
    uint16_t sr1 = (instr>>6) & 0x7;

    if((instr>>5) & 0x1) {        /* imm mode */
        /* imm may be negative or zero or positive*/
        uint16_t imm = sign_exetend(instr & 0x1f, 5);
        reg[dr] = reg[sr1] & imm;
    } else{                      /* reg mode */
        uint16_t sr2 = instr &0x7;
        reg[dr] = reg[sr1] & reg[sr2];
    }

    update_flag(reg[dr]);
}

/* bitwise not */
void do_not(uint16_t instr){
    uint16_t dr = (instr>>9) & 0x7;     /* 0x7 is unsigned int */
    uint16_t sr = (instr>>6) & 0x7;
    reg[dr] = ~reg[sr];
    update_flag(reg[dr]);
}

/**
 * branch:条件跳转,适用于if...else...
 */ 
void do_br(uint16_t instr){
    uint16_t offset = sign_exetend(instr&0x1ff,9);
    uint16_t i_cond = (instr>>9) & 0x7;
    /**
     * key point:
     * if ( (n&N) || (z&Z) || (p&P) ) , then pc += offset
     * where n is bit[11] of instr, N is bit[2] of condition register
     * etc...
     */ 
    if(i_cond & reg[R_COND]){
        reg[R_PC] += offset;
    }
}

/**
 * jump:无条件跳转,适用于 goto,return,break,continue等
 * unconditionally jump, JMP && RET
 * key point: 
 * The RET instruction is a special case of the JMP instruction. The PC is loaded
 * with the contents of R7, which contains the linkage back to the instruction
 * following the subroutine call instruction 
*/
void do_jmp(uint16_t instr){
    uint16_t r = (instr>>6) & 0x7;
    reg[R_PC] = reg[r];
}

/**
 * jsr(jump to subroutine)无条件跳转:适用于函数调用
 * 注意函数调用原理:
 * First, the incremented PC is saved in R7(见ret指令). This is the linkage back to the calling
 * routine. Then the PC is loaded with the address of the first instruction of the
 * subroutine, causing an unconditional jump to that address
 */ 
void do_jsr(uint16_t instr){
    reg[R_R7] = reg[R_PC];
    uint16_t flag = (instr >> 11) & 0x1;
    if(flag){               // jsr,即pc相对寻址
        uint16_t offset = sign_exetend(instr & 0x7ff,11);
        reg[R_PC] = reg[R_PC] + offset;
    } else{                 // jsrr,即绝对寻址
        uint16_t r = (instr >> 6) & 0x7;
        reg[R_PC] = reg[r];
    }
}

/**
 * load
 * 涉及到读内存,注意虚拟化时如何模拟读/写内存
 */ 
void do_ld(uint16_t instr){
    uint16_t offset = sign_exetend(instr & 0x1ff,9);
    uint16_t dr = (instr>>9) & 0x7;
    reg[dr] = mem_read(reg[R_PC]+offset);
    update_flag(reg[dr]);
}

/* load indrect */
void do_ldi(uint16_t instr){
    uint16_t offset = sign_exetend(instr&0x1ff,9);
    uint16_t dr = (instr>>9) & 0x7;
    reg[dr] = mem_read(mem_read(reg[R_PC]+offset));
    update_flag(reg[dr]);
}

/* load register*/
void do_ldr(uint16_t instr){
    uint16_t offset = sign_exetend(instr&0x3f, 6);
    uint16_t baser = (instr>>6) & 0x7;
    uint16_t dr = (instr>>9) & 0x7;
    reg[dr] = mem_read(reg[baser] + offset);
    update_flag(reg[dr]);
}

/**
 * lea: load effective address
 * 注: 实际上它根本没有引用内存!! 并不是从指定的位置读入数据,而是将有效地址写入到目的寄存器;
 * 注意与mov的区别
*/
void do_lea(uint16_t instr){
    uint16_t offset = sign_exetend(instr&0x1ff,9);
    uint16_t dr = (instr >> 9) & 0x7;
    reg[dr] = reg[R_PC] + offset;
    update_flag(reg[dr]);
}

/* store */
void do_st(uint16_t instr){
    uint16_t offset = sign_exetend(instr&0x1ff,9);
    uint16_t sr = (instr >> 9) & 0x7;
    mem_write(reg[R_PC] + offset,reg[sr]);
}

/* store indirect */
void do_sti(uint16_t instr){
    uint16_t offset = sign_exetend(instr&0x1ff,9);
    uint16_t sr = (instr >> 9) & 0x7;
    uint16_t addr = mem_read(reg[R_PC]+offset);
    mem_write(addr,reg[sr]);
}

/* store register */
void do_str(uint16_t instr){
    uint16_t offset = sign_exetend(instr & 0x3f, 6);
    uint16_t baser = (instr>>6) & 0x7;
    uint16_t sr = (instr>>9) & 0x7;
    mem_write(reg[baser]+offset, reg[sr]);
}

/**/
void do_rti(uint16_t instr){
    printf("error:rti is not implemented");
    abort();
}

/**/
void do_res(uint16_t instr){
    printf("error:res is not implemented");
    abort();
}

/* trap: see trap func bellow... */
//////////////////////////////////////////////////////////////////////////// trap func
/**
 * GETC
 * Read a single character from the keyboard. The character is not echoed
 * onto the console. Its ASCII code is copied into R0. The high eight bits
 * of R0 are cleared.
 */
void trap_get(){
    reg[R_R0] = (uint16_t) getchar();
    update_flag(reg[R_R0]);
}

/**
 * IN
 * Print a prompt on the screen and read a single character from the keyboard. The
 * character is echoed onto the console monitor, and its ASCII code is copied into R0.
 * The high eight bits of R0 are cleared.
 */ 
void trap_in(){
    printf("Enter a character >");
    char c = getchar();
    putc(c, stdout);
    reg[R_R0] = (uint16_t)c;
    fflush(stdout);
    update_flag(reg[R_R0]);
}

/**
 * OUT
 * Write a character in R0[7:0] to the console display.
 */ 
void trap_out(){
    putc((char)reg[R_R0], stdout);
    fflush(stdout);
}

/**
 * PUTS
 * Write a string of ASCII characters to the console display. The characters are 
 * contained in consecutive memory locations, one character per memory location, 
 * starting with the address specified in R0. Writing terminates with the occurrence 
 * of x0000 in a memory location.
 */ 
void trap_puts(){
    uint16_t addr = reg[R_R0];
    uint16_t c = mem_read(addr++);
    while (c){
        putc((char)c, stdout);
        c = mem_read(addr++);
    }
    fflush(stdout);
}

/**
 * PUTSP
 * Write a string of ASCII characters to the console. The characters are contained in
 * consecutive memory locations, two characters per memory location, starting with the
 * address specified in R0. The ASCII code contained in bits [7:0] of a memory location
 * is written to the console first. Then the ASCII code contained in bits [15:8] of that
 * memory location is written to the console. (A character string consisting of an odd
 * number of characters to be written will have x00 in bits [15:8] of the memory
 * location containing the last character to be written.) Writing terminates with the
 * occurrence of x0000 in a memory location.
 * 注意大端 小端
 */ 
void trap_putsp(){
    uint16_t addr = reg[R_R0];
    uint16_t c = mem_read(addr++);
    while (c){
        char ch1 = (char) c&0xff;
        putc(ch1, stdout);
        char ch2 =(char) c>>8;
        if(ch2) putc(ch2,stdout);
        c = mem_read(addr++);
    }
    fflush(stdout);
}

/**
 *  Halt execution and print a message on the console.
 */ 
void trap_halt(){
    puts("HALT");
    fflush(stdout);
}

/**
 *  trap 
 * 注意需要提前设置 R_7,用于从trap中返回!!!
 */
void do_trap(uint16_t instr){
    reg[R_R7] = reg[R_PC];
    switch (instr & 0xFF) {
        case TRAP_GETC: trap_get(); break;
        case TRAP_OUT: trap_out(); break;
        case TRAP_PUTS: trap_puts(); break;
        case TRAP_IN: trap_in(); break;
        case TRAP_PUTSP: trap_putsp(); break;
        case TRAP_HALT: trap_halt(); break;
    }
}

//////////////////////////////////////////////////////////////////////////// main
int main(int argc, const char* argv[]) {
    if(argc!=2){
        printf("usage: lc3-vm path");
        exit(-1);
    }
    read_image_file(argv[1]);

    /**
     *  Setup
     * SIGINT:程序终止(interrupt)信号, 在用户键入INTR字符(通常是Ctrl-C)时发出,用于通知前台进程组终止进程;
     * 
     */
    signal(SIGINT, handle_interrupt);
    disable_input_buffering();  // TODO


    /* since exactly one condition flag should be set at any given time, set the Z flag */
    reg[R_COND] = FL_ZRO;

    /* set the PC to starting position */
    reg[R_PC] = 0x3000;       /* 0x3000 is the default */

    int running = 1;
    // test by dingzhi
    int instr_cnt = 0;
    while (running) {
        uint16_t instr = mem_read(reg[R_PC]++); /* FETCH */
        uint16_t op = instr >> 12;
        
        instr_cnt += 1;
        // test by dingzhi
        // printf("execute instr %d, instr:%u ; op:%u \n",instr_cnt, instr,op);

        switch (op) {
            case OP_ADD:
                do_add(instr); break;
            case OP_AND: 
                do_and(instr); break;
            case OP_NOT:
                do_not(instr); break;
            case OP_BR: 
                do_br(instr); break;
            case OP_JMP: 
                do_jmp(instr); break;
            case OP_JSR:
                do_jsr(instr); break;
            case OP_LD: 
                do_ld(instr); break;
            case OP_LDI: 
                do_ldi(instr); break;
            case OP_LDR: 
                do_ldr(instr); break;
            case OP_LEA:
                do_lea(instr); break;
            case OP_ST: 
                do_st(instr); break;
            case OP_STI: 
                do_sti(instr); break;
            case OP_STR: 
                do_str(instr); break;
            case OP_TRAP:
                do_trap(instr); 
                if (instr & 0xFF == TRAP_HALT)
                    running = 0;
                break;
            case OP_RES:
                do_res(instr); break;
            case OP_RTI:
                do_rti(instr); break;
            default:
                printf("error: unrecongnized instruntion %ud",op);
                abort();
                break;
        }
    }
    /* Shutdown */
    restore_input_buffering();
}
