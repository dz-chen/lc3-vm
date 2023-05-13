[toc]
# lc3-vm 概述
**1.参考**  
https://www.jmeiners.com/lc3-vm/
https://people.cs.georgetown.edu/~squier/Teaching/HardwareFundamentals/LC3-trunk/docs/README-LC3tools.html


**2.编译**  
```
gcc -g my-lc3-vm.c -o my-lc3-vm
```

**3.执行**
```
./my-lc3-vm 2048.obj
```

**4.说明**  
1.这里的虚拟机只是一个指令解释器/模拟器，并非VMware、VirtualBox之类带有host操作系统的虚拟机；
2.给出的.obj文件没有考虑操作系统的存在，与ELF这类os平台相关的可执行文件是存在区别的（ELF有复杂的文件格式，且涉及虚拟地址； 而这里.obj格式简单，全是物理地址）；


# 调试
```
gdb my-lc3-vm
set args rogue.obj
b main
run

```