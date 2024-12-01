void _exit(int status) __attribute__((weak));
void _exit(int status) 
{
    while (1);
}

int main(void) 
{
    while (1) {
        // 主循环代码，例如LED闪烁、串口通信等
        // ...
    }
}