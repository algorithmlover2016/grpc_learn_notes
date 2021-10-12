In Windows cmd type 
to kill by name:
taskkill /IM "tensorboard.exe" /F
to kill by process number:
taskkill /F /PID proc_num

In Windows powershell type
to kill by name:
Stop-Process -Name "ProcessName" -Force
to kill by process number(pid, proc_num)
Stop-Process -ID PID -Force
