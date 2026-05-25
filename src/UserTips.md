# User Handbook

`setup.sh` 中代码 `set -euo pipefail` 由三部分组成：
1. `set -e` (`set -o eerexit`)：**任何命令失败**立即退出（包括管道中的任何环节）
2. `set -u` (`set -o nounset`)： **使用未定义变量**立即退出
3. `set -o pipefial`： **管道中任何子命令**失败都视为整个管道失败（e.g., `cmd1 | cmd2` = `true | false`）

在 [shell](#shell) （如bash） 中，**管道** (pipe) 是进程间通信机制，使用 `|` 将一个命令的**标准输出**（stdout） 直接连接到另一个命令的**标准输入**（stdin），从而实现多个命令的串联处理。

`cmd1 | cmd2 |cmd3` 
`cmd1` 的输出会作为 `cmd2` 的输入。
`cmd2` 的输出会作为 `cmd3` 的输入，依此类推。
(e.g., `ls -l | grep ".txt"` 中，`ls -l` 输出文件列表 $\rightarrow$ 传递给 `grep` $\rightarrow$ `grep` 只输出包含 .txt 的行)

**管道退出码**是最后一个命令的退出码。 `false | true` 的退出码为 0。


**<a name="shell">Shell</a>** 是操作系统中一个用户与内核交互的命令行界面（CLI）。核心功能是：
1. **解释执行用户输入的命令**：读取你键入的文本命令（如 `ls`, `cd`, `roslaunch`），将其解析并调用相应的程序或系统功能。
2. 提供脚本编程能力：支持变量、条件判断、循环、函数等，可编写 `.sh` 脚本批量执行任务。
3. 管理进程和 I/O 重定向：支持管道（`|`）、输入输出重定向（`>`、`<`）、后台运行（`&`）

常见的 Shell 类型：Bash (Bourne Again SHell)	Linux 默认 `Shell`，功能强大且兼容 `sh`。`Sh` (Bourne Shell)	最古老的 Unix Shell，脚本通常以 `#!/bin/sh` 开头。